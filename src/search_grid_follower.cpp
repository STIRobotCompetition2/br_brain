#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <nav2_msgs/action/follow_waypoints.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <eigen3/Eigen/Geometry>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <br_brick_management/srv/brick_detection.hpp>


#define OFFLOAD_X 0.5
#define OFFLOAD_Y 0.5
#define OFFLOAD_YAW 315


using namespace std::placeholders;

enum Nav2CommanderState{
    SEARCHING,
    COLLECTING,
    SEARCHING_TO_COLLECTING
};

class Nav2Commander : public rclcpp::Node {
    public:
    Nav2Commander() : Node("nav2_commander") {
        this->declare_parameter("arena_mode", true);
        arena_mode = this->get_parameter("arena_mode").as_bool();
        if(arena_mode) readGridFromFile("/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/searchgrid_arena.csv");
        else readGridFromFile("/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/searchgrid_spot_small.csv");

        

        this->state_pub = this->create_publisher<std_msgs::msg::String>("/nav2_commander_state", 1);
        this->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this,"/follow_waypoints");
        while(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2u))) RCLCPP_WARN(this->get_logger(), "Cannot find nav2 stack ...");
        set_active_service = this->create_service<std_srvs::srv::SetBool>(
                "/activate_search_grid_trajectory_follower",
                std::bind(&Nav2Commander::set_active_service_callback, this, _1, _2, _3)
        );
        brick_detection_service = this->create_service<br_brick_management::srv::BrickDetection>("/brick_detection", std::bind(&Nav2Commander::brick_detection_callback, this, _1, _2, _3));
        set_brick_search_active = this->create_client<std_srvs::srv::Trigger>("set_active");
        while(!set_brick_search_active->wait_for_service(std::chrono::seconds(2u))){
            RCLCPP_WARN(this->get_logger(), "Brick-detection not yet online");
        }
        RCLCPP_INFO(this->get_logger(), "brick-detection ONLINE !");  

        state_pub_timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Nav2Commander::pubState, this));

        activate_search_grid_trajectory();
    }
    private:

    void activate_search_grid_trajectory(){
        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();

        if(state != Nav2CommanderState::SEARCHING){
            RCLCPP_FATAL(this->get_logger(), "Illegal state combination occured");
        }
        else if(current_wp >= this->search_grid_trajectory.size()) {
            RCLCPP_INFO(this->get_logger(), "***** DONE *****");
            return;
        }
        else
            for(size_t i = current_wp; i < search_grid_trajectory.size(); i++)
                goal_msg.poses.push_back(this->search_grid_trajectory.at(i));
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&Nav2Commander::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&Nav2Commander::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Nav2Commander::result_callback, this, _1);
        client_ptr_->async_send_goal(goal_msg, send_goal_options);


    }

    void activate_brick_collection(){
        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();

        if(state != Nav2CommanderState::COLLECTING){
            RCLCPP_FATAL(this->get_logger(), "Illegal state combination occured");
        }

        goal_msg.poses = collect_trajectory;
        
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&Nav2Commander::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&Nav2Commander::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Nav2Commander::result_callback, this, _1);
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback)
    {
        if(state == Nav2CommanderState::SEARCHING) current_wp = current_wp == 0u ? feedback->current_waypoint;
        // RCLCPP_DEBUG(this->get_logger(), "Time spent on Nav goal: %f, time left: %f", feedback->navigation_time, feedback->estimated_time_remaining);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult & result)
    {

        if(state == Nav2CommanderState::SEARCHING_TO_COLLECTING){
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED || result.code == rclcpp_action::ResultCode::CANCELED){
                RCLCPP_INFO(this->get_logger(), "Quit Search Grid following successfully. Collecting bricks now ...");
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Search grid following quit not as planned. Trying to collect bricks anyway ...");

            }
            state = Nav2CommanderState::COLLECTING;
            activate_brick_collection();
        }
        else if(state == Nav2CommanderState::SEARCHING){
            if(result.code != rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_ERROR(this->get_logger(), "Search grid following exited prematurely. Try to re-enter search grid by skipping last waypoint ...");
                current_wp += 1u;
                state =  Nav2CommanderState::SEARCHING;
                activate_search_grid_trajectory();
            }
            else{
                current_wp = std::numeric_limits<size_t>::max();
                RCLCPP_INFO(this->get_logger(), "\n\n**** !DONE! ****\n\n");
            }
        }
        else if(state == Nav2CommanderState::COLLECTING){
            if(result.code != rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_ERROR(this->get_logger(), "Brick Collection ended unsuccessfully. Going back to search grid & change mode of brick-detection ...");
                
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Unloading brick ...");
                geometry_msgs::msg::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = -0.2;
                this->cmd_vel_pub->publish(cmd_vel_msg);
                rclcpp::sleep_for(std::chrono::seconds(10u));
                cmd_vel_msg.linear.x = 0.0;
                this->cmd_vel_pub->publish(cmd_vel_msg);
                RCLCPP_INFO(this->get_logger(), "Brick Collection ended successful. Going back to search grid & change mode of brick-detection ...");
                
            }
            state = Nav2CommanderState::SEARCHING;
            std_srvs::srv::Trigger::Request::SharedPtr set_active_req(new std_srvs::srv::Trigger::Request());
            this->set_brick_search_active->async_send_request(set_active_req);
            activate_search_grid_trajectory();
        }


    }

    void brick_detection_callback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<br_brick_management::srv::BrickDetection::Request> request,
        const std::shared_ptr<br_brick_management::srv::BrickDetection::Response> response
    ){
        if(state != Nav2CommanderState::SEARCHING){
            RCLCPP_FATAL(this->get_logger(), "Invalid state config occured (request to collect another brick while already collecting brick)");
            response->success = false;
            return;


        }
        collect_trajectory.clear();

        collect_trajectory.push_back(request->detection);
        // trajectory.end()->header.frame_id = "arena";
        geometry_msgs::msg::PoseStamped offload_pose;
        offload_pose.header.frame_id = "arena";
        offload_pose.pose.position.x = OFFLOAD_X;
        offload_pose.pose.position.y = OFFLOAD_Y;
        offload_pose.pose.orientation.w = std::cos(OFFLOAD_YAW * M_PI / 180.);
        offload_pose.pose.orientation.z = std::sin(OFFLOAD_YAW * M_PI / 180.);
        collect_trajectory.push_back(offload_pose);
        collect_trajectory.push_back(offload_pose);

        RCLCPP_INFO(this->get_logger(), "Received new brick detection at (%f,%f). Built collection trajectory of %u poses. Canceling current navigation goal to exit search mode ...", request->detection.pose.position.x, request->detection.pose.position.y, collect_trajectory.size());
        state = Nav2CommanderState::SEARCHING_TO_COLLECTING;

        this->client_ptr_->async_cancel_all_goals();

        response->success = true;
    }

    void set_active_service_callback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        response->success = true;
    }



    void readGridFromFile(std::string filename = "/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/searchgrid_arena.csv"){
        search_grid_trajectory.clear();
        std::ifstream file(filename);

        std::string line;
        
        while (getline(file, line)) {
            std::vector<std::string> row;
            std::stringstream ss(line);
            std::string cell;
            Eigen::Vector3d to_add_eigen(Eigen::Vector3d::Zero());
            size_t i = 0;
            while (getline(ss, cell, ',')) {
                to_add_eigen[i] = std::stod(cell);
                i++;
            }
            to_add_eigen.z() *= M_PI / 180.;
            geometry_msgs::msg::PoseStamped to_add_pose_stamped;
            to_add_pose_stamped.header.frame_id = "arena";
            to_add_pose_stamped.pose.position.x = to_add_eigen.x();
            to_add_pose_stamped.pose.position.y = to_add_eigen.y();
            to_add_pose_stamped.pose.orientation.z = std::sin(0.5 * to_add_eigen.z());
            to_add_pose_stamped.pose.orientation.w = std::cos(0.5 * to_add_eigen.z());
            search_grid_trajectory.push_back(to_add_pose_stamped);
        }
        file.close();
        current_wp = 0u;
        RCLCPP_INFO(this->get_logger(), "Read %lu waypoints from file %s", current_wp, filename.c_str());
    }

    std_msgs::msg::String state2stringmsg(){
        std::string state_str;
        switch(this->state){
            case Nav2CommanderState::SEARCHING:
            state_str = std::string("searching");
            break;
            case Nav2CommanderState::COLLECTING:
            state_str = std::string("collecting");
            break;
            case Nav2CommanderState::SEARCHING_TO_COLLECTING:
            state_str = std::string("transition ongoing: searching to collection");
            break;
            default:
            throw std::runtime_error("nav2 commander found unknown state");
        }
        std_msgs::msg::String out_msg;
        out_msg.data = state_str;
        return out_msg;
    }

    void pubState(){
        state_pub->publish(state2stringmsg());
    }

    Nav2CommanderState state = Nav2CommanderState::SEARCHING;
    std::vector<geometry_msgs::msg::PoseStamped> search_grid_trajectory, collect_trajectory;
    size_t current_wp;

    geometry_msgs::msg::PoseStamped on_exit;

    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr client_ptr_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_active_service;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_brick_search_active;
    rclcpp::Service<br_brick_management::srv::BrickDetection>::SharedPtr brick_detection_service;
    rclcpp::TimerBase::SharedPtr state_pub_timer;

    bool arena_mode;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    Nav2Commander::SharedPtr sgf(new Nav2Commander());
    rclcpp::spin(sgf);

}