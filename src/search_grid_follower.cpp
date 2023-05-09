#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <eigen3/Eigen/Geometry>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::placeholders;

class SearchGridFollower : public rclcpp::Node {
    public:
    SearchGridFollower() : Node("search_grid_follower"), progress(0u), is_active(false), is_on_pattern(true) {
        this->declare_parameter("arena_mode", false);
        arena_mode = this->get_parameter("arena_mode").as_bool();
        if(arena_mode) readGridFromFile("/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/searchgrid_arena.csv");
        else readGridFromFile("/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/searchgrid_spot_small.csv");

        

        this->active_pub = this->create_publisher<std_msgs::msg::Bool>("is_active", 1);
        this->on_pattern_pub = this->create_publisher<std_msgs::msg::Bool>("is_on_pattern", 1);

        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"/navigate_to_pose");
        if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) throw std::runtime_error("Cannot find nav2 stack ...");
        set_active_service = this->create_service<std_srvs::srv::SetBool>(
                "/activate_search_grid_follower",
                std::bind(&SearchGridFollower::set_active_service_callback, this, _1, _2, _3)
        );

        



        
    }
    private:

    void send_goal(){
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

        if(is_on_pattern){
            goal_msg.pose.pose.position.x = buffer.at(progress).x();
            goal_msg.pose.pose.position.y = buffer.at(progress).y();
            goal_msg.pose.pose.orientation.w = std::cos(buffer.at(progress).z() / 2.0);
            goal_msg.pose.pose.orientation.z = std::sin(buffer.at(progress).z() / 2.0);
            RCLCPP_INFO(this->get_logger(), "Following next WP (nr. %lu) in search grid ...", progress);
        }
        else{
            goal_msg.pose = on_exit;
            RCLCPP_INFO(this->get_logger(), "Going back to search grid ...");


        }
        


        goal_msg.pose.header.frame_id = "arena";

        RCLCPP_INFO(this->get_logger(), "Sending goal %f %f",buffer.at(0).x(),buffer.at(0).y());

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&SearchGridFollower::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&SearchGridFollower::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&SearchGridFollower::result_callback, this, _1);
        client_ptr_->async_send_goal(goal_msg, send_goal_options);


    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        if(is_on_pattern) on_exit = feedback->current_pose;
        RCLCPP_DEBUG(this->get_logger(), "Time spent on Nav goal: %f, time left: %f", feedback->navigation_time, feedback->estimated_time_remaining);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        if(is_active && is_on_pattern) progress++;
        if(is_active && !is_on_pattern) is_on_pattern = true;

        if(is_active) send_goal();        
    }
    void set_active_service_callback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if(request->data == this->is_active) RCLCPP_WARN(this->get_logger(), "Want to turn on/off search grid follower but requested (%s) state is already set", is_active ? "active" : "inactive");
        else{
            this->is_active = request->data;
            std_msgs::msg::Bool msg;
            msg.data = this->is_active;
            this->active_pub->publish(msg);
            
            if(this->is_active) send_goal();
            else {
                this->is_on_pattern = false;
                this->client_ptr_->async_cancel_all_goals();
            }
            RCLCPP_INFO(this->get_logger(), "Changed mode successfully to %s!", is_active ? "active" : "inactive");
        }
        response->success = true;
    }



    void readGridFromFile(std::string filename = "/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/searchgrid_arena.csv"){
        buffer.clear();
        std::ifstream file(filename);

        std::string line;
        
        while (getline(file, line)) {
            std::vector<std::string> row;
            std::stringstream ss(line);
            std::string cell;
            Eigen::Vector3d to_add(Eigen::Vector3d::Zero());
            size_t i = 0;
            while (getline(ss, cell, ',')) {
                to_add[i] = std::stod(cell);
                i++;
            }
            to_add.z() *= M_PI / 180.;
            buffer.push_back(to_add);
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Read %lu waypoints from file %s", buffer.size(), filename.c_str());
    }

    bool is_active;
    bool is_on_pattern;
    std::vector<Eigen::Vector3d> buffer;
    geometry_msgs::msg::PoseStamped on_exit;
    size_t progress;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub, on_pattern_pub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_active_service;

    bool arena_mode;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    SearchGridFollower::SharedPtr sgf(new SearchGridFollower());
    rclcpp::spin(sgf);

}