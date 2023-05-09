#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robot_parameters.h"

using namespace std::placeholders;


class BrickCollectorBase{
    public:
    BrickCollectorBase(
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr& navigate_client_ptr_,
        rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr& compute_paths_client_ptr_,
        RobotParameters robot_params
    ){
        this->navigate_client_ptr_ = navigate_client_ptr_;
        this->compute_paths_client_ptr_ = compute_paths_client_ptr_;
        this->robot_parameters = robot_params;

        
    
    }

    bool processQuery(const std::shared_ptr<grid_map::GridMap> obstacle_map, geometry_msgs::msg::PoseStamped brickPose){

        nav2_msgs::action::ComputePathThroughPoses::Goal goal_msg = nav2_msgs::action::ComputePathThroughPoses::Goal();
        goal_msg.use_start = false;

        updateTrajectory(obstacle_map, brickPose);

        goal_msg.goals = trajectory;
        last_code = rclcpp_action::ResultCode::UNKNOWN;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&SearchGridFollower::goal_response_callback, this, _1);

        auto callback = [&](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result)mutable{this->last_code = result.code;};

        send_goal_options.result_callback = std::bind(&BrickCollectorBase::computeTrajectoryResultCallback, this, _1);
        
        auto future_goal_handle = compute_paths_client_ptr_->async_send_goal(goal_msg, send_goal_options);
        
        while(last_code == rclcpp_action::ResultCode::UNKNOWN) rclcpp::sleep_for(std::chrono::milliseconds(10));
        return last_code == rclcpp_action::ResultCode::SUCCEEDED;
    };


    bool executeQuery(){
        if(trajectory.size() == 0u) {
            std::cerr << "No trajectory cached - Process it firstly" << std::endl;
            return false;
        }
        nav2_msgs::action::NavigateThroughPoses::Goal goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();


        goal_msg.poses = trajectory;
        last_code = rclcpp_action::ResultCode::UNKNOWN;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&SearchGridFollower::goal_response_callback, this, _1);
        send_goal_options.result_callback = std::bind(&BrickCollectorBase::executeTrajectoryResultCallback, this, _1);
        send_goal_options.feedback_callback = std::bind(&BrickCollectorBase::executeTrajectoryFeedbackCallback, this, _1, _2);



        navigate_client_ptr_->async_send_goal(goal_msg, send_goal_options);

        
        while(last_code == rclcpp_action::ResultCode::UNKNOWN){
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        return last_code == rclcpp_action::ResultCode::SUCCEEDED;
    }

    

    void setOffloadPose(const geometry_msgs::msg::PoseStamped offloadPose){
        this->offload_pose = offloadPose;
    }



    protected:
    virtual void updateTrajectory(const std::shared_ptr<grid_map::GridMap> obstacle_map, geometry_msgs::msg::PoseStamped brickPose) = 0;

    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_client_ptr_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr compute_paths_client_ptr_;
    geometry_msgs::msg::PoseStamped offload_pose;
    RobotParameters robot_parameters;
    std::vector<geometry_msgs::msg::PoseStamped> trajectory;
    rclcpp_action::ResultCode last_code;
    
    private:
    void computeTrajectoryResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathThroughPoses>::WrappedResult & result){
        this->last_code = result.code;
    }

    void executeTrajectoryResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result){
        this->last_code = result.code;
    }

    void executeTrajectoryFeedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback){
        // RCLCPP_DEBUG(ros_logger.get(), "Navigation Status: %f m remaining, %f s remaining, %lu poses remaining, %f s passed", feedback.distance_remaining, feedback.estimated_time_remaining, feedback.number_of_poses_remaining, feedback.navigation_time);
        std::cerr << "feedback" << std::endl;
    }

    private:
    std::shared_ptr<rclcpp::Logger> ros_logger;
    

};
