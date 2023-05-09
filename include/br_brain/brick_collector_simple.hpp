#include "brick_collector_base.hpp"
#include <eigen3/Eigen/Geometry>


class BrickCollectorSimple : public BrickCollectorBase {
    public:
    BrickCollectorSimple(
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr& navigate_client_ptr_,
        rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr& compute_paths_client_ptr_,
        RobotParameters robot_params
    ) : BrickCollectorBase(
            navigate_client_ptr_,
            compute_paths_client_ptr_,
            robot_params
        ){  }
    protected:
    void updateTrajectory(const std::shared_ptr<grid_map::GridMap> obstacle_map, geometry_msgs::msg::PoseStamped brickPose) {

        trajectory.clear();

        trajectory.push_back(brickPose);
        // trajectory.end()->header.frame_id = "arena";
        trajectory.push_back(offload_pose);

        double offload_angle = 2.0 * std::atan2(offload_pose.pose.orientation.z, offload_pose.pose.orientation.w);
        Eigen::Vector2d offload_direction(std::cos(offload_angle), std::sin(offload_angle));
        trajectory.push_back(offload_pose);
        // trajectory.end()->header.frame_id = "arena";

        trajectory.end()->pose.position.x -= (robot_parameters.size_x + (robot_parameters.size_x - robot_parameters.base_link_to_front)) * offload_direction.x();
        trajectory.end()->pose.position.y -= (robot_parameters.size_x + (robot_parameters.size_x - robot_parameters.base_link_to_front)) * offload_direction.y();
        // trajectory.end()->header.frame_id = "arena";


    }
};