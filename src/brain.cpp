#include <rclcpp/rclcpp.hpp>

#include <br_brain/brick_collector_simple.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <br_brick_management/srv/brick_detection.hpp>

#include <std_msgs/msg/string.hpp>



#define ACTION_CONNECT_TIMEOUT_S 10
#define VAL_MAP_CONNECT_TIMEOUT_S 10
#define PATTERN_TIMEOUT_S 10
#define STATE_MACHINE_UPDATE_PERIOD_MS 50
#define DLL_MODE false

#define BRICK_COLLECTOR_ALGO "simple"

enum ZoneValidityLevel{
    ALL_ALLOWED,
    ZONE_1,
    ZONE_1_AND_ZONE2
};

enum BrainState {
    INIT,
    IDLE,
    FINISHED,
    SEARCH_BRICKS_SAFE,
    SEARCH_BRICKS_CARPET,
    COLLECT_BRICK,
    ERROR
};

std::string brainstate2string(BrainState input){
    switch (input)
    {
    case BrainState::INIT:
        return "INIT";
        break;
    case BrainState::IDLE:
        return "IDLE";
        break;
    case BrainState::SEARCH_BRICKS_SAFE:
        return "SEARCH_BRICKS_SAFE";
        break;
    case BrainState::SEARCH_BRICKS_CARPET:
        return "SEARCH_BRICKS_CARPET";
        break;
    case BrainState::COLLECT_BRICK:
        return "COLLECT_BRICK";
        break;
    case BrainState::ERROR:
        return "ERROR";
        break;
    case BrainState::FINISHED:
        return "FINISHED";
        break;
    default:
        throw std::runtime_error("Got unknown enum (cannot be mapped to any string)");
    }
}

class Brain : public rclcpp::Node {
    public:
    Brain() : Node("brain"), state(BrainState::INIT) {

        primary_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        actionlib_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);



        if(!loadParameters()){
            rclcpp::shutdown();
            return;
        }

        if(!setUpBrickCollector()){
            rclcpp::shutdown();
            return;
        } 
        if(!setUpMapValidationSystem()) {
            rclcpp::shutdown();
            return;
        } 
        if(!setUpSearchPatternFollower()) {
            rclcpp::shutdown();
            return;
        } 

        if(!setUpBrickManagementInterface()) {
            rclcpp::shutdown();
            return;
        } 

        setUpMisc();

        setState(BrainState::IDLE);
        initializeStrategy();
        
    }

    private:

    bool loadParameters(){
        RCLCPP_INFO(this->get_logger(), "Loading parameters ...");
        this->declare_parameter("arena_mode", false);
        arena_mode = this->get_parameter("arena_mode").as_bool();

        this->declare_parameter("unload_pose_x_m", 1.0);
        this->declare_parameter("unload_pose_y_m", 1.0);
        this->declare_parameter("unload_pose_yaw_deg", 225.0);

        this->declare_parameter("robot_size_x_m", 1.0);
        this->declare_parameter("robot_size_y_m", 1.0);
        this->declare_parameter("base_link_to_front_m", 1.0);

        unload_pose.header.frame_id = "arena";
        unload_pose.pose.position.x = this->get_parameter("unload_pose_x_m").as_double();
        unload_pose.pose.position.y = this->get_parameter("unload_pose_y_m").as_double();
        unload_pose.pose.orientation.w = std::cos(this->get_parameter("unload_pose_yaw_deg").as_double() / 2.0 * M_PI / 180.);
        unload_pose.pose.orientation.z = std::sin(this->get_parameter("unload_pose_yaw_deg").as_double() / 2.0 * M_PI / 180.);

        RobotParameters robot_params;
        robot_params.base_link_to_front = this->get_parameter("base_link_to_front_m").as_double();
        robot_params.size_x = this->get_parameter("robot_size_x_m").as_double();
        robot_params.size_y = this->get_parameter("robot_size_y_m").as_double();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded !");
        return true;

    }

    bool setUpBrickManagementInterface(){
        RCLCPP_INFO(this->get_logger(), "Setting up interface to brick-management node ...");
        brick_detection_server = this->create_service<br_brick_management::srv::BrickDetection>("/brick_detection", std::bind(&Brain::processBrickDetectionCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, primary_callback_group);
        RCLCPP_INFO(this->get_logger(), "Interface to brick-management node online !");
        return true;
    }

    bool setBrickSearch(const bool target_state){
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = target_state;
        auto result = set_search_pattern_follower_client->async_send_request(request);
       
        set_brick_executor.spin_until_future_complete(result);

        return result.get()->success;        
    }

    

    

    void setState(BrainState new_state){
        state = new_state;
        std_msgs::msg::String state_msg;
        state_msg.data = ::brainstate2string(state);
        this->state_pub->publish(state_msg);
        RCLCPP_DEBUG(this->get_logger(), "State has changed to: %s", ::brainstate2string(state).c_str());
    }

    bool setUpBrickCollector(){
        RCLCPP_INFO(this->get_logger(), "Setting up brick-collection system ...");
        rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr compute_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(this,"/compute_path_through_poses", actionlib_callback_group);
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this,"/navigate_through_poses", actionlib_callback_group);
        if(! (compute_client_ptr_->wait_for_action_server(std::chrono::seconds(ACTION_CONNECT_TIMEOUT_S)) && navigate_client_ptr_->wait_for_action_server(std::chrono::seconds(ACTION_CONNECT_TIMEOUT_S)))){
            RCLCPP_ERROR(this->get_logger(), "Cannot connect to navigation/planning action servers of Nav2 within %u s - Aborting ...", ACTION_CONNECT_TIMEOUT_S);
            return false;
        }
        if(BRICK_COLLECTOR_ALGO == "simple") brick_collector.reset(new BrickCollectorSimple(navigate_client_ptr_, compute_client_ptr_, robot_params));
        // Further algorithms
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown brick collection algorithm '%s' given - Aborting ...", BRICK_COLLECTOR_ALGO);
            return false;
        }

        brick_collector->setOffloadPose(unload_pose);
        
        RCLCPP_INFO(this->get_logger(), "Brick-collection system is online !");
        return true;
    }

    bool setUpMapValidationSystem(){
        RCLCPP_INFO(this->get_logger(), "Setting up valid map system ...");

        map_validation_pub = this->create_publisher<std_msgs::msg::UInt8>("/valid_map/config", 1);
        size_t time = 0;
        while(map_validation_pub->get_subscription_count() == 0 && time < VAL_MAP_CONNECT_TIMEOUT_S * 10){
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            time++;
        }
        if(time == VAL_MAP_CONNECT_TIMEOUT_S * 10 && rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Valid map system not subscribing within %u s - Aborting ...", VAL_MAP_CONNECT_TIMEOUT_S);
            return false;
        }

        
        RCLCPP_INFO(this->get_logger(), "Valid map system is online !");
        return true;
    }

    bool setUpSearchPatternFollower(){
        RCLCPP_INFO(this->get_logger(), "Setting up connection to search pattern follower ...");

        set_search_pattern_follower_client = this->create_client<std_srvs::srv::SetBool>("/activate_search_grid_follower", rmw_qos_profile_services_default, service_callback_group);
        if(! set_search_pattern_follower_client->wait_for_service(std::chrono::seconds(PATTERN_TIMEOUT_S))){
            RCLCPP_ERROR(this->get_logger(), "Cannot connect to search pattern follower within %u s - Aborting ...", PATTERN_TIMEOUT_S);
            return false;
        }
        set_brick_executor.add_callback_group(service_callback_group, this->get_node_base_interface());
        RCLCPP_INFO(this->get_logger(), "Connection to search pattern follower online !");
        return true;
    }

    void setUpMisc(){
        RCLCPP_INFO(this->get_logger(), "Setting up miscellaneous (state-pub) ...");

        state_pub = this->create_publisher<std_msgs::msg::String>("/brain_state", 1);
        RCLCPP_INFO(this->get_logger(), "Set up miscellaneous (state-pub) !");
    }

    void setZoneValidityLevel(ZoneValidityLevel level){
        if(this->map_validation_pub->get_subscription_count() == 0){
            RCLCPP_WARN(this->get_logger(), "Sending valid-map message to dead-end topic (no subscribers)");
        }
        std_msgs::msg::UInt8 valid_config_msg;

        switch(level){
            case ZoneValidityLevel::ALL_ALLOWED:
            valid_config_msg.data = 0b0;
            break;
            case ZoneValidityLevel::ZONE_1:
            valid_config_msg.data = ~0b11;
            break;
            case ZoneValidityLevel::ZONE_1_AND_ZONE2:
            valid_config_msg.data = ~0b111;
            break;
            default:
            throw std::runtime_error("Unknow ZoneValidityLevel given in setZoneValidityLevel");
        }
        this->map_validation_pub->publish(valid_config_msg);

    }

    void initializeStrategy(){
        RCLCPP_INFO(this->get_logger(), "Initializing strategy ...");
        arena_mode ? setZoneValidityLevel(ZoneValidityLevel::ZONE_1) : setZoneValidityLevel(ZoneValidityLevel::ALL_ALLOWED);
        setBrickSearch(true);
        setState(BrainState::SEARCH_BRICKS_SAFE);
        RCLCPP_INFO(this->get_logger(), "Initialized strategy !");
                
    }

    void collectBrick(){
        setBrickSearch(false);
        if(!this->brick_collector->executeQuery()){
            RCLCPP_ERROR(this->get_logger(), "Execution of brick-collection query failed !");            
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Execution of brick-collection query succeeded !");            
        }
        setState(BrainState::SEARCH_BRICKS_SAFE);
        setBrickSearch(true);

    }

    void processBrickDetectionCallback(
        const std::shared_ptr<rmw_request_id_t> request_header, 
        const std::shared_ptr<br_brick_management::srv::BrickDetection::Request> request,
        std::shared_ptr<br_brick_management::srv::BrickDetection::Response> response
        ){
        if(state == BrainState::COLLECT_BRICK){
            RCLCPP_ERROR(this->get_logger(), "Received brick-detection while processing other - THIS SHOULD NEVER HAPPEN !");
            response->success = false;
            response->delete_from_memory = false;

            return;
        }

        RCLCPP_WARN(this->get_logger(), "TODO: Occupancy grid-forwarding to brick collector");

        

        geometry_msgs::msg::PoseStamped query;
        query.pose = request->detection.pose;
        query.header = request->detection.header;
        if(query.header.frame_id.size() == 0u){
            RCLCPP_WARN(this->get_logger(), "Detection has empty frame_id - Using 'arena' instead ...");
            query.header.frame_id = "arena";
        }
        if(!this->brick_collector->processQuery(nullptr, query)){
            RCLCPP_WARN(this->get_logger(), "Rejecting query as no suitable plan for collection and unloading could be found");
            response->success = false;
            response->delete_from_memory = true;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Start to collect given brick");
        setState(BrainState::COLLECT_BRICK);
        trigger_brick_collection_routine = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Brain::collectBrick, this));
        response->success = false;
        response->delete_from_memory = true;

        return;
    }

    std::shared_ptr<BrickCollectorBase> brick_collector;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr map_validation_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_search_pattern_follower_client;
    rclcpp::Service<br_brick_management::srv::BrickDetection>::SharedPtr brick_detection_server;
    BrainState state;

    rclcpp::TimerBase::SharedPtr trigger_brick_collection_routine;
    bool arena_mode;
    geometry_msgs::msg::PoseStamped unload_pose;
    RobotParameters robot_params;

    rclcpp::CallbackGroup::SharedPtr primary_callback_group;
    rclcpp::CallbackGroup::SharedPtr actionlib_callback_group;
    rclcpp::CallbackGroup::SharedPtr service_callback_group;

    std::atomic_bool set_search_pattern_follower_ran_once_flag;
    rclcpp::executors::SingleThreadedExecutor set_brick_executor;


};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    Brain::SharedPtr brain(new Brain());
    rclcpp::executors::MultiThreadedExecutor mte;
    mte.add_node(brain);
    mte.spin();
}