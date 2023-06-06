#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <array>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#define MAP_FRAME  "arena"
#define MAX_VAL 255.F

using std::placeholders::_1;


struct ZoneContructor{
    ZoneContructor(bool initialize = true, std::string filename = "/home/svenbecker/Documents/EPFL/Semester_Project/ros2_ws/src/br_brain/config/zones.csv"){
        if(initialize) loadFromConfig(filename);
    }


    void loadFromConfig(const std::string filename){
        zone_maps = std::make_shared<grid_map::GridMap>(grid_map::GridMap());
        zone_maps->setFrameId(MAP_FRAME);
        zone_maps->setGeometry(grid_map::Length(8.0,8.0), 1.0, Eigen::Vector2d(4.0, 4.0));

        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Could not open file: " << filename << std::endl;
            return;
        }

        std::string line;
        std::vector<std::string> identified_zones;
        float x(0.), y(7.5);

        while (getline(file, line)) {
            x = 0.5;
            std::vector<std::string> row;
            std::stringstream ss(line);
            std::string cell;

            while (getline(ss, cell, ',')) {
                std::string zone_name = "zone" + cell;
                if (std::find(identified_zones.begin(), identified_zones.end(), cell) == identified_zones.end()) {
                    zone_maps->add(zone_name, 0);
                    identified_zones.push_back(cell);
                }
                if(!zone_maps->isInside(grid_map::Position(x,y))){
                    std::cerr << "Out of map " << x << " " << y << std::endl;
                }
                
                zone_maps->atPosition(zone_name, grid_map::Position(x,y)) = MAX_VAL;
                x += 1.0;
            }
            y -= 1.0;
        }
        file.close();
    }
    bool getOccupancyGrid(
        const std::vector<size_t>& configuration, 
        std::shared_ptr<nav_msgs::msg::OccupancyGrid>& output_map, 
        const double resolution,
        const float occupied_value = 1.0, 
        const float free_value = 0.0
    ){
        grid_map::GridMap out_map_grid_map;
        out_map_grid_map.setFrameId(MAP_FRAME);
        out_map_grid_map.setGeometry(grid_map::Length(9.0,9.0), resolution, Eigen::Vector2d(4.0, 4.0));
        out_map_grid_map.add("validation", free_value);

        for(size_t i : configuration){
            std::string zone_name = "zone" + std::to_string(i);
            for(grid_map::GridMapIterator it(out_map_grid_map); !it.isPastEnd(); ++it) { 
                grid_map::Position it_position;
                out_map_grid_map.getPosition(*it, it_position);
                if(!zone_maps->isInside(it_position)) continue;
                if(zone_maps->atPosition(zone_name, it_position) != MAX_VAL) continue;
                out_map_grid_map.at("validation", *it) = occupied_value;
            }
        }
        nav_msgs::msg::OccupancyGrid return_grid;
        grid_map::GridMapRosConverter::toOccupancyGrid(out_map_grid_map, "validation", free_value, occupied_value, return_grid );
        output_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(return_grid);

        return true;
    }
    private:
    std::shared_ptr<grid_map::GridMap> zone_maps;
};


class ZoneManager : public rclcpp::Node {
    public:
    ZoneManager() : Node("zone_manager") {
        this->declare_parameter("local_costmap_resolution", 0.02);
        this->declare_parameter("global_costmap_resolution", 0.05);

        local_cm_resolution = this->get_parameter("local_costmap_resolution").as_double();
        global_cm_resolution = this->get_parameter("global_costmap_resolution").as_double();



        zc.reset(new ZoneContructor());
        rclcpp::QoS map_qos(10);  // initialize to default
        if (true) {
            map_qos.transient_local();
            map_qos.reliable();
            map_qos.keep_last(1);
        }
        occ_grid_loc_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/valid_map_local", map_qos);
        occ_grid_glob_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/valid_map_global", map_qos);

        config_sub = this->create_subscription<std_msgs::msg::UInt8>("/valid_map/config", 5, std::bind(&ZoneManager::configCallback, this, _1));

        rclcpp::sleep_for(std::chrono::milliseconds(100));
        this->publishConfig(static_cast<unsigned char>(0));
    }


    private:

    void publishConfig(const unsigned char config){
        std::vector<size_t> extracted_config;
        for(size_t i = 0; i < 5; i++){
            if(config >> i & 0x1) extracted_config.push_back(i);
        }
        zc->getOccupancyGrid(extracted_config, og, global_cm_resolution);
        og->header.frame_id = MAP_FRAME;
        og->header.stamp = this->get_clock()->now();
        occ_grid_glob_pub->publish(*og);

        zc->getOccupancyGrid(extracted_config, og, local_cm_resolution);
        og->header.frame_id = MAP_FRAME;
        og->header.stamp = this->get_clock()->now();
        occ_grid_loc_pub->publish(*og);
    }

    void configCallback(const std::shared_ptr<std_msgs::msg::UInt8> msg){
        publishConfig(msg->data);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_glob_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_loc_pub;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr config_sub;

    std::shared_ptr<ZoneContructor> zc;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> og;
    double local_cm_resolution, global_cm_resolution;

};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    ZoneManager::SharedPtr zm(new ZoneManager());
    rclcpp::spin(zm);
}
