cmake_minimum_required(VERSION 3.8)
project(br_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rclcpp_action REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


find_package(std_msgs REQUIRED)
find_package(br_brick_management REQUIRED)




find_package (Eigen3 3.3 REQUIRED NO_MODULE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

include_directories(include)

add_executable(search_grid_follower src/search_grid_follower.cpp)
target_link_libraries (search_grid_follower Eigen3::Eigen)

add_executable(valid_map_pub src/valid_map_pub.cpp)

add_executable(brain src/brain.cpp)





ament_target_dependencies(
  valid_map_pub
  rclcpp
  grid_map_core
  grid_map_ros
  std_msgs

  nav_msgs
)

ament_target_dependencies(
  search_grid_follower
  rclcpp
  rclcpp_action
  std_srvs
  nav_msgs
  nav2_msgs
  grid_map_ros
  grid_map_core
  std_msgs
)

ament_target_dependencies(
  brain
  rclcpp
  rclcpp_action
  std_srvs
  nav_msgs
  nav2_msgs
  grid_map_ros
  grid_map_core
  std_msgs
  br_brick_management
)

install(TARGETS
  search_grid_follower 
  DESTINATION lib/${PROJECT_NAME}/
)


install(TARGETS
  valid_map_pub 
  
  DESTINATION lib/${PROJECT_NAME}/
)

install(TARGETS
  brain 
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)