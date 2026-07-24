cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(traffic_light_detector_node src/traffic_light_detector_node.cpp)
target_include_directories(traffic_light_detector_node PRIVATE include)
ament_target_dependencies(
  traffic_light_detector_node
  rclcpp
  sensor_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_traffic_light_stability_filter
    test/test_traffic_light_stability_filter.cpp
  )
  target_include_directories(test_traffic_light_stability_filter PRIVATE include)
endif()

add_executable(occupancy_map_node src/occupancy_map_node.cpp)
ament_target_dependencies(
  occupancy_map_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS
  occupancy_map_node
  traffic_light_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
