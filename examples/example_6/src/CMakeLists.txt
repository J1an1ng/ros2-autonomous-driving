cmake_minimum_required(VERSION 3.8)
project(unity_rgb_bridge_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(unity_rgb_bridge_node src/unity_rgb_bridge_node.cpp)
target_compile_features(unity_rgb_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(unity_rgb_bridge_node rclcpp sensor_msgs)

install(TARGETS unity_rgb_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
