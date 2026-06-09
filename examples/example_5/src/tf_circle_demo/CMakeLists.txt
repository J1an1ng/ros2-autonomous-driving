cmake_minimum_required(VERSION 3.8)
project(tf_circle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_circle_node src/tf_circle_node.cpp)
ament_target_dependencies(
  tf_circle_node
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS
  tf_circle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()