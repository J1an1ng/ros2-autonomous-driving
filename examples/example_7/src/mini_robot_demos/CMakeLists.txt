cmake_minimum_required(VERSION 3.8)
project(mini_robot_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(time_observer src/time_observer.cpp)
ament_target_dependencies(time_observer rclcpp)

add_executable(clock_publisher src/clock_publisher.cpp)
ament_target_dependencies(clock_publisher rclcpp rosgraph_msgs)

add_executable(fake_distance_sensor src/fake_distance_sensor.cpp)
ament_target_dependencies(fake_distance_sensor rclcpp std_msgs)

add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node rclcpp std_msgs geometry_msgs)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs)

install(TARGETS
  time_observer
  clock_publisher
  fake_distance_sensor
  decision_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
