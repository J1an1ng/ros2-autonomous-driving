cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(autonomy_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simulation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(control_core src/control_core.cpp)
target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PRIVATE include)
target_link_libraries(controller_node control_core)
ament_target_dependencies(controller_node
  autonomy_interfaces
  geometry_msgs
  rclcpp
  simulation
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS control_core controller_node
  EXPORT export_control_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_control_core test/test_control_core.cpp)
  target_link_libraries(test_control_core control_core)
  target_include_directories(test_control_core PRIVATE include)
  add_launch_test(
    test/test_controller_node.py
    APPEND_ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs
    TIMEOUT 30
  )
endif()

ament_export_targets(export_control_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
