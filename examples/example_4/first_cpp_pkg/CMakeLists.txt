cmake_minimum_required(VERSION 3.8)
project(first_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(talker_param src/talker_param.cpp)
ament_target_dependencies(talker_param rclcpp std_msgs)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(talker "${cpp_typesupport_target}")
target_link_libraries(talker_param "${cpp_typesupport_target}")
target_link_libraries(listener "${cpp_typesupport_target}")

install(TARGETS
  talker
  talker_param
  listener
  DESTINATION lib/${PROJECT_NAME}
)

# for launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# find param files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
