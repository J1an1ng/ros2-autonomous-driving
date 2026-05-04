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

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)


target_link_libraries(talker "${cpp_typesupport_target}")
target_link_libraries(listener "${cpp_typesupport_target}")

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
