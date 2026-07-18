cmake_minimum_required(VERSION 3.8)
project(system_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/live_visualization.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME live_visualization
)

ament_package()
