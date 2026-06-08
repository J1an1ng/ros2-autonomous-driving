# ROS 2 workspace note:
#
# A top-level CMakeLists.txt is not required for ROS 2 workspaces.
# colcon discovers packages under the src/ directory by reading each package's
# package.xml and CMakeLists.txt.
#
# This file is intentionally kept as a harmless placeholder for compatibility
# with repository layouts that previously contained a ROS 1/catkin top-level
# CMakeLists.txt. Do not include ROS 1 catkin paths such as:
# /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
#
# Build this workspace with:
#   source /opt/ros/jazzy/setup.bash
#   colcon build
cmake_minimum_required(VERSION 3.8)
project(ros2_workspace_placeholder NONE)
