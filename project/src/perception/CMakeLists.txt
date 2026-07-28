cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(ONNXRUNTIME_VENDOR_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime"
)
if(NOT EXISTS
  "${ONNXRUNTIME_VENDOR_ROOT}/lib/libonnxruntime.so.1.22.0"
)
  message(FATAL_ERROR
    "Vendored ONNX Runtime 1.22.0 is missing from "
    "${ONNXRUNTIME_VENDOR_ROOT}"
  )
endif()
add_library(onnxruntime_vendor SHARED IMPORTED)
set_target_properties(
  onnxruntime_vendor
  PROPERTIES
  IMPORTED_LOCATION
    "${ONNXRUNTIME_VENDOR_ROOT}/lib/libonnxruntime.so.1.22.0"
  INTERFACE_INCLUDE_DIRECTORIES
    "${ONNXRUNTIME_VENDOR_ROOT}/include"
)

add_executable(yolo_rgbd_detector_node src/yolo_rgbd_detector_node.cpp)
target_include_directories(
  yolo_rgbd_detector_node
  PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(
  yolo_rgbd_detector_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(
  yolo_rgbd_detector_node
  ${OpenCV_LIBS}
  onnxruntime_vendor
)
set_target_properties(
  yolo_rgbd_detector_node
  PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_VENDOR_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN"
)

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
  yolo_rgbd_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    third_party/onnxruntime/lib/libonnxruntime.so
    third_party/onnxruntime/lib/libonnxruntime.so.1
    third_party/onnxruntime/lib/libonnxruntime.so.1.22.0
    third_party/onnxruntime/lib/libonnxruntime_providers_shared.so
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    third_party/onnxruntime/LICENSE
    third_party/onnxruntime/ThirdPartyNotices.txt
  DESTINATION share/${PROJECT_NAME}/licenses/onnxruntime
)

ament_package()
