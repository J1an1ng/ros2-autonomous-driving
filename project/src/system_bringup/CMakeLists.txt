cmake_minimum_required(VERSION 3.8)
project(system_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/live_visualization.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME live_visualization
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_mission_analyzer
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/test_analyze_mission_run.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../../tools
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_full_mission_runner
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/test_run_full_mission.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../../tools
    TIMEOUT 30
  )
endif()

ament_package()
