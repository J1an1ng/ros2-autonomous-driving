0.0) ros2 pkg create --build-type ament_cmake --license Apache-2.0 first_cpp_pkg

0.1) copy listener and talker to src
0.2) adapt CMakeLists.txt
0.3) adapt package.xml

NOW: Test colcon build; test code execution

1.0) create folder msg
1.1) create RobotStatus.msg
1.2) #include "example3/msg/robot_status.hpp" in listener.cpp and talker.cpp
1.3) CMakelists.txt updaten
1.5) package.xml updaten
1.6) ros2 interface show first_cpp_pkg/msg/RobotStatus

Jetzt neue message nutzen:
2.0) listener und talker updaten
2.1) colcon build & source install/setup.bash
2.1) ros2 run first_cpp_pkg listener & talker

Launch file
3.0) Launchfile erzeugen
3.1) für CMakeLists.txt:
	install(
	  DIRECTORY launch
	  DESTINATION share/${PROJECT_NAME}
	)
3.2) colcon build --packages-select first_cpp_pkg
	source install/setup.bash
3.3) ros2 launch first_cpp_pkg talker_listener.launch.py


Param file
