ROS 2 Jazzy Lecture Demos 2 and 3
==================================
Target: Ubuntu 24.04 + ROS 2 Jazzy + C++
Package name: mini_robot_demos

This file contains everything needed for:

Demo 2: "Time Travel in ROS 2: Wall Time vs. Simulation Time"
Demo 3: "Find the Bug: Debugging a Broken ROS 2 C++ Node Graph"

-------------------------------------------------------------------------------
DEMO 2: Time Travel in ROS 2: Wall Time vs. Simulation Time
-------------------------------------------------------------------------------

Goal:
  Show that wall/steady time and ROS time are different concepts.
  Then show that ROS time follows /clock when use_sim_time is true.


Terminal 1: normal wall-time behavior

  source /opt/ros/jazzy/setup.bash
  source ~/ros2_jazzy_lecture_ws/install/setup.bash
  ros2 run mini_robot_demos time_observer

Stop with Ctrl+C.

Terminal 1: start the observer in simulation-time mode

  ros2 run mini_robot_demos time_observer --ros-args -p use_sim_time:=true

What to show:
  ROS time will not progress normally until someone publishes /clock.

Terminal 2: publish a simulated clock

  source /opt/ros/jazzy/setup.bash
  source ~/ros2_jazzy_lecture_ws/install/setup.bash
  ros2 run mini_robot_demos clock_publisher

Terminal 3: pause simulated time

  source /opt/ros/jazzy/setup.bash
  source ~/ros2_jazzy_lecture_ws/install/setup.bash
  ros2 param set /lecture_clock step_seconds 0.0

Terminal 3: slow simulated time

  ros2 param set /lecture_clock step_seconds 0.1

Terminal 3: speed up simulated time

  ros2 param set /lecture_clock step_seconds 2.0

Terminal 3: return to real-time-ish behavior

  ros2 param set /lecture_clock step_seconds 0.5

Optional inspection commands:

  ros2 topic echo /clock
  ros2 param get /time_observer use_sim_time
  ros2 param get /lecture_clock step_seconds

Teaching punchline:
  Use ROS time for message stamps, TF, simulation, and bag replay.
  Use steady time for measuring durations, timeouts, and profiling.
  Do not casually mix clocks in C++.

-------------------------------------------------------------------------------
DEMO 3: Find the Bug: Debugging a Broken ROS 2 C++ Node Graph
-------------------------------------------------------------------------------

Goal:
  Show a realistic ROS debugging process.
  The C++ code is okay, but the launch configuration wires the graph incorrectly.

The intended graph is:

  /fake_distance_sensor  -- /mini_robot/distance -->  /decision_node
  /decision_node         -- /mini_robot/cmd_vel  -->  /controller_node

The intentionally broken graph is:

  /fake_distance_sensor  -- /mini_robot/distance -->  nobody
  /decision_node subscribes to /mini_robot/range
  /controller_node waits for /mini_robot/cmd_vel

Suggested live flow:

Terminal 1: launch the broken system

  source /opt/ros/jazzy/setup.bash
  source ~/ros2_jazzy_lecture_ws/install/setup.bash
  ros2 launch mini_robot_demos broken_debug_demo.launch.py

What the students see:
  The sensor prints distances.
  The decision node starts, but never reacts.
  The controller waits for commands.

Terminal 2: inspect the graph before looking at code

  source /opt/ros/jazzy/setup.bash
  source ~/ros2_jazzy_lecture_ws/install/setup.bash

  ros2 node list
  ros2 topic list -t

Inspect the decision node:

  ros2 node info /decision_node

Expected clue:
  /decision_node subscribes to /mini_robot/range.

Inspect the sensor node:

  ros2 node info /fake_distance_sensor

Expected clue:
  /fake_distance_sensor publishes /mini_robot/distance.

Check that data exists on the sensor topic:

  ros2 topic echo /mini_robot/distance

Stop the echo with Ctrl+C after a few messages.

Check the command topic:

  ros2 topic echo /mini_robot/cmd_vel

Expected result:
  No command messages arrive because the decision node never receives distance data.
  Stop with Ctrl+C after a few seconds.

Optional visual inspection:

  rqt_graph

Teaching question:
  Is this a C++ bug or a ROS graph/configuration bug?

Answer:
  It is a launch/configuration bug. The topic names do not match.

Terminal 1: stop the broken launch with Ctrl+C.

Terminal 1: launch the fixed system

  ros2 launch mini_robot_demos fixed_debug_demo.launch.py

Expected result:
  The decision node now receives distance values.
  The controller receives FORWARD and STOP commands.

Terminal 2: inspect the fixed graph

  ros2 node info /decision_node
  ros2 topic echo /mini_robot/cmd_vel

Stop with Ctrl+C after a few messages.

Teaching punchline:
  In ROS 2 debugging, first check the graph:
    nodes, topics, types, publishers, subscribers, parameters, QoS, and launch files.
  Do not start with GDB if the node is simply subscribed to the wrong topic.

-------------------------------------------------------------------------------
Optional extension for Demo 3: make it more interactive in class
-------------------------------------------------------------------------------

Before showing the launch file, ask students:

  1. Which nodes are alive?
  2. Which topics exist?
  3. Who publishes /mini_robot/distance?
  4. Who subscribes to /mini_robot/distance?
  5. Why is /mini_robot/cmd_vel empty?

Then reveal the intentional bug in broken_debug_demo.launch.py:

  'distance_topic': '/mini_robot/range'

and compare it with fixed_debug_demo.launch.py:

  'distance_topic': '/mini_robot/distance'

-------------------------------------------------------------------------------
Cleanup
-------------------------------------------------------------------------------

Remove demo bags/logs if any were created:

  cd ~/ros2_jazzy_lecture_ws
  rm -rf log

Remove the entire demo workspace:

  rm -rf ~/ros2_jazzy_lecture_ws

