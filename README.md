# Introduction to ROS — Team 20 Autonomous Driving Project

The graded ROS 2 Jazzy project is under `project/`. Development targets Ubuntu 24.04.

Detailed dependency, build, package, and simulation instructions are in
[`project/src/README.md`](project/src/README.md). The staged integration procedure and its
current progress are maintained in
[`project/INTEGRATION_README.md`](project/INTEGRATION_README.md).

## Quick start

```bash
cd project
bash src/setup_script.sh
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch system_bringup full_system.launch.py
```

The full-system launch starts with `enable_motion=false`. Do not enable motion until the
readiness checklist in the integration plan passes. To enable motion explicitly:

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=true
```

Never run `dummy_controller` and the real `controller` together because `/car_command`
must have exactly one publisher.
