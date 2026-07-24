# Simulation Package — ROS2 Jazzy Port

This is a ROS2 Jazzy port of the original ROS1 `simulation` package for the Unity simulator bridge.

## What was ported

- `VehicleControl.msg` ROS interface generation
- Unity TCP sensor receiver
  - True state / pose and twist
  - RGB camera
  - Depth camera
  - IMU
- UDP command transmitter
- JSON config reader
- ROS2 Python launch files
- RViz config converted to RViz2-style plugin names where possible
- Test scripts preserved

The old Catkin/libsocket setup was replaced with `ament_cmake` and standard POSIX sockets, so no external `libsocket` download is required.

## Dependencies

Install ROS2 Jazzy and then:

```bash
sudo apt install \
  ros-jazzy-rclcpp \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-ament-index-cpp \
  ros-jazzy-rviz2
```

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
unzip simulation_ros2_jazzy.zip
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation
source install/setup.bash
```

## Launch

Start the ROS bridge and Unity simulation window:

```bash
ros2 launch simulation simulation.launch.py
```

This also publishes the static vehicle sensor transforms:

```text
OurCar/INS -> OurCar/Center
OurCar/Center -> OurCar/Sensors/SensorBase
OurCar/Sensors/SensorBase -> OurCar/Sensors/DepthCamera
OurCar/Sensors/SensorBase -> OurCar/Sensors/RGBCameraLeft
OurCar/Sensors/SensorBase -> OurCar/Sensors/RGBCameraRight
OurCar/Sensors/SensorBase -> OurCar/Sensors/SemanticCamera
```

Together with the dynamic `world -> OurCar/INS` transform from the true-state stream,
these frames provide the complete TF chain from each sensor to `world`.

Start the bridge, Unity, TF publishers, and RViz:

```bash
ros2 launch simulation simulation_demo.launch.py
```

Run only the bridge when Unity is already running elsewhere:

```bash
ros2 launch simulation simulation_no_Unity.launch.py
```

Use an explicit Unity config file when needed:

```bash
ros2 launch simulation simulation.launch.py \
  config_file:=/path/to/simulation_config.json
```

The bundled Unity player resolves `AD_Sim_Data/StreamingAssets/simulation_config.json`
relative to its executable and cannot consume an arbitrary ROS launch path directly.
The launch file therefore creates a lightweight temporary runtime overlay, copies the
selected config into the expected relative location, and symlinks the large unchanged Unity
assets. The overlay is removed when Unity exits. The installed/source config is never
rewritten, so focused test settings cannot leak into a later normal launch.

The TCP receiver binds to `127.0.0.1` by default and reads the sensor port from
`sensorConfig.port` if a config file exists. This is the safe default for the bundled local
Unity process. When Unity runs on another machine, expose the receiver explicitly:

```bash
ros2 launch simulation simulation.launch.py \
  start_unity:=false \
  sensor_bind_host:=0.0.0.0
```

Unity must then connect to the ROS2 machine's IP address and configured sensor port.

## Command topic

The UDP command transmitter subscribes to:

```text
/car_command
```

Message type:

```text
simulation/msg/VehicleControl
```

ROS2 field names are lowercase:

```text
float32 throttle
float32 steering
float32 brake
float32 reserved
```

Example:

```bash
ros2 topic pub /car_command simulation/msg/VehicleControl \
  "{throttle: 0.2, steering: 0.0, brake: 0.0, reserved: 0.0}"
```

The transmitter repeats the most recently received command at 100 Hz. A one-shot throttle
command therefore remains active after its publisher exits. Test tools must continuously
publish and explicitly send a zero or brake command before stopping.

## Sensor topics

Topics are created dynamically from Unity object names, for example:

```text
/OurCar/Sensors/RGBCameraLeft/image_raw
/OurCar/Sensors/RGBCameraLeft/camera_info
/OurCar/Sensors/DepthCamera/image_raw
/OurCar/Sensors/DepthCamera/camera_info
/OurCar/Sensors/IMU
/OurCar/INS/pose
/OurCar/INS/twist
```

## Notes

- The original ROS1 launch files are kept as `*.ros1` references.
- `simulation.launch.py` and `simulation_demo.launch.py` start the included Ubuntu Unity build automatically.
- The included Unity build requires a graphical Linux session. For WSL or remote machines, run Unity on the host machine and use `simulation_no_Unity.launch.py`.
- The JSON parameter reader is retained as a diagnostic/config reader. In ROS2, parameters are node-local, so the receiver and transmitter also read the config file themselves.
