# Perception Handoff

我负责的 **Perception 部分** 基本完成，主要包括三块：TF / 坐标系、Occupancy Map、Traffic Light Detection。

## 1. TF / 坐标系

- `simulation.launch.py` 里已经建立了传感器静态 TF：

```text
OurCar/INS
-> OurCar/Center
-> OurCar/Sensors/SensorBase
-> OurCar/Sensors/DepthCamera
-> OurCar/Sensors/RGBCameraLeft
-> OurCar/Sensors/RGBCameraRight
-> OurCar/Sensors/SemanticCamera
```

- `true_state_parser` 会发布动态 `world -> OurCar/INS`。
- 所以 depth camera、RGB camera、semantic camera 到 `world` 的 TF 链是完整的。

## 2. Occupancy Map

- `perception/occupancy_map_node` 会订阅 depth image 和 camera info。
- 它会把 depth image 转成点云，再通过 TF 转到 `world` 坐标系。
- 输出包括：

```text
/perception/pcl/points
/perception/pcl/points_preprocessed
/perception/depth/projected_map
/perception/extended_map
```

- `perception.launch.py` 里也启动了 `octomap_server`，输出：

```text
/perception/octomap/projected_map
/perception/octomap/octomap_binary
/perception/octomap/octomap_full
```

## 3. Traffic Light Detection

- `perception/traffic_light_detector_node` 订阅 RGB camera 和 semantic camera。
- 输出红绿灯状态到：

```text
/perception/traffic_light_state
```

- 状态定义：

```text
0 = UNKNOWN
1 = RED
2 = YELLOW
3 = GREEN
```

- Debug 信息发布到：

```text
/perception/traffic_light_debug
```

## 运行方式

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch simulation simulation.launch.py
ros2 launch perception perception.launch.py
```

我这边已经验证过 `simulation` 和 `perception` 能正常 build：

```bash
colcon build --packages-select simulation perception
```

## 后续需要其他模块接上的地方

- Planning / Decision Making 可以使用 `/perception/extended_map` 或 `/perception/octomap/projected_map` 做避障/地图输入。
- Decision / Control 需要订阅 `/perception/traffic_light_state`，红灯时停车，绿灯时继续。
