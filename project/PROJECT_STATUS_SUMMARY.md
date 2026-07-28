# 项目状态：YOLO 非语义相机方案

更新日期：2026-07-27

> 本文只记录本项目的 YOLO 感知更新及其直接依赖，不讨论规划、决策、控制和
> Unity 仿真框架的整体代码归属。

## 1. 当前结论

项目已使用 `RGB + YOLOv8n + Depth/PointCloud` 替代语义分割相机完成目标检测。
生产配置中的 `segCamera.enabled` 为 `false`；YOLO 节点只读取普通 RGB、深度、
相机内参、TF 和少量路线信息，不读取语义图像或语义标签。

当前实现能够：

- 使用 C++ ROS 2 节点和 ONNX Runtime 运行固定尺寸的 YOLOv8n ONNX 模型；
- 在普通 RGB 图像中检测 COCO `car`、`bus`、`truck` 和 `traffic light`；
- 将深度像素通过三维重建、TF 和 RGB 重投影放入 YOLO 车辆框中，估计前车距离；
- 在 waypoint 25、32、52、59 选择控制本车道的交通灯，而不是对向或横向交通灯；
- 从检测框内的 HSV 颜色、灯泡上下布局和多帧结果判断红、黄、绿灯；
- 对 waypoint 25 的“小检测框没有完整框住下方绿灯灯泡”问题进行补充识别，
  同时拒绝交通灯下方的大块绿色路牌；
- 在检测、模型、TF 或深度数据异常时发布保守的安全状态。

本轮动态联调中，车辆完成了 waypoint 0--63 全路线并进入 `route_complete`。
waypoint 25、52、59 的本车道交通灯均在同一次完整路线中通过。waypoint 25
真实绿灯的补充绿色证据约为 90--103，高于严格放行阈值 30。

## 2. YOLO 感知链路

```text
普通左 RGB 图像
  └─ YOLOv8n ONNX
      ├─ vehicle bbox
      │    └─ Depth 像素 -> 3-D -> TF -> RGB 重投影
      │         └─ bbox 内近表面稳健距离 -> 前车检测/距离
      └─ traffic-light bbox
           └─ 路线门控 + 分路口 ROI/目标位置
                └─ HSV + 灯泡布局 + 必要时向下扩展颜色区域
                     └─ 多帧稳定器 -> RED/YELLOW/GREEN/UNKNOWN
```

这里的 PointCloud/Depth 不负责识别“这是什么物体”。YOLO 负责类别和二维框；
Depth/PointCloud 负责距离、三维位置和几何安全校验，因此在加入 YOLO 后仍然有用。

### 2.1 车辆检测和测距

- YOLO 使用 COCO 类别 ID `2/5/7`，对应 `car/bus/truck`。
- RGB 左相机和 Depth 相机存在约 0.2 m 基线，不能直接使用相同像素坐标取深度。
- 节点将采样的 Depth 像素反投影到三维，经 TF 变换到 RGB 光学坐标系后重新投影。
- 只保留车辆框裁剪区域中的有效点，至少要求 20 个点，并使用 20% 分位距离减少
  背景点和离群点影响。
- 还会检查有效距离和横向走廊，避免将路旁或对向车辆当作本车道前车。

### 2.2 交通灯检测和选择

- 交通灯单独使用 `0.01` 的低置信度阈值，因为远处目标可能只有约 `7 x 15 px`；
  车辆检测仍使用 `0.25`，不会因交通灯阈值而放宽。
- 只有 waypoint 25、32、52、59 且距离路口不超过 20 m 时，交通灯结果才能影响
  后续状态。
- waypoint 25、52、59 使用独立的透视 ROI 和期望目标位置，排除画面边缘的
  对向灯、横向灯和无关高置信度灯。
- 路线管理器越过目标 waypoint 后，门控会保持一个 waypoint，避免车辆刚停稳时
  因 waypoint 已更新而不再观察该灯。
- 明确的绿灯可在连续 2 帧后确认；普通/模糊状态需要 4 帧。非绿灯会立即撤销
  之前的通行许可，使系统能快速响应红灯。

### 2.3 waypoint 25 绿灯漏检与绿色路牌抑制

YOLO 的小交通灯框有时只包含红灯和外壳，未完整包含最下方的绿色灯泡。当前修复
不会把扩展后的大区域直接当成普通交通灯框，而是：

1. 保留原 YOLO 框用于常规 HSV 和灯泡布局分类；
2. 仅为补充绿色证据对小框增加少量水平边距并向下扩展；
3. 在扩展区域中只接受高亮、高饱和、尺寸紧凑且位于合理下方位置的绿色连通区域；
4. 要求候选不接触裁剪边界，并限制面积和形状；
5. 依据原检测框面积归一化分数，再交给严格绿灯阈值判断。

因此，小而亮的下方绿灯灯泡能够被补回；占据大面积、形状过宽或触碰裁剪边界的
绿色路牌不会成为放行证据。相应单元测试已经覆盖“绿灯框被裁短”“扩展碰到图像
边界”和“下方大块绿色路牌”三种情况。

## 3. 关键 YOLO 文件

```text
src/perception/src/yolo_rgbd_detector_node.cpp
src/perception/include/perception/yolo_postprocess.hpp
src/perception/include/perception/traffic_light_route_gate.hpp
src/perception/include/perception/traffic_light_stability_filter.hpp
src/perception/config/yolo_rgbd.yaml
src/perception/launch/perception.launch.py
src/perception/models/yolov8n.onnx
src/perception/models/README.md
tools/collect_yolo_frames.py
tools/export_yolo_onnx.py
tools/train_yolo_sim.py
datasets/team20_yolo.yaml
datasets/team20_yolo/README.md
requirements-yolo-training.txt
```

## 4. Which code is our own, and which code is reused?

以下归属只针对 YOLO 更新。“our own”表示 Team 20 在本项目中编写或实质修改的
集成代码，并不表示 YOLO 算法、预训练权重或依赖库由 Team 20 发明。最终报告列出
个人作者时，仍需由团队确认每位成员的实际贡献。

### 4.1 Team 20 编写/修改的 YOLO 集成代码

| 内容 | 文件 | Team 20 的工作 |
| --- | --- | --- |
| ROS 2 YOLO RGB-D 节点 | `src/perception/src/yolo_rgbd_detector_node.cpp` | 图像订阅、ONNX Runtime 调用、结果发布、RGB-D 重投影、车辆距离、交通灯候选选择和调试输出 |
| YOLO 后处理 | `src/perception/include/perception/yolo_postprocess.hpp` | letterbox 坐标还原、兼容多种 YOLO 输出布局、按类别 NMS、HSV/布局灯色分类、waypoint 25 补充绿灯和绿色路牌抑制 |
| 路口与时间逻辑 | `traffic_light_route_gate.hpp`、`traffic_light_stability_filter.hpp` | 四个路口的激活/越点保持、快速撤销绿灯许可、多帧稳定确认 |
| 参数和启动集成 | `yolo_rgbd.yaml`、`perception.launch.py` 中 YOLO 相关部分、`CMakeLists.txt` 中 YOLO/ONNX Runtime 相关部分 | 类别、阈值、ROI、话题、模型路径、节点和动态库安装配置 |
| 数据工具封装 | `collect_yolo_frames.py`、`train_yolo_sim.py`、`export_yolo_onnx.py`、dataset YAML/README | ROS 图像采集以及对 Ultralytics 训练/导出 API 的项目化封装 |

上述代码是围绕第三方模型和库编写的项目逻辑。特别是
`tools/train_yolo_sim.py` 和 `tools/export_yolo_onnx.py` 只是调用 Ultralytics
公开 API 的本地封装；训练器、网络结构、损失函数和导出器本身不是 Team 20 的代码。

### 4.2 复用的开源模型、代码和库

| 第三方内容 | 在本项目中的用途 | 版本/来源证据 | 许可证与归属说明 |
| --- | --- | --- | --- |
| Ultralytics YOLOv8n | 检测网络结构、COCO 预训练权重；离线训练和 ONNX 导出 | `yolov8n.onnx`、`requirements-yolo-training.txt`、三个工具脚本 | 当前仓库文档记录为默认 AGPL-3.0；模型和 Ultralytics 实现不属于 Team 20 |
| COCO 预训练内容 | 当前模型的初始类别和预训练知识 | 配置使用 80 类及 COCO 类别 ID | 数据集及其标注不是 Team 20 创建；报告中应注明模型为 COCO-pretrained，不应把 COCO 数据写成自有数据 |
| Microsoft ONNX Runtime CPU | C++ 运行 ONNX 模型 | 仓库内 vendored `1.22.0` | MIT；保留 `third_party/onnxruntime/LICENSE` 和 `ThirdPartyNotices.txt` |
| OpenCV | letterbox、图像处理、NMS、HSV、调试绘图 | 当前系统版本 `4.6.0` | 系统 Debian 包声明为 Apache-2.0 AND BSD-3-Clause，并包含其他第三方组件声明 |
| ROS 2 Jazzy API | 节点、消息、参数、TF 和 launch 接口 | `rclcpp`、`sensor_msgs`、`tf2_ros` 等依赖 | 外部 ROS 2 包；本项目只调用其 API，遵守各 ROS 包自身许可证 |
| NumPy、PyYAML、rclpy | 采集/训练辅助脚本 | Python imports 和 training requirements | 外部 Python 包；不属于运行时 C++ YOLO 节点 |

### 4.3 模型与许可证记录

当前随项目分发的模型信息：

```text
file:    src/perception/models/yolov8n.onnx
size:    12,824,062 bytes
sha256:  8707bb120b03a6bf8a199e7c35c6d47ad5f83de4fdd233110db270f7d355a3be
input:   fixed 640 x 640
classes: 80 COCO classes
runtime: Microsoft ONNX Runtime 1.22.0 CPU
```

`src/perception/package.xml` 当前将 perception 包声明为 `AGPL-3.0-only`，
与仓库中关于默认 Ultralytics 模型/代码许可证的记录一致。ONNX Runtime 仍保持
独立的 MIT 许可证和 third-party notices，不能用 perception 包许可证替代或删除。

当前 `requirements-yolo-training.txt` 只约束
`ultralytics>=8.3,<9`，没有锁定精确导出版本。仓库也没有保存原始
`yolov8n.pt` 的校验值。因此在最终公开报告或发布版本前，应补记：

- 实际导出时的精确 Ultralytics 版本；
- 原始 `.pt` checkpoint 的下载来源和 SHA-256；
- 导出命令、opset 和日期；
- 若以后使用自采数据微调，图像/标注作者、采集范围、训练/验证划分和数据许可证。

本节依据当前工作区文件和许可证头整理。如果团队成员曾从未保存在仓库中的示例、
论坛或其他项目手工复制代码片段，应由相应作者在最终文档中补充来源。

### 4.4 可直接用于英文报告的表述

> The YOLO integration code was developed by Team 20. It includes the ROS 2
> C++ inference node, RGB-D reprojection and ranging, YOLO output parsing and
> non-maximum suppression, route-aware traffic-light selection, HSV/layout
> colour classification, temporal filtering, configuration, diagnostics, and
> runtime diagnostics. The underlying YOLOv8n architecture, COCO-pretrained
> weights, and Ultralytics training/export implementation are reused
> third-party components and are not our own work. Runtime inference uses the
> reused Microsoft ONNX Runtime 1.22.0 library, while image processing uses
> OpenCV 4.6.0 and the node interface uses ROS 2 Jazzy. Their respective
> copyright notices and licences are retained.

## 5. 接口和运行参数

主要输入：

```text
/OurCar/Sensors/RGBCameraLeft/image_raw
/OurCar/Sensors/RGBCameraLeft/camera_info
/OurCar/Sensors/DepthCamera/image_raw
/OurCar/Sensors/DepthCamera/camera_info
/planning/path
/route/current_waypoint_id
/route/current_waypoint_distance
```

主要输出：

```text
/perception/front_vehicle_detected
/perception/front_vehicle_distance
/perception/front_vehicle_debug
/perception/traffic_light_state
/perception/traffic_light_scores
/perception/traffic_light_debug
/perception/yolo/debug_image
```

安全降级值为：

```text
front_vehicle_detected = false
front_vehicle_distance = inf
traffic_light_state = UNKNOWN
```

## 6. 验证状态

删除测试源码前，2026-07-27 最后一次执行 perception 测试的结果为：

| 测试组 | 用例数 | 结果 |
| --- | ---: | --- |
| TrafficLightStabilityFilter | 4 | 全部通过 |
| TrafficLightRouteGate | 4 | 全部通过 |
| YoloPostprocess | 11 | 全部通过 |
| YoloModelRuntime | 1 | 全部通过 |
| 合计 | 24 | 0 error、0 failure、0 skipped |

当时的测试覆盖：

- letterbox 几何还原；
- YOLO feature-first、end-to-end 和自定义两类输出布局；
- 按类别 NMS；
- 红/黄/绿 HSV 与灯泡位置分类；
- 小框截断绿灯、向下补充绿灯、图像边界裁剪和绿色路牌拒绝；
- 路口激活、越 waypoint 保持和节点重启恢复；
- 明确绿灯快速确认、模糊状态延迟确认、非绿灯立即撤销；
- 通过 vendored ONNX Runtime 对仓库内真实模型执行 forward 并解析输出。

为精简提交代码，以上四组测试源码、CMake 测试目标及
`ament_cmake_gtest` 依赖现已删除。本表仅作为删除前的历史验证记录，当前精简版本
不能再执行这 24 项单元测试。

动态结果：

- waypoint 0--63 完整路线通过并到达 `route_complete`；
- waypoint 25 修复后真实绿灯补充分数约为 90--103，严格阈值为 30；
- waypoint 25、52、59 均选择了画面中控制本车道的信号灯并通过；
- 最终测试结束后停止了临时单独启动的 YOLO 节点，没有遗留重复进程。

曾发现两个同名 YOLO 节点同时发布灯色，造成 RED/GREEN 结果互相覆盖。运行时必须
保证 `/perception/traffic_light_state` 和其他 perception 输出只有一个 YOLO
publisher。该问题属于启动/进程重复，不是模型分类问题。

## 7. 构建与安全检查

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select perception --symlink-install

# 首次启动先禁止运动，检查框、距离和灯色
ros2 launch system_bringup full_system.launch.py \
  enable_motion:=false \
  enable_visualization:=true

# 检查每个输出只能有一个 publisher
ros2 topic info /perception/traffic_light_state --verbose
```

替换兼容 ONNX 模型时：

```bash
ros2 launch system_bringup full_system.launch.py \
  yolo_model:=/absolute/path/to/best.onnx \
  enable_motion:=false
```

## 8. 当前限制与后续文档建议

- 当前随项目提供的是 COCO YOLOv8n 基线，不是已经完成 Unity 数据微调的自定义模型；
  `datasets/team20_yolo` 目前只有目录说明和 YAML，没有可归属为 Team 20 的训练图像
  或标签。
- 交通灯很小，因此依赖低 YOLO 阈值、路线门控和分路口 ROI。全路线已经跑通，
  但换相机参数、分辨率、路线或地图后必须重新标定 ROI。
- 目前的完整路线结果来自现场联调记录。为了让最终报告可复现，建议保留一次最终
  版本 rosbag、完整日志、四个路口的 debug 截图和模型校验值。
- 最终文档应分别写清楚“Team 20 的 YOLO 集成贡献”和“复用的 YOLOv8n/COCO/
  ONNX Runtime/OpenCV/ROS 2”，不要笼统写成“YOLO was developed by us”。
