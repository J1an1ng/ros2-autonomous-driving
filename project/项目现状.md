# 项目状态：full-system-v1.9.0

更新日期：2026-08-01

> 本文是 full-system-v1.9.0 的书面总结，覆盖全系统架构、感知模型、路线与轨迹、
> 决策与控制设计、成员分工、Team 20 自研/实质修改代码、课程提供代码和第三方复用
> 内容。各包 README 保留更详细的 topic、参数、测试和调优说明。

## 1. 当前结论

项目已使用 `RGB + YOLOv8n + Depth/PointCloud` 替代语义分割相机完成目标检测。
生产配置中的 `segCamera.enabled` 为 `false`；YOLO 节点只读取普通 RGB、深度、
相机内参、TF 和少量路线信息，不读取语义图像或语义标签。

当前实现能够：

- 从 YAML 加载经过闭环验证的 64 个 waypoint，发布局部路线、路线语义、目标距离和
  速度上限，并在 waypoint 63 发布 `route_complete`；
- 将车辆投影到有序路线，使用 Hermite/Bézier 几何平滑、等弧长重采样、连续性融合
  和质量门禁生成 `/planning/path`；
- 使用 C++ ROS 2 节点和 ONNX Runtime 运行固定尺寸的 YOLOv8n ONNX 模型；
- 在普通 RGB 图像中检测 COCO `car`、`bus`、`truck` 和 `traffic light`；
- 将深度像素通过三维重建、TF 和 RGB 重投影放入 YOLO 车辆框中，估计前车距离；
- 在 waypoint 25、32、52、59 选择控制本车道的交通灯，而不是对向或横向交通灯；
- 从检测框内的 HSV 颜色、灯泡上下布局和多帧结果判断红、黄、绿灯；
- 对 waypoint 25 的“小检测框没有完整框住下方绿灯灯泡”问题进行补充识别，
  同时拒绝交通灯下方的大块绿色路牌；
- 在 Event1 的 waypoint 17--20 窗口内稳定关联 YOLO/RGB-D 前车，允许经过路线
  门控的视觉距离在点云暂时无法匹配时继续参与纵向控制；
- 跨越短时 YOLO 漏检保留前车关联和安全跟车目标，避免目标速度在跟车速度与
  14 m/s 巡航速度之间跳变；
- 取消 waypoint 25、32、52、59 红绿灯前固定 3 m/s 的远场限速，改用统一的
  距离制动包络逐渐减速；
- 分别在 waypoint 25/32/52/59 前约 2.0/4.0/3.4/4.0 m 命令停车，并通过
  waypoint 25 的局部路线到达半径保证 2 m 停车线的距离参考不会提前切换；
- 使用显式状态机按“紧急制动 > 交通灯停车 > stale merge 停车 > 自适应跟车 >
  汇入减速 > 规划速度”的优先级生成最终目标速度；
- 使用 Pure Pursuit 跟踪轨迹，使用 derivative-on-measurement PID 调节速度，并且
  只由真实 controller 发布 `/car_command`；
- 在检测、模型、TF 或深度数据异常时发布保守的安全状态。

2026-08-01 已使用当前生产配置完成 waypoint 0--63 全路线闭环复验并进入
`route_complete`。Event1 自适应跟车、waypoint 47--48 右侧汇入减速以及 waypoint
25、32、52、59 四个受控交通灯的停车/绿灯放行均在同一次运行中触发。日志未发现
碰撞、驶离路线、节点崩溃、`ERROR` 或 `FATAL`；路线完成后 Unity 和全部 ROS 节点
正常退出。

## 2. 总体架构、模型与设计选择

### 2.1 系统分层和权威接口

```text
Unity / simulation bridge
  ├─ RGB + Depth + CameraInfo + TF ──> perception
  │       ├─ YOLO RGB-D vehicle/light observations ──────────────┐
  │       ├─ point cloud ──────────────> obstacle_monitor ────────┤
  │       └─ occupancy map / OctoMap ──> visualization/diagnostics
  ├─ world -> OurCar/INS TF ──> route_manager ──> trajectory_generator
  │                                  │                    │       │
  │                                  └─ route context ────┤       │
  │                                                       └─ path/nominal speed
  └─ vehicle twist/TF ────────────────────────────────────────────┤
                                                                  v
                                                        decision_making
                                                    final speed + stop intent
                                                                  │
                planning path + vehicle twist/TF ─────────────────┤
                                                                  v
                                                              controller
                                                                  │
                                                             /car_command
                                                                  │
                                                                  v
                                                        simulation / Unity
```

包边界是设计的一部分：`perception` 只报告观察结果，`route_manager` 只提供有序路线
上下文，`trajectory_generator` 只生成名义路径/速度，`decision_making` 组合交通规则与
障碍物行为，`controller` 才把最终意图变成油门、转向和制动。`/planning/target_speed`
不是最终控制速度；controller 只接受 `/decision/target_speed`。`/car_command` 必须只有
一个权威 publisher，避免多个控制器相互覆盖。

### 2.2 主要模型和取舍

| 设计位置 | 当前选择 | 选择理由 | 代价/限制 |
| --- | --- | --- | --- |
| 全局路线 | 固定地图上的 64-waypoint YAML 路线 | 课程地图固定，路线可复现、可检查、可逐点配置语义和速度 | 地图或任务改变时不能自动全局重规划 |
| 局部轨迹 | ego 投影 + cubic Hermite / quadratic Bézier + 重采样/质量门禁 | 比直连 waypoint 平滑，同时比 MPC 简单、透明、易测试 | 只是几何平滑，不优化车辆动力学，也不负责避障 |
| 目标检测模型 | 普通 RGB 上的 COCO YOLOv8n ONNX，C++/ONNX Runtime CPU 推理 | 不依赖 Unity semantic camera；轻量模型可随项目直接运行 | 小交通灯与 Unity 域差异需要路线门控、HSV 和时序补偿 |
| 距离与障碍物 | RGB-D 重投影 + 规划路径点云走廊 + 近场安全走廊 | YOLO 提供类别，Depth/PointCloud 提供距离和几何一致性，互相降低误检 | 传感器缺帧时会保守停车；特定事件窗口需要经过验证的直接视觉 fallback |
| 行为决策 | 可解释状态机、锁存和显式优先级 | 红绿灯、跟车、汇入和紧急状态可追踪、可单元测试，安全优先级明确 | 不具备学习策略对未知场景的泛化能力 |
| 横向控制 | Pure Pursuit + 随车速变化的 lookahead | 对固定路线足够稳定，公式简单，便于调参和检查转向方向 | 高动态/极限操纵能力弱于 MPC |
| 纵向控制 | PID，微分项作用于测量速度 | 实现透明；避免目标速度阶跃造成 derivative kick | 依赖 Unity 速度反馈和场景相关调参 |
| 故障处理 | 输入超时、stop latch、有限输出和 safe-stop brake | stale/missing 数据不会继续沿用旧控制命令 | TCP/Depth 短暂抖动可能造成保守的瞬时停车 |

### 2.3 感知与纵向行为细节

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

#### 2.3.1 车辆检测和测距

- YOLO 使用 COCO 类别 ID `2/5/7`，对应 `car/bus/truck`。
- RGB 左相机和 Depth 相机存在约 0.2 m 基线，不能直接使用相同像素坐标取深度。
- 节点将采样的 Depth 像素反投影到三维，经 TF 变换到 RGB 光学坐标系后重新投影。
- 只保留车辆框裁剪区域中的有效点，至少要求 20 个点，并使用 20% 分位距离减少
  背景点和离群点影响。
- 还会检查有效距离和横向走廊，避免将路旁或对向车辆当作本车道前车。

#### 2.3.2 交通灯检测和选择

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

#### 2.3.3 waypoint 25 绿灯漏检与绿色路牌抑制

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

#### 2.3.4 Event1 前车检测与连续跟车

Event1 车辆在 waypoint 17--20 内位于本车道相机走廊，但预处理点云可能暂时无法
提供一致的路径距离。当前障碍物融合和决策逻辑作了以下更新：

- `vision_direct_control_waypoint_ids` 包含 waypoint 17--20；在这个经过验证的
  窗口内，稳定且位于走廊内的 YOLO/RGB-D 车辆距离可以直接参与前车控制；
- `vision_vehicle_detection_hold_time=1.3 s`，短时检测缺帧时保留最后一次可信
  关联，避免回到全速巡航后又重新刹车；
- 跟车在 20 m 内激活，并使用 2.2 s 时距、距离误差和滤波后的距离变化率连续
  计算目标速度；
- Event1 的目标速度下降斜率限制为 `2.0 m/s²`，从当前车速开始平滑减小；
- 20 m 激活边界外保留 2 m 释放滞回，减少边界附近反复进入/退出跟车；
- waypoint 20 后停止 Event1 的视觉直接控制，因为前车开始驶向路边，避免产生
  第二次不必要制动；
- 独立的近场紧急制动仍保留，不会被平滑跟车逻辑削弱。

#### 2.3.5 红绿灯限速与渐进刹车

waypoint 25、32、52、59 的路线远场速度上限均为 `8 m/s`。这消除了切换到
`stop_zone` 时由路线层直接产生的 `14 -> 3 m/s` 目标速度阶跃。红灯、黄灯或
UNKNOWN 状态下，决策层按以下包络限制目标速度：

```text
v_target = min(planning_speed, 8.0, sqrt(2 * 0.7 * remaining_distance))
remaining_distance = waypoint_distance - active_stop_distance
```

因此车辆会随停车余量连续减速，而不是一进入路口 waypoint 就全力制动。当前停车
边界如下：

| 受控 waypoint | 路线远场上限 | 停车边界 | 说明 |
| ---: | ---: | ---: | --- |
| 25 | 8 m/s | 2.0 m | 局部 `reach_distance=1.5 m`，防止全局 3 m 到达半径提前切换到 waypoint 26 |
| 32 | 8 m/s | 4.0 m | waypoint 33--35 的 3 m/s 转弯限速保持不变 |
| 52 | 8 m/s | 3.4 m | 停车线较默认值前移 0.6 m |
| 59 | 8 m/s | 4.0 m | 最初验证距离制动包络的末端红绿灯 |

绿灯确认后解除交通灯制动限制，后续弯道 waypoint 自身的速度限制仍然有效。

### 2.4 v1.9.0 代码质量与文档统一

本版本在不改变 ROS topic、service、message 和 launch 接口的前提下统一了 Team 20
代码风格：

- 修复 `route_manager` 的 flake8 和 PEP 257 问题，并补充异常路线输入测试；
- 使用根目录 `.editorconfig` 和 `project/CODE_STYLE.md` 统一 Python、C++、launch、
  测试和工具代码的可读性规则；
- 新增 `project/tools/check_code_quality.sh`，集中执行 Python flake8/PEP 257 与 C++
  cpplint/uncrustify；
- 将项目自带的 `dummy_controller` 保持为 v1.8.0 基线，不归入 Team 20 风格修改；
- 对 vendored ONNX Runtime、JSON 头文件和其他第三方内容设置编辑器保护，避免保存时
  产生无关格式变化；
- 完善总 README 和 `src/README.md`，记录四位成员分工、项目功能、依赖安装、
  clone/pull、构建、测试、完整运行和故障排查步骤。

## 3. 关键文件

```text
# 全系统主链路
src/route_manager/route_manager/route_manager_node.py
src/route_manager/route_manager/route_map.py
src/route_manager/route_manager/route_progress.py
src/route_manager/config/route_map.yaml
src/trajectory_generator/trajectory_generator/path_processor.py
src/trajectory_generator/trajectory_generator/trajectory_generator_node.py
src/decision_making/decision_making/obstacle_monitor_node.py
src/decision_making/decision_making/traffic_light_state_machine.py
src/controller/src/control_core.cpp
src/controller/src/controller_node.cpp
src/system_bringup/launch/full_system.launch.py

# 感知和模型
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

# v1.9.0 代码质量与项目级文档
../.editorconfig
CODE_STYLE.md
tools/check_code_quality.sh
../README.md
src/README.md

# Event1、路口速度与制动配置/测试
src/decision_making/config/traffic_light_state_machine.yaml
src/decision_making/test/test_traffic_light_state_machine.py
src/route_manager/route_manager/route_progress.py
src/route_manager/config/route_map.yaml
src/route_manager/test/test_route_progress.py
src/route_manager/test/test_route_safety_geometry.py
```

## 4. 成员分工、Team 20 代码与复用内容

“Team 20 代码”表示团队在本项目中编写或实质修改的实现、配置、测试和集成逻辑，
不表示团队发明了所采用的公开算法。课程提供代码和第三方依赖即使因 ROS 2 兼容、
格式或集成而被修改，原始归属也不会转移给 Team 20。

### 4.1 Who did what?

| 成员 | 主要责任 | 可追踪的模块/产物 |
| --- | --- | --- |
| Lei Fang | `route_manager` | `src/route_manager`：路线 YAML、waypoint recorder、路线加载/校验、进度判定、局部路线和 route tests |
| Yida Lyu | `decision`、`controller`、part of integration | `src/decision_making`、`src/controller`：状态机、障碍物行为、最终目标速度、Pure Pursuit、PID、安全停车和 decision-to-command 接口 |
| Tao Zhang | Perception、part of integration | `src/perception`：YOLO RGB-D、交通灯颜色/时序、车辆距离、点云/occupancy、模型配置和 perception-to-decision 接口 |
| Jianing Huang | `Trajectory_Generator`、part of integration | `src/trajectory_generator`：路线投影、平滑/重采样、连续性/质量门禁、名义速度和 route-to-planning 接口 |

Yida Lyu、Tao Zhang、Jianing Huang 的 integration 是共享工作，包括各自模块边界的
topic/config 对齐、`system_bringup` 联调、端到端测试、动态验证和文档。团队没有提供
更细的个人拆分，因此本文不虚构更细粒度的个人归属。

### 4.2 全系统归属表

| 组件 | 归属类别 | Team 20 的工作 / 复用边界 |
| --- | --- | --- |
| `route_manager` | Team-written / substantively developed | 路线模型、记录工具、加载校验、进度与越点判定、局部路线、参数、测试和文档；负责人 Lei Fang |
| `trajectory_generator` | Team-written / substantively developed | 路线投影、Hermite/Bézier 应用、重采样、连续性融合、质量验证、fallback、测试和文档；负责人 Jianing Huang |
| `decision_making` | Team-written / substantively developed | 交通灯状态机、障碍物融合、跟车/汇入/紧急优先级、timeout/latch、测试和文档；负责人 Yida Lyu |
| `controller` | Team-written / substantively developed | Pure Pursuit/PID 的项目实现、输入检查、输出限幅、安全停车、参数、测试和文档；负责人 Yida Lyu |
| `perception` 中 YOLO RGB-D 和项目映射逻辑 | Team-written / substantively developed around reused libraries/model | ROS 节点、RGB-D 重投影、候选选择、HSV/时序规则、occupancy 输出、配置、测试和文档；负责人 Tao Zhang |
| `system_bringup`、项目配置、集成测试/文档 | Team integration | 单入口 launch、模块开关、安全默认值、RViz、接口联调和验证；Yida/Tao/Jianing shared integration |
| `simulation` / Unity bridge and assets | Course-supplied, retained/reused | 提供传感器/车辆状态和命令传输；Team 20 通过 `system_bringup` 和项目配置调用该包，不把 simulator、bridge 或原始 ROS 2 port 声明为自研 |
| `dummy_controller` | Course-supplied, not production | 示例控制器，保持项目基线，不属于 Team 20 生产控制或代码风格评分范围 |
| `.editorconfig`、`CODE_STYLE.md`、质量门禁和项目级 README | Team 20 shared deliverables | 统一风格、质量检查、可追踪性和运行/发布说明；未按个人进一步拆分 |

复用算法和实现边界：

| 方法/依赖 | 复用内容 | Team 20 实现内容 |
| --- | --- | --- |
| Pure Pursuit / PID | 公开控制方法和数学原理 | Yida Lyu 在本项目中的 C++ 实现、ROS 适配、安全检查和参数调优 |
| Hermite / Bézier interpolation | 公开几何曲线方法 | Jianing Huang 的路线裁剪、切线约束、曲线拼接、重采样、质量门禁与连续性逻辑 |
| Finite-state machine pattern | 通用软件设计模式 | Yida Lyu 的交通灯、跟车、汇入、紧急制动状态/优先级/锁存实现 |
| YOLOv8n / COCO weights | Ultralytics 网络、预训练权重和类别知识 | Tao Zhang 的 ROS 2 ONNX 推理集成、RGB-D 测距、路线门控和交通灯后处理 |
| ROS 2 / TF / OpenCV / ONNX Runtime / OctoMap | 第三方框架和库 | Team 20 的节点、topic、参数、变换、配置和系统集成代码 |

### 4.3 Team 20 编写/修改的 YOLO 集成代码

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

### 4.4 复用的开源模型、代码和库

| 第三方内容 | 在本项目中的用途 | 版本/来源证据 | 许可证与归属说明 |
| --- | --- | --- | --- |
| Ultralytics YOLOv8n | 检测网络结构、COCO 预训练权重；离线训练和 ONNX 导出 | `yolov8n.onnx`、`requirements-yolo-training.txt`、三个工具脚本 | 当前仓库文档记录为默认 AGPL-3.0；模型和 Ultralytics 实现不属于 Team 20 |
| COCO 预训练内容 | 当前模型的初始类别和预训练知识 | 配置使用 80 类及 COCO 类别 ID | 数据集及其标注不是 Team 20 创建；报告中应注明模型为 COCO-pretrained，不应把 COCO 数据写成自有数据 |
| Microsoft ONNX Runtime CPU | C++ 运行 ONNX 模型 | 仓库内 vendored `1.22.0` | MIT；保留 `third_party/onnxruntime/LICENSE` 和 `ThirdPartyNotices.txt` |
| OpenCV | letterbox、图像处理、NMS、HSV、调试绘图 | 当前系统版本 `4.6.0` | 系统 Debian 包声明为 Apache-2.0 AND BSD-3-Clause，并包含其他第三方组件声明 |
| ROS 2 Jazzy API | 节点、消息、参数、TF 和 launch 接口 | `rclcpp`、`sensor_msgs`、`tf2_ros` 等依赖 | 外部 ROS 2 包；本项目只调用其 API，遵守各 ROS 包自身许可证 |
| NumPy、PyYAML、rclpy | 采集/训练辅助脚本 | Python imports 和 training requirements | 外部 Python 包；不属于运行时 C++ YOLO 节点 |

### 4.5 模型与许可证记录

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

### 4.6 可直接用于英文报告的表述

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

Event1 跟车和路口制动还使用以下控制链路接口：

```text
/route/speed_limit
/planning/target_speed
/decision/front_obstacle_distance
/decision/merge_obstacle_distance
/decision/target_speed
/decision/state
/decision/stop_required
/decision/debug
```

控制器应使用 `/decision/target_speed`，而不是直接使用
`/planning/target_speed`。这样 Event1 跟车、交通灯距离制动和紧急制动才能按优先级
合成到同一个最终目标速度。

安全降级值为：

```text
front_vehicle_detected = false
front_vehicle_distance = inf
traffic_light_state = UNKNOWN
```

## 6. 验证状态

### 6.1 full-system-v1.9.0 当前验证

2026-08-01 对当前源码执行了完整发布检查：

| 验证项 | 结果 |
| --- | --- |
| 项目代码质量门禁 | 51 个 Python 文件及配置范围内 C++ 全部通过；继承的 simulator bridge 使用文档化豁免 |
| ROS 2 构建 | 8 个 workspace package 全部成功 |
| 完整 `colcon test` | 179 tests，0 errors，0 failures，1 expected skip |
| Unity 闭环路线 | waypoint 0--63 完成，进入 `route_complete` |
| 运行时稳定性 | 无 `ERROR`、`FATAL`、进程异常退出、碰撞或驶离路线 |
| 关闭流程 | Unity 和全部 ROS 节点 clean exit |

完整运行还确认：

- Event1 在 waypoint 17 附近进入 `FOLLOWING_VEHICLE`，约束解除后恢复 `CRUISE`；
- waypoint 47--48 汇入区域进入 `SLOWING_FOR_MERGE` 并安全恢复；
- waypoint 25、32、52、59 的交通灯逻辑均停车/放行，其中 waypoint 32、59 在
  红灯阶段停车等待，确认绿灯后释放；
- `/car_command` 只有生产 `controller_node` 一个 publisher；
- `/perception/traffic_light_state` 只有生产 YOLO RGB-D 节点一个 publisher；
- RGB、Depth、TF、YOLO、占据地图、路线、轨迹、决策与控制链路持续工作，未使用
  semantic camera。

启动初期可能出现缺少 TF/path/velocity 的告警，运行中也可能出现短时 stale-depth
诊断。控制器会在输入不完整时安全停车，并在新数据到达后恢复；本次运行中这些告警
均未导致路线中断。

### 6.2 full-system-v1.8.0 历史静态验证

2026-07-29 对当时源码执行了路线管理与红绿灯/车辆决策的非 linter 自动化测试：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pytest -q -m 'not linter' \
  src/route_manager/test \
  src/decision_making/test/test_traffic_light_state_machine.py
```

结果为 `87 passed, 3 deselected`。相关测试覆盖：

- waypoint 17--20 的 Event1 视觉直接控制窗口；
- Event1 距离边界滞回、短时检测丢失保持和目标速度平滑下降；
- 前车跟车与独立近场紧急制动的优先级；
- waypoint 25、32、52、59 的距离制动包络；
- waypoint 25 的 2.0 m、waypoint 52 的 3.4 m 局部停车边界；
- waypoint 25 的 1.5 m 局部路线到达半径；
- 四个动态红绿灯 waypoint 的 8 m/s 路线远场上限，以及后续弯道限速不变。

上述结果验证了状态转换、速度计算、路线配置和 waypoint 推进逻辑，但不能代替
Unity 中的闭环制动距离测试。

v1.8.0 发布前又通过 `colcon` 重新构建并测试了 5 个受影响的包：

| 当前测试组 | 用例数 | 结果 |
| --- | ---: | --- |
| Perception focused YOLO ROI | 4 | 全部通过 |
| Controller | 11 | 全部通过 |
| Decision making | 98 | 全部通过 |
| Trajectory generator | 37 | 全部通过 |
| Route manager functional | 15 | 全部通过，另有 3 个 linter 用例按命令排除 |
| 合计 | 165 | 全部通过 |

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

本表保留为 2026-07-27 的历史验证记录。当前版本已重新加入一个聚焦的
`test_yolo_postprocess` 目标及 `ament_cmake_gtest` 依赖，其中 4 个用例覆盖
预测 ROI 的紧凑绿色灯泡、红灯拒绝、裁剪边界拒绝和大型绿色路牌拒绝。原先其余
route-gate、稳定器和真实模型运行测试源码仍未恢复，因此当前不能直接重跑表中的
完整 24 项历史测试。

### 6.3 动态验证记录

2026-08-01 当前 v1.9.0 动态结果：

- waypoint 0--63 完整路线通过并到达 `route_complete`；
- Event1 自适应跟车和 waypoint 47--48 汇入减速正常触发并恢复；
- waypoint 25、32、52、59 的交通灯停车/绿灯释放均正常；
- 路线完成后保持安全停车，所有进程正常关闭；
- 日志关键字扫描未发现错误、崩溃、碰撞或进程死亡。

2026-07-27 历史动态结果：

- waypoint 0--63 完整路线通过并到达 `route_complete`；
- waypoint 25 修复后真实绿灯补充分数约为 90--103，严格阈值为 30；
- waypoint 25、52、59 均选择了画面中控制本车道的信号灯并通过；
- 最终测试结束后停止了临时单独启动的 YOLO 节点，没有遗留重复进程。

2026-07-29 后新增的 Event1 连续跟车、四路口平滑减速和停车线配置，已由上述
2026-08-01 v1.9.0 完整闭环路线复验覆盖。

曾发现两个同名 YOLO 节点同时发布灯色，造成 RED/GREEN 结果互相覆盖。运行时必须
保证 `/perception/traffic_light_state` 和其他 perception 输出只有一个 YOLO
publisher。该问题属于启动/进程重复，不是模型分类问题。

## 7. 构建与安全检查

```bash
source /opt/ros/jazzy/setup.bash
bash tools/check_code_quality.sh
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose

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
- v1.9.0 已完成一次含两个事件的 waypoint 0--63 生产闭环运行；单独的 no-event
  对照运行以及持久化 rosbag/config 证据包尚未完成，因此不能表述为“完整 Gate 4
  证据已归档”。正式演示仍应保存完整日志、Event1 跟车曲线、四个路口的目标速度/
  停车距离、debug 截图和模型校验值，以便第三方复现。
- 最终文档应分别写清楚“Team 20 的 YOLO 集成贡献”和“复用的 YOLOv8n/COCO/
  ONNX Runtime/OpenCV/ROS 2”，不要笼统写成“YOLO was developed by us”。
