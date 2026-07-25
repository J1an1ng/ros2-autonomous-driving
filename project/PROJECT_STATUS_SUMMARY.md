# 项目完成情况与已知问题总结

更新日期：2026-07-25

版本目标：`full-system-v1.6.0`

## 1. 当前结论

完整自动驾驶链路已经集成并完成最新整路线回归。系统可通过一个 launch 文件启动
Unity、仿真桥、感知、路线管理、轨迹生成、决策和控制节点，从 spawn 0 沿规定路线
经过 waypoint 0--63 并在终点停车。

当前 Unity 生产配置为：

```text
spawnIndex: 0
eventEnable: [true, true]
route_initial_waypoint_index: 0
```

2026-07-25 在 v1.5.0 上完成了两轮端到端验证：

- 无事件基线从 waypoint 0 行驶到 63，未进入 `FOLLOWING_VEHICLE`、
  `EMERGENCY_BRAKING` 或非预期的 `SLOWING_FOR_MERGE`。
- 两个事件同时开启的完整路线也到达 waypoint 63。横穿/汇入车辆在 waypoint 43--45
  被平缓处理；前车急刹在 waypoint 47--50 触发动态跟车制动，最小融合前车距离约
  9.92 m。前车重新运动后自车没有强制刹停到 0，而是在约 1.37 m/s 后继续跟车，
  随后正常恢复巡航。

两轮均未观察到碰撞、驶离道路、车辆行为残留制动或需要人工接管的情况。当前版本可
作为演示、验收和后续场景扩展的生产基线。

v1.6.0 安全修改及独立 merge 新鲜度修复合入后完成了双事件 waypoint 0--63
回归。最新路线调整只保留 waypoint 33--35 的安全转弯几何，waypoint 39--42
保持 v1.5.0 的几何和速度配置。调整后的 waypoint 39 到 43 用时 6.805 秒，低速
方案为 9.505 秒；实测平均速度从 1.867 m/s 提升到 2.859 m/s。最新整路线
rosbag 记录用时 370.258 秒；四个受控交通灯均停车并释放，事件区正常进入动态
跟车，且没有紧急制动、油门/刹车同时输出、运行期进程死亡或路线完成前的 path
fallback。

## 2. 已完成的系统功能

### 仿真与统一启动

- `system_bringup/full_system.launch.py` 是完整系统统一入口。
- 默认 `enable_motion=false`；输入缺失、数据过期或运动未授权时控制器输出安全制动。
- `/car_command` 仅由生产 controller 发布，不能同时运行 dummy controller。
- Unity TCP/UDP 桥、RGB、semantic、depth、车辆状态和 TF 均已接通。
- 当前 source 和 install 中的 Unity 配置均保持 spawn 0、两个事件开启。

### 路线、轨迹与控制

- 路线覆盖 waypoint 0--63，并支持终点停车。
- waypoint 33--35 保留闭环验证过的安全转弯几何和低速限制；waypoint 39--42
  保持 v1.5.0 的几何和更快速度配置。
- route manager 支持 waypoint 到达/越过判断、重复点连续推进、局部路线窗口和
  `route_complete`。
- trajectory generator 生成平滑、受 waypoint 朝向约束的局部路径，并拒绝不合理
  路径。
- controller 使用 Pure Pursuit 横向控制和 PID 纵向控制，包含路径、TF、车速、决策
  速度和停车请求的新鲜度检查。
- 安全停车命令固定为油门 0、转向 0、制动 1；正常控制不会同时输出油门和制动。

### 交通灯感知与决策

- 交通灯检测融合 RGB 灯色与 semantic 交通灯区域，包含组件选择、去抖、时间戳
  新鲜度和相邻信号排除。
- waypoint 25、32、52、59 四处受控交通灯均已验证红灯停车和稳定绿灯释放。
- 强绿灯连续两帧即可确认，混合红绿证据仍需四帧；红、黄或 UNKNOWN 会立即撤销
  绿灯许可。
- 决策保留红灯停车锁存，但感知层已确认的绿灯可立即解除该锁存；UNKNOWN 或过期
  灯色不能错误释放已经确认的停车。
- 路口接近速度和硬停车边界分离，避免远距离过早停车。

### 车辆语义检测

- semantic 图像中的红色组件用于 NPC 车辆检测；黄色交通灯类别不会被当作车辆。
- 红色连通组件需要满足面积、ROI、深度有效像素和横向走廊条件。
- 车辆距离采用组件有效深度的近表面稳健分位数，而不是单个最小噪声像素。
- 语义距离包含时间戳超时和物理可行的距离跳变过滤。

### 语义与点云融合

- 全路线车辆控制默认要求“新鲜语义车辆 + 规划路径点云距离”一致。
- 匹配容差为 2.5 m 与语义距离 20% 两者中的较大值。
- 点云匹配证据超过 0.75 s 后失效，旧点云不能确认新车辆。
- waypoint 43--48 是经实车仿真验证的直接语义窗口，因为该处点云走廊会测到与 NPC
  无关的路面和道路结构。
- 原始点云的短距离安全走廊只作为 0.8 m 内的最后碰撞保护，不参与远距离语义匹配。
- waypoint 48 的固定道路结构签名会被抑制，避免永久误刹；真实车辆语义仍可控制。
- 最终只向决策层发布融合后的前车距离和右前方汇入距离。

### 动态跟车和制动

- `FOLLOWING_VEHICLE` 使用当前距离、期望安全间距和滤波后的距离变化率动态调整目标
  速度。
- 当前期望间距为 6.0 m 静止间距加 2.2 s 车头时距。
- 距离持续缩小时逐渐增强制动；距离稳定时保持速度；距离增加时平滑恢复。
- 只有近场风险进入紧急制动，避免远处横穿车辆因径向接近速度较大而过早满刹。
- 前车重新运动并持续满足释放条件时，可从强制制动交回动态跟车，无需自车先完全
  停止。
- 右前方汇入行为仅允许在 waypoint 43--48 触发，其他路段的横向车辆不会触发该
  专用状态。
- 汇入障碍锁存在普通点云缺帧期间保持；观测超过 3 秒后进入
  `merge_obstacle_stale_stop`，并且只能由新鲜安全观测解除。
- 前车和汇入障碍分别维护新鲜度；只有真实汇入点云样本会刷新汇入时间戳，持续的
  semantic 或 raw-front 更新不能伪造安全观测并解除锁存。

## 3. 最新验证证据

### 自动化测试

- 8 个 ROS 包完整构建成功。
- decision making 的融合、路径走廊、来源新鲜度和状态机测试：70/70 通过。
- traffic-light stability filter GTest：4/4 通过。
- controller core GTest：10/10 通过。
- trajectory generator：34/34 通过。
- route manager 功能测试：9/9 通过。
- 完整 `colcon test` 的功能测试均通过；唯一失败仍是 route manager 既有 flake8
  检查，报告 260 项格式、引号和 import 顺序问题，另有 1 项跳过。

### 无事件完整路线

| 项目 | 结果 |
| --- | --- |
| 路线范围 | waypoint 0--63 |
| 车辆跟随误触发 | 无 |
| 紧急制动误触发 | 无 |
| 非事件路段汇入减速 | 无 |
| 终点停车 | 正常 |

测试中多次出现红色路侧或横向车辆语义候选，最近语义距离曾低于 2 m，但由于没有与
新鲜规划路径点云匹配，均未影响纵向控制。

### 两事件完整路线

| 场景 | waypoint | 结果 |
| --- | --- | --- |
| 横穿/汇入车辆 | 43--45 | 约 19.7 m 开始平缓跟随，目标离开后恢复 |
| 前车急刹 | 47--50 | 动态增强制动，最小融合前车距离约 9.92 m |
| 前车重新运动 | 47--48 | 自车最低约 1.37 m/s 后继续跟车，没有不必要地刹停 |
| 事件释放 | 50 | 正常返回 `CRUISE`，无残留制动 |
| 路线终点 | 63 | 正常停车 |

完整路线中自适应跟车出现于 waypoint 43、44、45、47、48、49、50；没有出现非预期
的汇入减速，也没有进入不必要的 `EMERGENCY_BRAKING`。

### 历史 Stage 6 证据

早期三轮正式测试的统计数据和图表仍保存在 `stage6_results/`，分析脚本为
`tools/analyze_stage6_bags.py`。这些结果对应当时 waypoint 0--59 的路线版本，应视为
历史重复性证据；2026-07-25 的 waypoint 0--63 测试是当前行为基线。

## 4. 当前生产接口

核心输入：

```text
/OurCar/INS/pose
/OurCar/INS/twist
/OurCar/Sensors/RGBCameraLeft/image_raw
/OurCar/Sensors/SemanticCamera/image_raw
/OurCar/Sensors/DepthCamera/image_raw
/planning/path
/planning/target_speed
/perception/traffic_light_state
/perception/front_vehicle_detected
/perception/front_vehicle_distance
```

核心决策与控制输出：

```text
/decision/front_obstacle_distance
/decision/merge_obstacle_distance
/decision/target_speed
/decision/stop_required
/decision/state
/decision/debug
/car_command
```

为解决 Event2 问题临时增加的逐级点云距离、融合确认和动态跟车数值 topic 已在验证
完成后删除。生产代码保留一个综合 `/decision/debug`，不再持续发布只供一次性监控
脚本使用的中间消息。

## 5. 已知问题和限制

### Event2 直接语义窗口

waypoint 43--48 的点云会稳定观察到与 NPC 不同距离的道路结构，因此该窗口允许通过
深度验证和横向门控的语义车辆直接控制。该例外只覆盖已验证路段，不应无条件扩展到
全路线。

### 全路线双传感器确认存在短暂释放可能

Event2 窗口之外，语义或规划路径点云任一输入过期都会撤销车辆确认。这可以避免无车
误刹，但在车辆短暂被遮挡或某个传感器掉帧时可能提前释放跟车。当前超时和控制速度
已通过基线测试，后续新增天气、遮挡或不同车辆外观时仍需重新验证。

### Unity 串行传感器可能暂停

历史 rosbag 中 twist 偶发较长消息间隔。controller 会按设计进入安全制动并在数据
恢复后继续，因此不建议取消超时保护。后续应优先优化 Unity 传输调度或分离高优先级
车辆状态通道。

### route manager 风格检查

route manager 的功能测试通过，但既有 flake8 测试仍报告 260 项格式问题。若发布流程
要求 CI 全绿，应单独进行纯格式化提交并重新执行完整动态回归，避免把格式改写与安全
逻辑修改混在一起。

### 原始动态测试数据未长期归档

部分历史 rosbag 位于 `/tmp`，不是长期存储。需要审计或课程归档时，应重新录制最小
topic 集并复制到持久存储。

## 6. 构建、测试和运行

构建：

```bash
cd <project>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行单元测试：

```bash
colcon test --packages-select decision_making controller trajectory_generator
colcon test-result --verbose
```

运行语义车辆独立回归：

```bash
python3 tools/smoke_test_semantic_vehicle_detector.py
```

安全启动：

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=false \
  enable_visualization:=false
```

完整驾驶：

```bash
ros2 launch system_bringup full_system.launch.py \
  route_initial_waypoint_index:=0 \
  enable_motion:=true \
  enable_visualization:=false
```

启动前必须确认 Unity 配置仍为 spawn 0、`eventEnable: [true, true]`，并确认
`/car_command` 只有一个 publisher。

## 7. 清理说明

本次验证结束后已删除：

- Event2 单独启用/禁用配置副本；
- 无事件和全事件重复配置副本；
- 配置切换与自动备份脚本；
- 一次性 Event2/整路线监控脚本；
- 仅供该脚本使用的中间诊断 topic；
- 相应 Python 字节码缓存。

保留的代码均属于生产安全行为、可复用回归测试或历史 Stage 6 证据。
