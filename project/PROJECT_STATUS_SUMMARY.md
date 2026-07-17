# 项目完成情况与已知问题总结

更新日期：2026-07-17

集成分支：`temp_inte`

阶段6测试基线：`bfc5c00`（`Complete Stage 5 integration validation`）

## 1. 当前结论

任务书要求的完整自动驾驶链路已经集成，阶段0至阶段6均已完成。系统可通过一个
launch 文件启动 Unity、仿真桥、感知、路线管理、轨迹生成、决策和控制节点，并从
spawn0 沿规定路线行驶至终点。

阶段6连续进行了三次启用全部生产节点和两个 NPC 事件的完整仿真。三次均经过
waypoint0--59 并进入 `route_complete`，未观察到碰撞、驶离道路或需要人工接管的
情况。当前版本可以作为后续演示、验收和参数微调的集成基线。

## 2. 已完成的系统功能

### 仿真与系统启动

- 使用 `system_bringup/full_system.launch.py` 作为完整系统统一入口。
- 默认 `enable_motion=false`，输入缺失或运动未授权时控制器持续输出安全制动。
- Unity 使用 spawn0；正式完整测试启用 `[true, true]` 两个 NPC 事件。
- 已验证 Unity TCP/UDP 桥、相机、深度、车辆状态、TF 和控制指令方向约定。
- `/car_command` 仅由生产 controller 发布，不能同时运行 dummy controller。

### 路线与轨迹

- 路线与任务书给定道路一致，覆盖全部60个 waypoint。
- route manager 支持 waypoint 到达/越过判断、重复点连续推进、局部路线窗口和
  `route_complete` 终点状态。
- 修正了 waypoint32 后方 T 字路口和 waypoint34--35 一带的转弯几何，使车辆不再
  因过早压路肩或过晚碰撞护栏而停滞。
- trajectory generator 使用平滑且受 waypoint 朝向约束的路径，并对不合理路径进行
  校验；阶段6三轮均未出现路径拒绝或 fallback。

### 感知

- 深度图转换为点云、地面过滤、二维占据栅格、膨胀地图和 OctoMap 已接通。
- 障碍距离按照当前规划路径投影，而不是只使用固定车体矩形，从而避免 waypoint48
  固定护栏被长期误判为前车。
- 交通灯检测结合 RGB 和 semantic 图像，包含时间戳新鲜度保护和目标灯选择。
- waypoint25、32、52、59 四处受控交通灯已完成红灯停车和绿灯释放验证。
- 对确定性的 Unity 光照进行了红/绿宽阈值和窄黄灯阈值调整，解决了绿灯被黄色或
  灯壳反光覆盖后无法起步的问题。

### 决策

- 已实现巡航、红灯减速/停车/等待/释放、合流减速和紧急制动状态优先级。
- 红灯停车点约位于受控 waypoint 前3.69--3.97 m，避免过早停车。
- Event1 能生成和渲染 NPC，右前方距离触发合流减速，NPC 离开后自动恢复。
- Event2 当前实际生成的右侧合流行为也能被检测、避让并恢复，车辆可以继续完成路线。
- 所有超时输入和显式停车请求都会进入安全状态，恢复有效输入后允许自动恢复。

### 控制

- 使用 Pure Pursuit 横向控制和 PID 纵向控制，并按照 Unity 转向符号输出。
- 包含路径、TF、车速、决策速度和停车状态的新鲜度检查。
- 安全停车输出固定为油门0、转向0、制动1。
- 阶段2完成直线、左右弯、初始航向误差、停车和六种故障注入测试。
- 完整仿真中未出现油门与制动同时为正的命令。

## 3. 分阶段完成状态

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 0 | 统一启动入口、安装配置和静态安全检查 | 完成 |
| 1 | Unity 桥、传感器和 TF 契约 | 完成 |
| 2 | 低速控制和故障安全测试 | 完成 |
| 3 | 无事件完整路线和 waypoint52 回归 | 完成 |
| 4 | 地图、障碍物与交通灯感知 | 完成 |
| 5 | 交通规则和单独 NPC 事件 | 完成，Event2 限制见下文 |
| 6 | 三轮完整基准与重复性测试 | 完成，带已记录限制 |

## 4. 阶段6最终证据

三轮正式测试配置均为 spawn0、route index0、两个 NPC 事件开启、全部生产节点开启和
`enable_motion=true`。

| 运行 | 完成时间 | 最大/平均速度 | 跟踪 RMSE / p95 | Event1 最小合流距离 | Event2 最小合流距离 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 648.845 s | 3.155 / 1.697 m/s | 0.7563 / 2.0004 m | 2.437 m | 2.444 m |
| 2 | 651.614 s | 3.162 / 1.752 m/s | 0.7775 / 2.1516 m | 2.377 m | 2.443 m |
| 3 | 651.287 s | 3.153 / 1.625 m/s | 0.7440 / 2.0827 m | 2.438 m | 2.394 m |

共同结果：

- 三次均完成路线，最大 waypoint 为59。
- 未观察到碰撞或驶离道路。
- 运行期间无节点崩溃。
- 路径拒绝/fallback 次数为0。
- 油门与制动同时输出次数为0。
- 红灯停车后均能在稳定绿灯出现时继续行驶。

统计数据和图表位于 `project/stage6_results/`，分析脚本为
`project/tools/analyze_stage6_bags.py`。原始 MCAP 包约119 MiB，保存在本机
`/tmp/team20_stage6_evidence/run{1,2,3}/bag`，未纳入 Git。

## 5. 尚存在的问题和限制

### Event2 未产生任务预期的紧急制动车辆

这是当前最重要的功能覆盖限制。Unity 确实触发并渲染 Event2，但三轮完整测试和此前
局部测试中，NPC 只产生右侧/合流方向的障碍证据，没有进入自车规划路径前方，也没有
执行明显的“前车急刹后加速离开”动作。因此：

- 决策层没有获得有限的 front-obstacle distance；
- `EMERGENCY_BRAKING` 没有在正式三轮测试中触发；
- 只能确认系统无碰撞地处理了 Unity 实际生成的 Event2，不能声称任务预期的 NPC
  紧急制动场景已经通过。

如验收必须演示该动作，需要修改/补充 Unity 场景事件，或增加可控 ROS NPC 场景生成器，
然后重新验证触发距离、完全停车、最小净距和障碍离开后的恢复。

### 固定道路结构仍可能造成短暂合流减速

路径相对障碍走廊已经解决 waypoint48 的永久误刹，但部分路肩或护栏仍会短暂进入
右侧合流区域。阶段6每轮出现15--20次 `SLOWING_FOR_MERGE` 状态进入，其中包含真实
NPC 和静态结构重复进入；这些状态都能自动释放，没有阻止路线完成。

若需要进一步提高乘坐体验，可在不削弱前方紧急制动的前提下增加对象分类、时序运动
判断，或对合流行为使用更精确的道路区域门控。

### Unity 串行传感器偶发较长状态间隔

三轮 rosbag 中 twist 最大消息间隔为12.11--14.20 s，主要发生于启动或 Unity 传感器
流暂停。controller 会按设计进入满制动并在数据恢复后自动继续，因此未造成碰撞或永久
卡死，但会导致不够平顺的停顿。Run2 还出现约17 s 的启动 TF 延迟并自行恢复。

不建议简单取消超时保护。后续应优先检查 Unity 串行传输调度、降低非必要相机负载，
或让高优先级车辆状态使用独立传输通道。

### 完整栈退出超过 launch 的五秒宽限期

四个 Python 节点的重复 shutdown 异常已修复，冷启动和退出检查中均显示
`process has finished cleanly`。但是完整系统收到 SIGINT 后，Unity、TF、C++ 和 Python
节点整体仍可能超过 launch 的5秒宽限期，随后由 SIGTERM 清理。这不影响驾驶功能，
但属于待优化的生命周期问题。

### 代码风格检查仍有历史失败

全新工作区功能测试通过，包括72项 Python 功能测试和10项 controller-core GTest。
route manager 的既有 flake8 测试仍报告260项格式、引号和 import 顺序问题。此次没有为
通过样式检查而批量改写已验证代码。发布前如要求 CI 全绿，应单独建立纯格式化提交并
重新执行完整回归。

### 本机 rosbag2 依赖未系统安装

阶段6使用从官方 Jazzy deb 解包到 `/tmp/rosbag_overlay` 的临时 overlay 录包和分析。
其他机器复现前应安装 `ros-jazzy-rosbag2`。`/tmp` 中的原始证据不是长期存储，若需要
归档应复制到课程存储或发布附件。

## 6. 启动与后续使用

正常构建：

```bash
cd <ros-workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  simulation perception route_manager trajectory_generator \
  decision_making controller system_bringup
source install/setup.bash
```

安全启动并检查 topic、TF 和唯一控制发布者：

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=false
```

确认 readiness checklist 后启动完整驾驶：

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=true
```

具体 topic 契约、参数、各阶段过程和测试证据见 `project/INTEGRATION_README.md`；控制器
接口及参数见 `project/src/controller/README.md`；阶段6图表解释见
`project/stage6_results/README.md`。
