# 项目集成状态总结

当前集成分支：`release/full-system-v1.4.0`

## 1. 当前结论

任务要求的完整自动驾驶链路已经集成完成。系统可通过
`system_bringup/full_system.launch.py` 一次启动 Unity、仿真桥、TF、感知、
路线管理、轨迹生成、决策、控制、OctoMap 和可视化节点，并从 spawn0 自动完成
waypoint0--63。

当前生产路线包含64个 waypoint。waypoint60--63 是终点延长段，用于通过最后一个
红绿灯并抵达目标点，不能删除。最新全事件闭环运行成功进入 `route_complete`，
没有观察到碰撞、驶离道路、人工接管、运行期节点退出或油门与制动同时输出。

Event1 合流避让、Event2 前车紧急制动、stale-obstacle 安全策略、两处关键弯道以及
四个动态红绿灯均已在完整路线中验证。当前版本可以作为演示和验收基线。

## 2. 当前系统能力

### 仿真与统一启动

- 完整系统统一入口为 `system_bringup/full_system.launch.py`。
- 默认 `enable_motion=false`；运动未授权或关键输入缺失时，控制器持续安全制动。
- 正式运行使用 spawn0、route index0，并启用两个 Unity NPC 事件。
- 启动器会为 Unity 暂存所选仿真配置，确保 ROS bridge 与 Unity 可执行文件使用同一
  spawn/event 设置，退出后自动清理临时运行目录。
- `/car_command` 只能由生产 `controller_node` 发布。

### 路线与轨迹

- 路线覆盖 waypoint0--63，终点发布 `route_complete` 和零目标速度。
- route manager 支持到达/越过判定、重复点连续推进、局部路径窗口和终点保持。
- waypoint33--35 与 waypoint39--42 使用闭环验证过的弯道几何；弯内限速分别为
  3 m/s 和 2 m/s，避免切弯、压路牙和护栏碰撞。
- trajectory generator 使用 waypoint 朝向约束和平滑路径校验。完整任务在
  `route_complete` 前没有出现路径失败。

### 感知与红绿灯

- 深度图、点云、地面过滤、二维占据栅格、膨胀地图和 OctoMap 已接入完整系统。
- 障碍距离相对当前规划路径计算，输出前向距离和右前合流距离。
- 红绿灯检测结合 RGB 与 semantic 图像，并检查两路图像的时间戳新鲜度。
- waypoint25、32、52、59 四处受控信号均完成停车/放行验证。
- 混合红绿证据需要连续四帧确认绿色；红分不高于4且绿分不低于20的明确绿色使用
  连续两帧确认。任何非绿色帧都会重置快速确认序列。
- 决策层收到确认绿色后可以立即取消正在进行的红灯或 UNKNOWN 边界停车，不再要求
  车辆在已经变绿时先完全停住。

### NPC 事件与决策

决策优先级为：

`EMERGENCY_BRAKING > traffic-light stop > merge slowdown > planning speed`

- Event1 的右前 NPC 会触发 `SLOWING_FOR_MERGE`，障碍离开后根据新鲜观测恢复。
- 新鲜合流障碍将速度限制为2.5 m/s。短时 obstacle 数据缺口继续保持该限速。
- 已锁存的合流障碍超过 `merge_stale_timeout` 后进入
  `merge_obstacle_stale_stop`，目标速度为0；只有新鲜的安全观测可以解除锁存。
- Event2 的前向 NPC 能触发速度相关的紧急制动。紧急锁存只有在前向区域获得新鲜
  清空证据且车辆速度低于停车阈值后才释放，避免尚未停稳就重新给油。

### 控制与故障安全

- 横向使用 Pure Pursuit，纵向使用 PID，并按照 Unity 的转向符号约定输出。
- 控制器检查路径、TF、车速、决策速度和停车状态的新鲜度。
- 安全停车输出固定为油门0、转向0、制动1。
- 数据中断、显式停车和 motion disabled 均会进入安全制动；输入恢复后允许自动恢复。
- 完整运行中没有出现油门与制动同时为正的命令。

## 3. 当前验收结果

完整运行配置为 spawn0、route index0、两个 NPC 事件开启、全部生产节点开启以及
`enable_motion=true`。

| 项目 | 结果 |
| --- | ---: |
| 路线 | waypoint0--63，`route_complete` |
| 任务时间 | 487.410 s |
| 最大/平均车速 | 4.403 / 2.290 m/s |
| Event2 紧急制动触发距离 | 3.412 m |
| Event2 最小前向/合流距离 | 2.696 / 2.298 m |
| 紧急制动释放速度 | 0.034 m/s |
| 最大 twist 消息间隔 | 2.857 s |
| 最大决策速度输出间隔 | 0.066 s |
| 油门与制动同时输出 | 0 |
| 运行期节点退出 | 0 |

关键行为：

- Event2 在 waypoint47 遇到2.597 s 的 stale-obstacle 安全停车窗口，期间没有恢复
  巡航。新鲜数据返回后先恢复2.5 m/s 合流限速，随后进入唯一一次
  `EMERGENCY_BRAKING`，约4.60 s 后在接近静止时释放。
- 最后一个信号在 waypoint59 进入短暂停车请求。稳定绿灯发布后8 ms 内解除决策
  锁存，整个窗口最低车速为1.574 m/s，没有再次出现绿灯亮起后完全刹停数秒的现象。
- waypoint33--35 和 Event2 前的 waypoint39--42 弯道均沿验证过的车道中心轨迹通过，
  没有观察到右轮上路牙或碰撞。
- waypoint60--63 全部完成，车辆通过最后信号后抵达终点。

完整 MCAP：

`/tmp/full_route_final_fix_20260724/full_route_final_fix_20260724_0.mcap`

ROS 日志目录：

`~/.ros/log/2026-07-24-22-41-34-139334-localhost-61094/`

## 4. 构建与测试状态

当前生产配置中的 `route_manager`、`trajectory_generator`、`decision_making`、
`perception` 和 `system_bringup` 均构建成功。

当前汇总结果为101项测试、0项 error、1项 failure、1项 skip。所有功能测试、
路线安全几何测试和4项交通灯稳定滤波 GTest 均通过。唯一失败是 route manager
已有的 flake8 检查，共260项引号、import 顺序和格式问题；这属于代码风格债务，不是
运行功能失败。

## 5. 已知限制

### 保守的合流误减速

路边静态结构偶尔会进入右前合流检测区域。最终运行在 waypoint56 出现一次0.248 s
的 `merge_obstacle_stale_stop`，下一条新鲜观测到达后自动恢复。它不影响路线完成，
但若继续优化乘坐平顺性，应增加道路区域门控、对象分类或运动时序判断，不能通过取消
stale 安全停车来规避。

### Unity 传感器间隔

Unity 串行发送多路传感器，完整负载下偶尔会延迟车辆状态。本次最大 twist 间隔为
2.857 s，控制器按设计安全停车并自动恢复。应优先优化 Unity 传输调度，而不是放宽或
删除控制器输入新鲜度检查。

### 终点后路径提示

进入 `route_complete` 后只剩单个终点，trajectory generator 可能报告
“path has fewer than two points”并使用目标点 fallback。此时目标速度已经是0，
车辆保持停车，因此不影响任务。

### 可视化和退出提示

RViz 可能输出 indexed-image shader 警告，OctoMap 启动时可能输出空地图提示。
完整系统 Ctrl-C 后，Unity 有时超过 launch 的5秒 SIGINT 宽限期，随后由 SIGTERM
正常结束；ROS 节点均能干净退出。

### 代码风格

route manager 的260项历史 flake8 问题仍需单独整理。若发布流程要求 CI 全绿，
应建立纯格式化提交并重新运行完整回归，避免把格式化和功能修改混在一起。

## 6. 启动与证据

构建：

```bash
cd <ros-workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  simulation perception route_manager trajectory_generator \
  decision_making controller system_bringup
source install/setup.bash
```

安全启动：

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=false
```

确认唯一控制发布者、TF、规划路径、车速和决策话题正常后启动完整驾驶：

```bash
ros2 launch system_bringup full_system.launch.py enable_motion:=true
```

系统接口、readiness checklist 和详细验证要求见 `INTEGRATION_README.md`；
模块参数见各包内 README。
