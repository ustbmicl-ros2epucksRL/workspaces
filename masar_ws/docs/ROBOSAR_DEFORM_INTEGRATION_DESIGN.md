# RoboSAR 与 DEFORM 集成设计说明

本文基于当前可读代码重新梳理 `/root/workspace/masar_ws` 与 `/root/workspace/DEFORM` 的实现边界，并按当前诉求把阶段定义调整为：

- 第 0 阶段：只做 ROS1+Gazebo+RViz 下的 4 台 TurtleBot 应急组可变编队导航闭环。
- 第一阶段：搜索组先发现/发布应急任务点，再由 4 台 TurtleBot 应急组可变编队导航到目标，到位后切任务控制器接口并自转 6 秒，随后继续下一个任务。
- 第二阶段：后续再把第一阶段的 6 秒自转占位控制器替换为 ROS1 实现的任务执行控制器，参考 `/root/workspace/ROS2swarm_w`，但不引入 ROS2/Webots 运行时。

当前结论：目标可以实现，但不能把 DEFORM 整仓直接合并后期待自动闭环。当前 `masar_ws` 已经存在一份 `src/robosar_deform_vendor` 拷贝，后续重点不是“从零复制”，而是补齐缺失的 DEFORM 运行资源，并在 `masar_ws` 内新增应急任务管理、目标派发、到达屏障、控制器切换和安全层。

## 1. 范围结论

本轮范围：

- 只考虑 ROS1 + Gazebo + RViz。
- 不改 `/root/workspace/DEFORM`。
- 不接 ROS2 + Webots。
- 不把响应组注册进 RoboSAR 现有 frontier/coverage 搜索任务分配器。
- 不在第一阶段实现真实 victim/fire/transport 任务策略，只保留任务控制器接口并用 6 秒自转占位验证闭环。

系统目标链路：

```text
搜索组探索 -> 发现应急目标 -> 生成应急任务
-> 应急组接单 -> DEFORM 风格可变编队导航到 staging pose
-> 全员到达屏障 -> 切换任务控制器
-> 第一阶段自转 6 秒占位
-> DONE -> 继续下一个任务
```

## 2. 关键设计修正

对之前设计说明中容易造成实现偏差的部分做如下修正：

- 响应组默认是 4 台 TurtleBot：`tb_0 ... tb_3`，不是 5 台或 12 台。
- `/root/workspace/DEFORM` 原始 demo 默认偏 12 机器人，必须改成 4 机器人配置。
- `src/robosar_deform_vendor` 当前已包含 DEFORM 的主要 ROS1 源码包、消息、服务、仿真 world、TurtleBot/Realsense 描述，以及已从 `/root/workspace/DEFORM` 复制回来的 `perception/drone_detect`。
- 响应组专用 launch 默认使用 `drone_detect` 输出的 `/tb_i/camera/depth/image_filter`，避免 DEFORM 把队友当成动态障碍持续触发压缩/重分配；如需压力测试，可通过 `response_plan_manager_one_run.xml` 的 `depth_topic` 参数切回 raw depth。
- `src/robosar/robosar_emergency_response` 当前只有 checkpoint 资源，没有可运行的响应组源码、launch 和 config。
- 第 0 阶段不是任务管理闭环，而是应急组单独完成“Gazebo 环境 + RViz + 可变编队导航 + 依次目标点”的闭环。
- 第一阶段才接入搜索组发现的任务点，并在到达后执行 6 秒自转占位任务。
- `/sim_map` 是静态地图，不包含箱子、临时障碍、搜索机器人、响应机器人等未知/动态障碍；应急组避障与变形不能只依赖 `/sim_map`。
- DEFORM 的 `task_type` 只适合作为启动时导航/队形收敛 demo 参数；在 RoboSAR 中应把 DEFORM 限定为响应组导航控制器，到位后的任务行为由独立任务控制器接口接管。
- DEFORM 输出不能直接写最终 `/tb_i/cmd_vel`；必须先重映射到 `/tb_i/deform_cmd_vel`，再由 `response_cmd_mux` 统一切换到最终 `/tb_i/cmd_vel`。
- DEFORM 的 `/move_base_simple/goal` 只读取目标 `x/y`，不消费目标 yaw；第一阶段的 `staging_yaw` 只能由到达屏障、后续任务控制器或单独对齐动作处理，不能假设 DEFORM 会完成朝向控制。

## 3. 参考图

第 0 阶段 Gazebo/RViz 场景参考 DEFORM 多障碍编队导航实验：障碍物形成窄通道，应急组在不同通道宽度下压缩、展开或重新分配槽位。红色箭头表示执行/推进方向。

DEFORM 多障碍编队导航方向参考`docs/assets/deform_navigation_direction_reference.png`，用作第 0 阶段 Gazebo/RViz 场景与验收目标参考。

## 4. 当前仓库事实

### 4.1 RoboSAR 已有能力

RoboSAR 已具备搜索组基础闭环：

- Gazebo 仿真入口：`src/robosar/robosar_sim_bringup/launch/robosar_sim_tepper.launch`
- 搜索机器人生成：`src/robosar/robosar_gazebo_sim/launch/spawn_robots.launch`
- 静态地图：`/sim_map`
- 搜索任务分配：`src/robosar/robosar_task_allocator/src/high_ta.py`
- 任务执行入口：`src/robosar/robosar_navigation/include/robosar_navigation/mission_executive.hpp`
- 搜索组多机器人控制：`src/robosar/robosar_controller/src/lazy_traffic_controller.cpp`
- 应急对象仿真和事件：`src/robosar/robosar_sim_bringup/scripts/AprilTagSim.py`

搜索组链路应保持独立：

```text
/frontier_filter/filtered_frontiers
-> high_ta.py
-> task_allocation
-> MissionExecutive
-> MultiAStar
-> robosar_controller/lazy_traffic_controller
-> agent_i cmd_vel
```

### 4.2 已有应急消息和事件流

当前已有消息：

- `robosar_messages/msg/EmergencyObject.msg`
- `robosar_messages/msg/EmergencyObjectArray.msg`
- `robosar_messages/msg/EmergencyTask.msg`
- `robosar_messages/msg/EmergencyTaskArray.msg`
- `robosar_messages/msg/ResponseGroupStatus.msg`

当前已有应急对象 topic：

- `/robosar_sim/emergency_events`
- `/robosar_sim/emergency_objects`
- `/slam_toolbox/victim_markers`
- `/robosar_sim/fire_markers`
- `/robosar_sim/transport_markers`

`EmergencyObject` 已足够承载第一阶段任务输入：对象类型、对象 id、发现状态、发现机器人、精确位姿、box 目标位姿、场景名和检测半径。

### 4.3 当前缺口

需要新增或恢复的能力：

- `robosar_emergency_response` 的源码、launch、config。
- DEFORM vendor 拷贝已补齐 `drone_detect`，响应组专用 `one_run`/launch 需要默认使用 filtered depth。
- 需要提供 4 台 TurtleBot 响应组 Gazebo spawn launch，不能直接使用仍按 12 台机器人启动的 `plan_manager/launch/multi_run.launch`。
- 需要提供 4 机器人参数、响应组 RViz 配置和第 0 阶段目标序列。
- 程序化目标派发，替代 RViz 手工 `/move_base_simple/goal`。
- 到达屏障、cmd_vel mux、安全层、任务完成反馈。
- DEFORM `PlanManager` 当前没有标准“到达/失败”状态 topic，且 `PlanManager::readParameters()` 读取 `plan_manager/obs_clearance`，原始 `one_run.xml` 却设置 `path_optimizer/obs_clearance`；响应组 launch 或代码适配时应显式修正这些运行契约。

## 5. DEFORM 可复用实现

DEFORM 是 ROS1 Noetic + catkin 项目，核心能力在 `/root/workspace/DEFORM/src`。

当前 `masar_ws` 中的 `src/robosar_deform_vendor` 已经拷贝了核心包和仿真资源：`plan_manager`、`traj_opti`、`plan_env2d`、`drone_detect`、`path_opti`、`path_searching2d`、`width_check`、`ot_process`、`plan_rrt`、`to_visualize`、`multi_turtlebot3_simulation`、`turtlebot3_description`、`realsense2_description`、`realsense_gazebo_plugin`。对第 0 阶段来说，这已经足够作为主要基础；真正要补的是响应组包装层、4 机器人配置和控制权隔离。

关键包：

- `plan_manager`
- `control/traj_opti`
- `perception/plan_env2d`
- `perception/drone_detect`
- `planning/path_opti`
- `planning/path_searching2d`
- `planning/width_check`
- `planning/ot_process`
- `planning/plan_rrt`
- `utility/to_visualize`
- `utility/tb_simulation/multi_turtlebot3_simulation`
- `utility/tb_simulation/turtlebot3/turtlebot3_description`
- `utility/tb_simulation/realsense2_description`
- `utility/tb_simulation/realsense_gazebo_plugin`

DEFORM `PlanManager` 的核心状态机：

```text
WAIT_ODOM_OR_TARGET
FRM_INIT_PATH
FRM_KEEP
FRM_DEFORM
AVOID_OBS
APPROACH_GOAL
EMERGENCY_STOP
```

关键机制：

- 每台 TurtleBot 一个 `plan_manager_node`。
- 原始 demo 订阅全局 `/move_base_simple/goal`。
- leader 规划/共享路径点 `/path_point_sharing`。
- leader 共享队形旋转和槽位分配 `/formation_info`。
- `width_check` 基于局部障碍估计通行宽度。
- `ot_process` 用 POT 最优传输做机器人到槽位的重分配。
- `traj_opti` 输出差速控制速度。
- 局部地图由 `plan_env2d/grid_map.cpp` 基于深度图或点云更新 occupancy/ESDF。
- `drone_detect` 在原始 demo 中把 `/tb_i/camera/depth/image_raw` 过滤成 `/tb_i/camera/depth/image_filter`，用于擦除其他机器人在深度图里的像素；第 0 阶段默认恢复该链路。

必须改造的点：

- 原始 `multi_run.launch` 仍按 12 台机器人启动，应提供响应组专用 4 台版本。
- `frm_shape.yaml` 中 `vehicle_number` 应改为 4。
- `plan_manager/tb_num`、`controller/tb_num`、`ot_client/tb_num` 均应统一为 4。
- 响应组 launch 应显式设置 `plan_manager/obs_clearance`，不能只设置原始文件里的 `path_optimizer/obs_clearance`。
- `grid_map/frame_id` 默认 `world`，RoboSAR 集成时应统一到 `map`，或发布 `map -> world` identity。
- DEFORM 的 `~cmd_vel` 必须 remap 到 `/tb_i/deform_cmd_vel`。
- `/move_base_simple/goal` 第一阶段可以保留，但应由程序发布，不依赖 RViz 手工点击。

## 6. 双组机器人定义

### 搜索组

- 默认机器人：`agent_0 ... agent_4`
- 继续使用 RoboSAR 已有探索、建图、发现、任务分配和搜索控制链路。
- 只负责探索地图、定位应急目标点。
- 不参与 DEFORM 编队控制。

### 应急组

- 默认机器人：`tb_0 ... tb_3`
- 默认模型：TurtleBot3 burger。
- 默认初始位置按用户截图配置：

```xml
<arg name="spawn_response_group" default="true"/>
<arg name="model" default="burger"/>

<node pkg="tf2_ros" type="static_transform_publisher"
      name="response_map_to_world"
      args="0 0 0 0 0 0 1 map world"/>

tb_0: x=17.00, y=8.70, yaw=0.0
tb_1: x=16.55, y=9.04, yaw=0.0
tb_2: x=16.55, y=8.36, yaw=0.0
tb_3: x=16.10, y=8.70, yaw=0.0
```

这组初始位置是第 0 阶段与第一阶段在 RoboSAR Tepper 坐标系中的默认响应组出生位姿。若第 0 阶段使用纯 DEFORM 风格小场景，可保留同样的相对队形并整体平移到该 world 的起点。

## 7. 第 0 阶段闭环设计

目标：先不接搜索组，只验证 4 台应急组 TurtleBot 能在 Gazebo/RViz 中根据观测到的障碍变换编队并依次到达目标点。

### 7.1 第 0 阶段场景

第 0 阶段默认使用当前项目内的 DEFORM 风格编队展示 world，只启动应急响应组 `tb_0 ... tb_3`，用于验证参考图方向的可变编队导航效果：

```text
src/robosar/robosar_emergency_response/worlds/stage0_deform_navigation_reference.world
```

旧的简化 corridor world 和 vendor 中的 DEFORM 多障碍 world 保留为压力测试或对照场景，可通过 launch 参数切回：

```text
world_name:=$(find robosar_emergency_response)/worlds/stage0_response_corridor.world
world_name:=$(find multi_turtlebot3_simulation)/worlds/test_multi_obs.world
```

场景要求：

- 至少两个不规则或组合障碍区，形成宽通道、窄通道和转弯段。
- 障碍物作为 Gazebo 实体存在，第一阶段不要求写入 `/sim_map`。
- TurtleBot 优先通过 DEFORM 原始链路的 D435 深度图感知障碍；安全兜底可以额外使用 `/tb_i/scan`。
- RViz fixed frame 使用 `map`，必要时发布 `map -> world` identity。

第 0 阶段不要依赖搜索组，也不要依赖 `/robosar_sim/emergency_events`。目标点由目标序列节点直接给出。

### 7.2 第 0 阶段节点

建议新增：

```text
robosar_emergency_response/launch/stage0_response_deform_demo.launch
robosar_emergency_response/launch/response_group_gazebo.launch
robosar_emergency_response/launch/response_group_deform.launch
robosar_emergency_response/launch/response_plan_manager_one_run.xml
robosar_emergency_response/config/deform/frm_shape_response4.yaml
robosar_emergency_response/config/stage0_goal_sequence.yaml
robosar_emergency_response/config/response_group.yaml
robosar_emergency_response/scripts/response_goal_sequence_node.py
robosar_emergency_response/scripts/response_group_coordinator_node.py
robosar_emergency_response/scripts/deform_dispatch_bridge_node.py
robosar_emergency_response/scripts/response_cmd_mux_node.py
```

第 0 阶段数据流：

```text
stage0_goal_sequence.yaml
-> response_goal_sequence_node.py
-> /response_group/active_task              robosar_messages/EmergencyTask，承载稳定 task_id
-> /response_group/formation_goal
-> deform_dispatch_bridge_node.py
-> /move_base_simple/goal
-> tb_i plan_manager_node
-> /tb_i/deform_cmd_vel
-> response_cmd_mux_node.py
-> /tb_i/cmd_vel
```

到达一个目标后，`response_group_coordinator_node.py` 判断 4 台机器人都满足到达条件，再通知 `response_goal_sequence_node.py` 发布下一个目标。第 0 阶段也应保留 mux，即使暂时只有导航模式；这样第一阶段接任务占位控制器时不用重做控制权架构。

### 7.3 第 0 阶段验收

验收标准：

- Gazebo 中只有 4 台 `tb_0 ... tb_3` 应急机器人即可。
- RViz 可看到响应组、目标点、路径/局部图、队形槽位、当前目标序号。
- 机器人经过宽通道时保持相对展开队形。
- 机器人进入窄通道前或窄通道中能够压缩队形或重分配槽位。
- 不出现长时间卡顿、互相顶死或明显撞墙。
- 能依次到达至少 3 个预设目标点。
- 目标点全部完成后响应组停止或回到 home pose。

## 8. 第一阶段闭环设计

目标：搜索组给出应急目标点，应急组 4 台 TurtleBot 用 DEFORM 可变编队导航到目标，到位后切到任务控制器接口，自转 6 秒，再继续下一个任务。

### 8.1 第一阶段数据流

```text
AprilTagSim.py / 搜索组发现逻辑
-> /robosar_sim/emergency_events
-> emergency_task_manager_node.py
-> /emergency/tasks
-> /emergency/active_task
-> response_group_coordinator_node.py
-> /response_group/active_task 或 /response_group/dispatch_task
-> deform_dispatch_bridge_node.py
-> /move_base_simple/goal
-> DEFORM plan_manager_node x4
-> /tb_i/deform_cmd_vel
-> response_cmd_mux_node.py
-> /tb_i/cmd_vel
-> 全员到达屏障
-> /response_group/behavior_mode = TASK_SPIN_6S_PLACEHOLDER
-> spin_task_placeholder_node.py
-> /tb_i/task_cmd_vel
-> response_cmd_mux_node.py
-> /tb_i/cmd_vel
-> /response_group/task_result
-> emergency_task_manager_node.py
-> DONE / 下一个任务
```

### 8.2 应急任务管理器

新增：

```text
robosar_emergency_response/scripts/emergency_task_manager_node.py
```

输入：

```text
/robosar_sim/emergency_events      robosar_messages/EmergencyObject
/robosar_sim/emergency_objects     robosar_messages/EmergencyObjectArray
/response_group/status             robosar_messages/ResponseGroupStatus
/response_group/task_result        robosar_messages/EmergencyTask，status 使用 DONE/FAILED
```

输出：

```text
/emergency/tasks                   robosar_messages/EmergencyTaskArray
/emergency/active_task             robosar_messages/EmergencyTask
/response_group/dispatch_task      robosar_messages/EmergencyTask
/emergency/task_markers            visualization_msgs/MarkerArray
```

去重 key：

```text
(object_type, object_id, scene_name)
```

默认优先级：

```text
victim > fire > box
```

同优先级按发现时间 FIFO。

### 8.3 staging pose

victim/fire：

```text
approach_direction = normalize(target_pose - response_group_center)
staging_pose = target_pose - approach_direction * staging_distance
staging_yaw = yaw(approach_direction)
```

box：

```text
push_direction = normalize(goal_pose - box_pose)
staging_pose = box_pose - push_direction * staging_distance
staging_yaw = yaw(push_direction)
```

推荐默认参数：

```text
staging_distance_victim: 0.9
staging_distance_fire: 1.1
staging_distance_box: 0.8
```

第一阶段不要求到达目标后的任务编队队形；到 staging pose 后只执行 6 秒自转占位。

注意：DEFORM 原始 goal 回调只使用 `PoseStamped.pose.position.x/y`，不使用 yaw。`staging_yaw` 应写入 `EmergencyTask.staging_pose` 供到达屏障、占位任务或后续第二阶段任务控制器使用；不要把 yaw 完成交给 `/move_base_simple/goal`。应用层目标编号也必须走 `EmergencyTask.task_id`，不能塞进 `PoseStamped.header.seq`，因为 ROS 发布时会把 `Header.seq` 当作传输序号维护。

### 8.4 到达屏障

所有响应机器人都满足以下条件后才能切换控制器：

```text
distance(tb_i, assigned_staging_slot_i) < arrival_radius
abs(yaw_error_i) < arrival_yaw_tolerance
condition holds for arrival_hold_seconds
```

推荐默认参数：

```text
arrival_radius: 0.30
arrival_yaw_tolerance: 0.80
arrival_hold_seconds: 1.0
navigation_timeout: 180.0
```

第一阶段可以先以机器人到 `staging_pose + DEFORM slot_i` 的距离作为屏障判断；后续可改为读取 DEFORM 的到达状态 topic。

由于 DEFORM 原始实现只在日志中打印 `tb i reach goal`，第一阶段不能解析日志作为屏障。若后续修改 `PlanManager`，可以增加 `/tb_i/deform_status`，但第 0/第一阶段优先用外部 odom/slot 判断，风险更小。

当前第 0 阶段已在当前 vendor 拷贝的 `plan_manager` 中增加轻量状态出口 `/tb_i/deform_reached`（`std_msgs/Bool`）：新目标到来发布 `False`，DEFORM 内部判定 `tb i reach goal` 时发布 `True`。`response_group_coordinator_node.py` 仍保留外部 odom/slot 屏障，但优先使用全员 `deform_reached=True` 作为目标完成信号，避免解析日志，也避免外部节点反推 DEFORM 动态压缩队形时出现槽位误差。

### 8.5 6 秒自转占位控制器

新增：

```text
robosar_emergency_response/scripts/spin_task_placeholder_node.py
```

输入：

```text
/response_group/behavior_mode
/emergency/active_task
```

输出：

```text
/tb_i/task_cmd_vel
/response_group/task_result
```

行为：

```text
on TASK_SPIN_6S_PLACEHOLDER:
  publish Twist(linear.x=0.0, angular.z=spin_angular_velocity) for tb_0..tb_3
  keep spin_duration seconds
  publish zero Twist for tb_0..tb_3
  report DONE
```

推荐默认参数：

```text
spin_duration: 6.0
spin_angular_velocity: 0.6
task_timeout: 10.0
```

## 9. cmd_vel 控制权

每台应急机器人必须有统一 mux：

```text
/tb_i/deform_cmd_vel
/tb_i/task_cmd_vel
        |
        v
/tb_i/raw_selected_cmd_vel
        |
        v
/tb_i/safety_filtered_cmd_vel
        |
        v
/tb_i/cmd_vel
```

控制规则：

- `NAV_FORMATION`: 只接受 `/tb_i/deform_cmd_vel`。
- `TASK_CONTROL`: 只接受 `/tb_i/task_cmd_vel`。
- `FAILED` / `STOP`: 输出零速度。
- 安全层始终最后生效。

这一步是必须项，否则 DEFORM 和任务控制器会同时写 `/tb_i/cmd_vel`，导致控制权冲突。

## 10. 未知/动态障碍处理

当前 `/sim_map` 不包含箱子、临时障碍、搜索机器人、响应机器人等未知/动态对象，因此编队变形必须依赖局部观测。

第 0 阶段和第一阶段至少需要：

- TurtleBot D435 深度图输入，或等效点云输入；`/tb_i/scan` 作为安全层兜底，不建议作为 DEFORM 主地图输入的第一选择。
- `plan_env2d/grid_map.cpp` 更新 occupancy/ESDF。
- `width_check` 从局部地图估计通行宽度。
- `response_cmd_mux_node.py` 读取 `/tb_i/scan`、TF 或 `/gazebo/model_states` 做兜底安全限速/急停。

当前已补齐 `drone_detect`，`plan_manager` 默认使用 `/tb_i/camera/depth/image_filter`，减少其他响应机器人被当成墙体导致队形过度收缩。若切回 raw depth，应在验收中重点观察互相遮挡是否造成卡顿。

如果 Gazebo 传感器不能稳定看到搜索机器人或其他动态对象，应新增：

```text
robosar_emergency_response/scripts/dynamic_obstacle_cloud_node.py
```

输入：

```text
/gazebo/model_states
```

输出：

```text
/response_group/dynamic_obstacle_cloud
```

该节点把 `agent_i`、非自身 `tb_j` 和不可接触的应急对象注入 DEFORM local map。第二阶段 transport 时，当前 `transport_box` 需要白名单，不能被安全层一刀切急停。

推荐安全参数：

```text
robot_robot_slowdown_distance: 0.75
robot_robot_stop_distance: 0.45
wall_slowdown_distance: 0.55
wall_stop_distance: 0.25
clearance_hysteresis: 0.10
```

## 11. 建议文件落点

DEFORM vendor 落点如下。当前 `masar_ws` 已经存在其中大部分目录；后续应只补齐缺失内容或新增响应组专用 launch/config，不要修改 `/root/workspace/DEFORM` 原仓：

```text
src/robosar_deform_vendor/plan_manager
src/robosar_deform_vendor/control/traj_opti
src/robosar_deform_vendor/perception/plan_env2d
src/robosar_deform_vendor/perception/drone_detect
src/robosar_deform_vendor/planning/path_opti
src/robosar_deform_vendor/planning/path_searching2d
src/robosar_deform_vendor/planning/width_check
src/robosar_deform_vendor/planning/ot_process
src/robosar_deform_vendor/planning/plan_rrt
src/robosar_deform_vendor/utility/to_visualize
src/robosar_deform_vendor/utility/tb_simulation/multi_turtlebot3_simulation
src/robosar_deform_vendor/utility/tb_simulation/turtlebot3/turtlebot3_description
src/robosar_deform_vendor/utility/tb_simulation/realsense2_description
src/robosar_deform_vendor/utility/tb_simulation/realsense_gazebo_plugin
```

当前需要特别确认：

- `src/robosar_deform_vendor/perception/drone_detect` 已从 `/root/workspace/DEFORM` 复制源码、launch、config、package.xml。
- `src/robosar_deform_vendor/plan_manager/launch/multi_run.launch` 仍按 12 台机器人启动，不能直接作为响应组启动文件。
- `src/robosar_deform_vendor/plan_manager/config/frm_shape.yaml` 仍是 `vehicle_number: 12`，第 0/第一阶段应使用响应组专用 `frm_shape_response4.yaml`，不要直接改动原始参考配置。
- `src/robosar/robosar_emergency_response` 已建立 ROS package，第 0 阶段源码/launch/config 已落地；第一阶段节点仍按后续计划实现。

应急响应包：

```text
src/robosar/robosar_emergency_response/package.xml
src/robosar/robosar_emergency_response/CMakeLists.txt
src/robosar/robosar_emergency_response/launch/stage0_response_deform_demo.launch
src/robosar/robosar_emergency_response/launch/stage1_emergency_response_demo.launch
src/robosar/robosar_emergency_response/launch/response_group_gazebo.launch
src/robosar/robosar_emergency_response/launch/response_group_deform.launch
src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml
src/robosar/robosar_emergency_response/config/deform/frm_shape_response4.yaml
src/robosar/robosar_emergency_response/config/stage0_goal_sequence.yaml
src/robosar/robosar_emergency_response/config/response_group.yaml
src/robosar/robosar_emergency_response/scripts/response_goal_sequence_node.py
src/robosar/robosar_emergency_response/scripts/emergency_task_manager_node.py
src/robosar/robosar_emergency_response/scripts/response_group_coordinator_node.py
src/robosar/robosar_emergency_response/scripts/deform_dispatch_bridge_node.py
src/robosar/robosar_emergency_response/scripts/response_cmd_mux_node.py
src/robosar/robosar_emergency_response/scripts/spin_task_placeholder_node.py
src/robosar/robosar_emergency_response/scripts/dynamic_obstacle_cloud_node.py
```

## 12. 阶段计划

### 第 0 阶段：应急组独立可变编队闭环

目标：

- 基于已复制的 DEFORM vendor 包启动，使用 `drone_detect` filtered depth 输入。
- 建立 4 机器人 `frm_shape_response4.yaml`。
- 默认使用 `robosar_emergency_response/worlds/stage0_deform_navigation_reference.world` 展示第 0 阶段响应组可变编队；旧 corridor world 和 vendor `test_multi_obs.world` 保留为压力测试。
- 启动 4 台 TurtleBot、DEFORM plan manager、RViz。
- 用预设目标序列驱动响应组依次导航。
- 验证队形压缩、展开、槽位重分配和避障稳定性。

完成标志：

```text
4 台 tb_i 能在第 0 阶段 world 中依次完成多个目标点，过程中能根据局部障碍改变队形且无明显撞墙/卡死。
```

### 第一阶段：搜索组任务点到应急组导航和占位任务闭环

目标：

- 搜索组保持现有探索链路。
- `AprilTagSim.py` 或搜索组发现逻辑发布 `/robosar_sim/emergency_events`。
- `emergency_task_manager_node.py` 生成 `EmergencyTask` 队列。
- 应急组接收 active task，计算 staging pose。
- DEFORM 可变编队导航到 staging pose。
- 到达屏障通过后切到 `TASK_SPIN_6S_PLACEHOLDER`。
- 4 台机器人自转 6 秒，上报 DONE。
- 自动领取下一个任务。

完成标志：

```text
搜索组发现目标 -> 应急组编队到达 -> 自转 6 秒 -> DONE -> 下一个目标
```

### 第二阶段：任务执行控制器替换

目标：

- 实现 ROS1 `response_global_info_node.py`，复刻 ROS2swarm_w 的 `/global_info` JSON。
- victim/fire 接 ROS1 版 `neat_roundflocking_node.py`。
- box/transport 接 ROS1 版 `neat_transport_node.py`。
- 用真实任务控制器替换第一阶段的 6 秒自转占位。
- 调 Gazebo box 质量、摩擦、接触参数和安全白名单。

第二阶段参考但不直接运行 ROS2：

```text
/root/workspace/ROS2swarm_w/src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern.py
/root/workspace/ROS2swarm_w/src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern.py
/root/workspace/ROS2swarm_w/src/launch_webots/launch_webots/global_info_supervisor.py
```

已有 checkpoint 资源：

```text
src/robosar/robosar_emergency_response/assets/checkpoints/roundflocking/flocking_genome_package_gen99.pkl
src/robosar/robosar_emergency_response/assets/checkpoints/transport/transport_genome_package_gen264.pkl
```

## 13. 风险与约束

### 13.1 DEFORM 依赖

DEFORM 需要 CasADi/IPOPT 和 POT：

```text
coinor-libipopt-dev
casadi
python3-ot / POT
```

如果环境缺依赖，`width_check`、`traj_opti` 或 `ot_process` 会编译/运行失败。第 0 阶段应先逐包编译验证。

### 13.2 当前 vendor 完整性

当前 `src/robosar_deform_vendor` 已经不是空目录，但还不能直接视为“已集成完成”：

- `drone_detect` 已补齐；响应组 wrapper launch 默认使用 `/tb_i/camera/depth/image_filter`。
- `realsense_gazebo_plugin` 必须随第 0 阶段依赖一起构建，否则 URDF 中的 D435 Gazebo 插件无法加载，`/tb_i/camera/depth/image_raw` 不会稳定发布，`plan_env2d` 会表现为 `has_depth=0`、ESDF 距离长期无效。
- 原始 `multi_run.launch` 和 `frm_shape.yaml` 仍是 12 机器人导航配置。
- DEFORM demo world 和 RViz 已在 vendor 内，可用于第 0 阶段快速验证，但响应组任务管理和 mux 不在 DEFORM 中。

第 0 阶段实施顺序应先解决这些运行完整性问题，再接 RoboSAR 第一阶段任务闭环。

### 13.3 topic 全局命名

DEFORM 原始实现使用全局 topic：

```text
/move_base_simple/goal
/path_point_sharing
/formation_info
/current_width
```

第 0 阶段可以保留，因为只启动一个响应组。第一阶段也可先保留，但应在文档和 launch 中明确这些 topic 属于唯一响应组。后续多响应组才需要 namespacing。

### 13.4 编队到达判定

DEFORM 原始 `PlanManager` 只打印 `tb i reach goal`，没有标准完成 topic。第一阶段应由 `response_group_coordinator_node.py` 基于 odom/slot 做外部到达屏障，而不是解析日志。

### 13.5 动态障碍可见性

仅靠传感器可能因为 FOV、遮挡、自过滤导致动态机器人没有进入 DEFORM local map。`response_cmd_mux` 安全层是必需兜底；必要时再加 `dynamic_obstacle_cloud_node.py`。

当前 Gazebo 深度链路建议使用 `vglrun -d /dev/dri/card0 roslaunch ...` 启动。无 GPU/vglrun 时，headless 深度图频率可能低于 `grid_map/odom_depth_timeout`，导致 `odom or depth lost` 和 ESDF 无效；使用 vglrun 后，D435 depth/filter topic 可稳定进入 `plan_env2d`，`/tb_i/deform_debug` 中 `has_depth=1`。

### 13.6 DEFORM 参数契约

响应组适配时需要显式校验这些参数和 topic：

- `task_type: 1` 用于编队导航。
- `vehicle_number: 4`、`plan_manager/tb_num: 4`、`controller/tb_num: 4`、`ot_client/tb_num: 4` 必须一致。
- `plan_manager/frm_type` 对 4 机器人不应超过 4。
- 4 机器人避障缩编阈值不能用 `floor(0.2 * fixed_num)` 直接判定，因为结果为 0 会导致 `avoid_sum=0` 也触发持续缩编。当前 vendor 适配已改为至少 1 台机器人进入避障才允许缩编。
- `plan_manager/obs_clearance` 应显式设置，避免原始 launch 只设置 `path_optimizer/obs_clearance` 造成 `PlanManager` 内部读到默认值。
- `~cmd_vel` 必须重映射为 `/tb_i/deform_cmd_vel`，由 mux 输出 `/tb_i/cmd_vel`。
- 当前 vendor 适配额外发布 `/tb_i/deform_debug`，用于排查 `fixed_id/tb_id` 重分配、状态机、目标距离、ESDF、邻车距离和 NMPC 输出速度；正常运行不订阅时开销很低。

### 13.7 box 任务

第一阶段不做真实推箱。第二阶段 transport 控制器接入后，当前 transport box 需要允许接触，但墙体、搜索机器人和其他响应机器人仍应急停或限速。

## 14. MVP 定义

第 0 阶段 MVP：

1. 4 台 `tb_0 ... tb_3` 在 Gazebo 生成。
2. RViz 显示响应组、局部图、目标和队形。
3. 预设目标序列自动发布。
4. 响应组根据局部障碍变形编队并依次到达目标点。

第一阶段 MVP：

1. 搜索组继续探索并发现 victim/fire/box。
2. `EmergencyTaskManager` 生成任务队列。
3. 应急组接收第一个任务并导航到 staging pose。
4. 全员到达后切到任务控制器接口。
5. 4 台 TurtleBot 自转 6 秒。
6. 任务状态变为 DONE，并自动执行下一个任务。

这个 MVP 已经覆盖核心诉求：

```text
探索搜索组定位目标 -> 应急组可变编队导航 -> 全员到达后切控制器执行 -> 往复
```
