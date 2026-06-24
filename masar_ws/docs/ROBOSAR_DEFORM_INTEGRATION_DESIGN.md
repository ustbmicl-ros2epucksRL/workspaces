# RoboSAR 与 DEFORM 集成实现说明

本文按当前 `masar_ws` 中的实际代码重新整理 RoboSAR 与 DEFORM 的集成边界、运行入口和数据流。它不是早期方案草稿，而是当前实现口径说明。

当前实现位于 ROS1 Noetic 工作空间 `masar_ws`，通常在 4a6000 设备的 `masar_ros1_srv-g` 容器中运行。文档中的路径均以 `masar_ws` 为根目录说明。DEFORM 代码使用 `src/robosar_deform_vendor` 中的 vendor 拷贝，顶层独立 `DEFORM` 仓库只作为来源和参考，不作为运行时直接修改对象。

## 1. 当前结论

当前项目已经落地三个层次：

1. 第 0 阶段：4 台 TurtleBot3 burger 响应组在 ROS1 + Gazebo + RViz 中独立运行 DEFORM 可变编队导航。
   - 入口：`./run_response_deform_demo.sh`
   - Launch：`src/robosar/robosar_emergency_response/launch/response_deform_demo.launch`
   - World：`src/robosar/robosar_emergency_response/worlds/deform_navigation_demo.world`
   - 目标序列：`src/robosar/robosar_emergency_response/config/response_goal_sequence.yaml`

2. Tepper 大系统响应闭环：先启动 RoboSAR 搜索/建图/任务基础栈，再叠加 4 台 TurtleBot 响应组。
   - 入口：`./run_tepper_response_demo.sh`
   - 主仿真：`robosar_sim_bringup/launch/robosar_sim_tepper.launch`
   - 响应叠加层：`src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch`
   - 搜索组继续使用 `agent_0 ... agent_4`
   - 响应组使用 `tb_0 ... tb_3`

3. 独立任务控制器验证：不经过 DEFORM 导航，直接验证 roundflocking 和 transport 控制器。
   - 入口：`./run_response_controller_demo.sh`
   - 支持：`roundflocking`、`roundflocking-checkpoint`、`roundflocking-heuristic`、`transport`、`transport-checkpoint`、`transport-heuristic`

当前系统目标链路为：

```text
搜索组探索/发现应急对象
-> emergency_task_manager 生成任务队列
-> 响应组 DEFORM 编队导航到 staging pose
-> response_group_coordinator 判定到达
-> 切换 TASK_CONTROL
-> response_task_controller 执行 roundflocking / transport / spin 占位
-> task_result DONE/FAILED
-> 继续下一个任务或自动返航
```

## 2. 实现边界

当前集成只在 ROS1 + Gazebo + RViz 内完成，不启动 ROS2 或 Webots。`ROS2swarm_w` 中的学习控制器语义通过 ROS1 适配代码复用，主要体现在 `/global_info` JSON、checkpoint 输入输出格式、roundflocking 和 transport 控制器适配上。

明确边界如下：

- `src/robosar_deform_vendor` 是本工作空间内的 DEFORM vendor 拷贝，包含 `plan_manager`、`traj_opti`、`plan_env2d`、`drone_detect`、`path_opti`、`path_searching2d`、`width_check`、`ot_process`、`plan_rrt`、`to_visualize`、`multi_turtlebot3_simulation`、TurtleBot/Realsense 描述与 Gazebo 插件。
- `src/robosar/robosar_emergency_response` 是 RoboSAR 与 DEFORM 的包装层，负责 launch、配置、任务管理、目标派发、到达屏障、控制权切换、安全层和任务控制器。
- 搜索组仍由 RoboSAR 原有链路负责，不把响应组注册进 frontier/coverage 搜索任务分配器。
- Webots 验证、ROS1/ROS2 桥接和 ROS2 容器说明放在 `ROS2swarm_w` 相关文档中，本文件只描述 ROS1/Gazebo 集成。

## 3. 机器人和坐标系

搜索组：

- 名称：`agent_0 ... agent_4`
- 职责：未知环境探索、SLAM、应急对象发现、原有任务生成与搜索控制。
- 控制链路保持 RoboSAR 原设计：

```text
/frontier_filter/filtered_frontiers
-> high_ta.py
-> task_allocation
-> MissionExecutive
-> MultiAStar
-> robosar_controller/lazy_traffic_controller
-> agent_i cmd_vel
```

响应组：

- 名称：`tb_0 ... tb_3`
- 模型：TurtleBot3 burger
- 职责：接收应急任务，编队导航到 staging pose，随后执行任务控制器。
- 第 0 阶段默认出生点在 `response_group_gazebo.launch` 中配置：

```text
tb_0: x=4.2, y=-4.6, yaw=1.5708
tb_1: x=4.8, y=-4.6, yaw=1.5708
tb_2: x=4.2, y=-5.2, yaw=1.5708
tb_3: x=4.8, y=-5.2, yaw=1.5708
```

Tepper 大系统默认出生点在 `tepper_response_demo.launch` 中配置：

```text
tb_0: x=17.00, y=8.70, yaw=0.0
tb_1: x=16.55, y=9.04, yaw=0.0
tb_2: x=16.55, y=8.36, yaw=0.0
tb_3: x=16.10, y=8.70, yaw=0.0
```

当前 launch 会发布 `map -> world` identity 静态变换，使 DEFORM world 口径与 RoboSAR `map` 口径对齐。DEFORM 目标接口 `/move_base_simple/goal` 只消费目标 `x/y`，不负责最终 yaw 对齐；yaw 由到达屏障或任务控制器侧处理。

## 4. 关键文件

运行入口：

```text
run_response_deform_demo.sh
run_tepper_response_demo.sh
run_response_controller_demo.sh
```

第 0 阶段 launch/config：

```text
src/robosar/robosar_emergency_response/launch/response_deform_demo.launch
src/robosar/robosar_emergency_response/launch/response_group_gazebo.launch
src/robosar/robosar_emergency_response/launch/response_group_deform.launch
src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml
src/robosar/robosar_emergency_response/config/response_goal_sequence.yaml
src/robosar/robosar_emergency_response/config/response_group.yaml
src/robosar/robosar_emergency_response/config/response_safety.yaml
src/robosar/robosar_emergency_response/config/deform/frm_shape_response4.yaml
```

Tepper 大系统 launch/config：

```text
src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch
src/robosar/robosar_emergency_response/config/tepper_response_group.yaml
src/robosar/robosar_emergency_response/config/response_safety.yaml
```

独立控制器 demo：

```text
src/robosar/robosar_emergency_response/launch/roundflocking_controller_demo.launch
src/robosar/robosar_emergency_response/launch/transport_controller_demo.launch
src/robosar/robosar_emergency_response/config/roundflocking_controller_demo.yaml
src/robosar/robosar_emergency_response/config/transport_controller_demo.yaml
src/robosar/robosar_emergency_response/worlds/roundflocking_controller_demo.world
src/robosar/robosar_emergency_response/worlds/transport_controller_demo.world
```

核心节点：

```text
scripts/response_goal_sequence_node.py
scripts/emergency_task_manager_node.py
scripts/deform_dispatch_bridge_node.py
scripts/response_group_coordinator_node.py
scripts/response_cmd_mux_node.py
scripts/response_global_info_node.py
scripts/response_task_controller_node.py
scripts/dynamic_obstacle_cloud_node.py
scripts/response_task_fixture_node.py
```

学习控制器资源：

```text
assets/checkpoints/roundflocking/flocking_genome_package_0.3.pkl
assets/checkpoints/transport/transport_genome_package.pkl
src/robosar/robosar_emergency_response/src/robosar_emergency_response/checkpoint_controller.py
src/robosar/robosar_emergency_response/src/robosar_emergency_response/roundflocking_heuristic.py
src/robosar/robosar_emergency_response/src/robosar_emergency_response/transport_heuristic.py
```

## 5. 第 0 阶段实现

第 0 阶段不依赖搜索组，不订阅 `/robosar_sim/emergency_events`，只验证响应组在 Gazebo/RViz 中的 DEFORM 编队导航。

启动方式：

```bash
cd masar_ws
./run_response_deform_demo.sh
```

常用选项：

```bash
./run_response_deform_demo.sh --vglrun
./run_response_deform_demo.sh --headless
./run_response_deform_demo.sh --dry-run
./run_response_deform_demo.sh --cleanup-only
```

默认 wrapper 会自动加载 `run_vnc_gpu.sh --env-only`，然后调用包内脚本。包内脚本默认使用隔离 ROS/Gazebo master：

```text
ROS_MASTER_URI=http://127.0.0.1:11411
GAZEBO_MASTER_URI=http://127.0.0.1:11445
```

第 0 阶段数据流：

```text
response_goal_sequence.yaml
-> response_goal_sequence_node.py
-> /response_group/active_task
-> /response_group/formation_goal
-> deform_dispatch_bridge_node.py
-> /move_base_simple/goal
-> DEFORM plan_manager_node x4
-> /tb_i/deform_cmd_vel
-> response_cmd_mux_node.py
-> /tb_i/cmd_vel
-> response_group_coordinator_node.py
-> /response_group/status
-> response_goal_sequence_node.py 推进下一个 goal
```

默认目标点来自 `response_goal_sequence.yaml`，共 5 个：

```text
(4.2, 4.6) -> (0.0, 4.6) -> (0.0, -4.6) -> (-4.2, -4.6) -> (-4.2, 4.6)
```

`response_group.yaml` 当前使用较宽松的到达判定，原因是 DEFORM 在窄通道和安全层作用下会留下稳定槽位残差：

```text
arrival_radius: 0.80
enable_goal_region_arrival: true
goal_region_robot_radius: 1.75
goal_region_center_radius: 0.85
navigation_timeout: 180.0
default_mode: NAV_FORMATION
```

验收口径：

- Gazebo 中只有 4 台 `tb_i` 响应机器人即可。
- RViz 中能看到目标点、队形槽位、当前状态和可视化 marker。
- 宽通道中队形相对展开，窄通道中能够压缩或重分配槽位。
- 不应出现长期互相顶死、撞墙或目标序列无法推进。

## 6. Tepper 大系统实现

Tepper 响应 demo 的启动脚本会按 RoboSAR 原有顺序启动搜索/任务基础栈，再启动响应组 overlay。

启动方式：

```bash
cd masar_ws
./run_tepper_response_demo.sh
```

常用选项：

```bash
./run_tepper_response_demo.sh --vglrun
./run_tepper_response_demo.sh --headless
./run_tepper_response_demo.sh --dry-run
./run_tepper_response_demo.sh use_spin_placeholder:=true
```

脚本当前启动顺序：

```text
roscore
rosparam set /use_sim_time true
roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch
roslaunch robosar_sim_bringup robosar_sim_tepper.launch
roslaunch slam_toolbox robosar_mapping_sim_g.launch
roslaunch robosar_task_generator task_generator.launch
roslaunch robosar_navigation mission_executive.launch
rosrun robosar_controller robosar_traffic_controller_node
rosrun robosar_task_allocator mission_commander.py -s=true -t='high'
python3 robosar_gui.py
roslaunch robosar_emergency_response tepper_response_demo.launch
```

`tepper_response_demo.launch` 内部不会再启动主 Gazebo，它通过 `response_group_gazebo.launch start_gazebo:=false` 只补充响应组 `tb_0 ... tb_3`，并叠加 DEFORM 和任务控制节点。

Tepper 响应数据流：

```text
/robosar_sim/emergency_events
/robosar_sim/emergency_objects
-> emergency_task_manager_node.py
-> /emergency/tasks
-> /emergency/active_task
-> /response_group/active_task
-> deform_dispatch_bridge_node.py
-> /move_base_simple/goal
-> DEFORM plan_manager_node x4
-> /tb_i/deform_cmd_vel
-> response_cmd_mux_node.py
-> /tb_i/cmd_vel
-> response_group_coordinator_node.py
-> /response_group/status
-> emergency_task_manager_node.py 切换 TASK_CONTROL
-> response_task_controller_node.py
-> /tb_i/task_cmd_vel
-> response_cmd_mux_node.py
-> /tb_i/cmd_vel
-> /response_group/task_result
-> emergency_task_manager_node.py 派发下一任务或自动返航
```

GUI 的 START/STOP/HOME 命令通过 `/system_mission_command` 参与控制。`tepper_response_group.yaml` 默认 `wait_for_start: true`，所以响应队列会等 START 后开始派发；STOP 会把响应组模式切到停止，HOME/自动返航使用保留任务 `task_id=-1000`。

## 7. 应急任务管理

消息定义位于 `src/robosar/robosar_messages/msg`：

```text
EmergencyObject.msg
EmergencyObjectArray.msg
EmergencyTask.msg
EmergencyTaskArray.msg
ResponseGroupStatus.msg
```

任务输入：

```text
/robosar_sim/emergency_events      robosar_messages/EmergencyObject
/robosar_sim/emergency_objects     robosar_messages/EmergencyObjectArray
/response_group/status             robosar_messages/ResponseGroupStatus
/response_group/task_result        robosar_messages/EmergencyTask
/system_mission_command            robosar_messages/mission_command
/response_group/start              std_msgs/Bool，可选
```

任务输出：

```text
/emergency/tasks                   robosar_messages/EmergencyTaskArray
/emergency/active_task             robosar_messages/EmergencyTask
/response_group/active_task        robosar_messages/EmergencyTask
/response_group/behavior_mode      std_msgs/String
/emergency/task_markers            visualization_msgs/MarkerArray
```

`emergency_task_manager_node.py` 当前逻辑：

- 按 `(object_type, object_id, scene_name)` 去重。
- 默认优先级为 `victim > fire > box`，同优先级按发现时间 FIFO。
- 大系统默认 `enable_priority_preemption: false`，不会在 EN_ROUTE 中途频繁抢占。
- 任务到达 staging pose 后，状态从 `EN_ROUTE` 切为 `ACTING`，同时发布 `TASK_CONTROL`。
- 收到 `/response_group/task_result` 的 DONE/FAILED 后，更新队列并派发下一任务。
- 所有已知应急任务完成后，默认 `auto_return_home: true`，响应组回到初始中心。

行为映射在 `task_manager.py` 中：

```text
VICTIM -> TASK_ROUNDFLOCKING
FIRE   -> TASK_ROUNDFLOCKING
BOX    -> TASK_TRANSPORT
```

当 `use_spin_placeholder:=true` 时，上述任务都会改为 `TASK_SPIN_6S_PLACEHOLDER`，用于第一阶段只验证“到达后切任务控制器”的闭环。

staging pose 计算：

```text
victim/fire:
  approach_direction = normalize(target_pose - response_group_center)
  staging_pose = target_pose - approach_direction * staging_distance

box:
  push_direction = normalize(goal_pose - box_pose)
  staging_pose = box_pose - push_direction * staging_distance
```

Tepper 默认 staging 参数：

```text
staging_distance_victim: 1.0
staging_distance_fire: 1.0
staging_distance_box: 1.0
default_hold_seconds: 20.0
```

## 8. DEFORM Vendor 适配

响应组不直接使用 vendor 原始 `plan_manager/launch/multi_run.launch`，因为原始 demo 偏 12 机器人配置。当前使用响应组专用 launch：

```text
src/robosar/robosar_emergency_response/launch/response_group_deform.launch
src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml
src/robosar/robosar_emergency_response/config/deform/frm_shape_response4.yaml
```

`frm_shape_response4.yaml` 当前配置 4 机器人单列/压缩队形：

```text
task_type: 1
vehicle_number: 4
slot: (0,0), (0,-0.8), (0,-1.6), (0,0.8)
```

`response_plan_manager_one_run.xml` 关键适配：

- 每台 `tb_i` 启动一个 `plan_manager_node`。
- 启动对应 `drone_detect.launch`，默认深度输入为 `/tb_i/camera/depth/image_filter`。
- `~cmd_vel` remap 到 `/tb_i/deform_cmd_vel`，不直接写 `/tb_i/cmd_vel`。
- `~grid_map/odom` remap 到 `/tb_i/odom`。
- `~grid_map/cloud` remap 到 `/response_group/dynamic_obstacle_cloud`。
- 显式设置 `plan_manager/obs_clearance=0.35` 和 `path_optimizer/obs_clearance=0.35`。
- 加载 `plan_env2d/env.yaml`、`plan_rrt/rrt_cfg.yaml` 和响应组 `frm_shape_response4.yaml`。
- 每台机器人启动 `ot_process` 的 `ot_srv.py`，用于槽位分配。

vendor `plan_manager` 当前已经有响应组所需的轻量状态出口：

```text
/tb_i/deform_reached    std_msgs/Bool
/tb_i/deform_debug      调试信息
```

新目标到来时会发布 `False`，DEFORM 内部判定到达后发布 `True`。`response_group_coordinator_node.py` 同时结合外部 odom/槽位/目标区域判定，避免依赖日志解析。

vendor 中还修正了编队压缩阈值相关逻辑，避免 `formation_reduce_robot_fraction` 被计算成 0 后导致任意避障信号都触发压缩。

## 9. 到达屏障

`response_group_coordinator_node.py` 订阅：

```text
/response_group/active_task
/formation_info
/tb_i/odom
/tb_i/deform_reached
```

发布：

```text
/response_group/status
/response_group/formation_slots
/response_group/state_text
```

当前到达判定由三层条件组成：

1. 严格槽位屏障：机器人接近自己分配的 staging slot。
2. 目标区域屏障：机器人和响应组中心进入 staging pose 周围的宽松区域。
3. DEFORM 内部到达：全员 `/tb_i/deform_reached=True`。

第 0 阶段以宽松区域和 `deform_reached` 为主，避免窄通道下槽位残差导致目标无法推进。Tepper 阶段对 `BOX` 使用更宽松的 transport 目标区域，因为响应组只需要到达箱体后方附近，后续接触/推送由 transport 任务控制器负责。

超时逻辑：

- 第 0 阶段默认 `navigation_timeout: 180.0`
- Tepper 阶段默认 `navigation_timeout: 300.0`
- 超时后发布 `ResponseGroupStatus.FAILED`

## 10. cmd_vel 控制权和安全层

响应组所有速度最终都经过 `response_cmd_mux_node.py`：

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

模式选择：

```text
NAV_FORMATION -> 采用 /tb_i/deform_cmd_vel
TASK_CONTROL  -> 采用 /tb_i/task_cmd_vel
STOP/FAILED   -> 输出零速度
```

安全层输入包括：

```text
/tb_i/scan
/tb_i/odom
/gazebo/model_states
/response_group/active_task
/response_group/behavior_mode
```

主要安全逻辑：

- LiDAR 软势场避障。
- 动态对象安全，默认关注 `agent_0 ... agent_4`。
- NAV_FORMATION 中对队友方向做 scan exclusion，避免响应组成员被互相当成墙。
- transport 任务可通过 `transport_task_bypass_safety: true` 放行接触箱体，否则推箱阶段会被安全层过早阻断。

`response_safety.yaml` 当前还包含 formation reduce、dynamic obstacle trigger/stop/reverse 等参数，是第 0 阶段和 Tepper 阶段共同加载的安全配置。

## 11. 动态障碍和局部地图

RoboSAR 的 `/sim_map` 是静态地图，不包含箱子、临时障碍、搜索机器人、响应机器人等动态实体。因此 DEFORM 变形不能只依赖 `/sim_map`。

当前机制：

- 每台 `tb_i` 的 Realsense/深度图经 `drone_detect` 输出 `/tb_i/camera/depth/image_filter`，减少把队友识别为障碍。
- `plan_env2d/grid_map` 使用深度图和点云更新局部 occupancy/ESDF。
- `dynamic_obstacle_cloud_node.py` 从 `/gazebo/model_states` 和可选 `/map` 生成 `/response_group/dynamic_obstacle_cloud`。
- `response_plan_manager_one_run.xml` 把该点云接入 DEFORM 的 `~grid_map/cloud`。

Tepper 中 `dynamic_obstacle_cloud_node.py` 当前配置：

```text
map_topic: /map
cloud_topic: /response_group/dynamic_obstacle_cloud
frame_id: world
obstacle_prefixes: [agent_]
include_static_map: true
clear_model_prefixes: [agent_, tb_]
```

这表示搜索机器人会作为动态障碍注入，响应组和搜索组附近的静态地图点会被清除，降低局部地图把机器人本体附近误认为不可通行的概率。

## 12. 任务控制器

`response_task_controller_node.py` 是第一阶段占位控制器和第二阶段学习/解析式控制器的统一入口。

输入：

```text
/response_group/behavior_mode
/response_group/active_task
/global_info
/tb_i/scan
```

输出：

```text
/tb_i/task_cmd_vel
/response_group/task_result
```

### 12.1 /global_info 适配

`response_global_info_node.py` 发布 ROS2swarm 兼容 JSON：

```text
/global_info
```

内容包括：

- `robots`：来自 `/tb_i/odom`，并从 `/gazebo/model_states` 补充其他模型。
- `objects`：来自 Gazebo 模型状态，过滤机器人和地面等非任务对象。
- 任务别名：
  - BOX 任务提供 `transport_box` 和 `target_zone`
  - VICTIM/FIRE 任务提供 `flocking_target`

这层适配让 ROS1 Gazebo 任务控制器可以复用 ROS2swarm/VMAS 训练出的观测口径。

### 12.2 占位自转

当 `use_spin_placeholder:=true` 或任务 behavior 为 `TASK_SPIN_6S_PLACEHOLDER` 时：

```text
spin_duration: 6.0
spin_angular_velocity: 0.6
```

控制器向所有 assigned robots 发布原地旋转，结束后发布 DONE。这是第一阶段验证“任务到达 -> 控制器接管 -> 完成反馈”的最小闭环。

### 12.3 Roundflocking

用途：

- `VICTIM`
- `FIRE`

当前 Tepper 默认：

```text
roundflocking_controller_type: heuristic
roundflocking_checkpoint_file: roundflocking/flocking_genome_package_0.3.pkl
roundflocking_world_scale: 1.75
roundflocking_completion_enabled: true
```

说明：

- 配置中保留 checkpoint 文件，可以通过参数切到 `roundflocking_controller_type:=checkpoint`。
- 大系统默认值实际是 `heuristic`，不是 checkpoint。
- 独立 demo 的 `roundflocking` 和 `roundflocking-checkpoint` 默认使用 checkpoint，`roundflocking-heuristic` 使用解析式控制器。
- DONE 判定优先使用 `RoundflockingCompletionMonitor`，要求机器人围绕目标产生实际角度推进；若未启用完成监测，则按持续时间结束。

### 12.4 Transport

用途：

- `BOX`

当前 Tepper 默认：

```text
transport_controller_type: checkpoint
transport_checkpoint_adapter_type: tb3_slot
transport_checkpoint_file: transport/transport_genome_package.pkl
goal_threshold: 0.15
complete_transport_on_timeout: false
```

说明：

- checkpoint 原始输出按 ROS2swarm 差速口径适配。
- Gazebo/TurtleBot 推箱使用 `tb3_slot` 接触槽位整形，避免 VMAS 中的理想动作直接套到物理 TB3 后无法稳定接触箱体。
- DONE 条件是箱体中心进入目标区域并保持，不能只靠超时完成。
- 独立 demo 支持 `transport-checkpoint` 和 `transport-heuristic`。

## 13. 独立控制器 demo

这些 demo 用于隔离验证任务控制器，不经过 DEFORM 导航和应急任务队列。

Roundflocking：

```bash
cd masar_ws
./run_response_controller_demo.sh roundflocking
./run_response_controller_demo.sh roundflocking-checkpoint
./run_response_controller_demo.sh roundflocking-heuristic
```

Transport：

```bash
cd masar_ws
./run_response_controller_demo.sh transport
./run_response_controller_demo.sh transport-checkpoint
./run_response_controller_demo.sh transport-heuristic
```

脚本默认使用隔离 ROS/Gazebo master：

```text
ROS_MASTER_URI=http://127.0.0.1:11611
GAZEBO_MASTER_URI=http://127.0.0.1:11645
```

transport demo 默认打开 collision debug marker，便于观察接触槽位和箱体目标区。

## 14. 与早期设计的差异

当前实现相对于早期文档已经发生这些变化：

- 第 0 阶段 world 名称是 `deform_navigation_demo.world`，不是 `deform_navigation_reference.world`。
- `robosar_emergency_response` 包、launch、config、脚本已经存在，不再是“建议新增”。
- `drone_detect` 已经在 vendor 中恢复，并且响应组默认使用 filtered depth。
- `PlanManager` 已增加 `/tb_i/deform_reached`，到达屏障不再依赖日志解析。
- DEFORM 输出已重映射到 `/tb_i/deform_cmd_vel`，最终 `/tb_i/cmd_vel` 由 mux 统一发布。
- `response_cmd_mux_node.py` 已包含 LiDAR 势场、动态对象安全和 transport bypass。
- `dynamic_obstacle_cloud_node.py` 已在 Tepper overlay 中启用。
- `tepper_response_group.yaml` 中 roundflocking 大系统默认是 `heuristic`，checkpoint 作为可切换选项保留。
- 第 0 阶段目标序列是 5 个点，不是只验收 3 个点。
- 当前测试目录中显式保留的源码测试主要是 `test/test_checkpoint_controller.py`，不能把缓存目录里的旧 pyc 当成正式测试文件。

## 15. 当前限制

仍需要注意的限制：

- 本集成只覆盖 ROS1/Gazebo，Webots 和 ROS1/ROS2 桥接不在本文件入口中。
- DEFORM 的 `/move_base_simple/goal` 仍是全局目标接口，同一 ROS master 下只适合一个响应组实例。
- `/global_info` 适配依赖 Gazebo model name 和任务别名，换 world 或换对象命名时需要同步配置。
- Realsense/Gazebo 深度、CasADi/IPOPT、POT 等依赖必须在容器镜像内可用。
- Transport 物理推箱对摩擦、接触槽位和安全 bypass 参数敏感，换模型或箱体尺寸后需要重新调参。
- 第 0 阶段和 Tepper 阶段的到达半径是按当前 world 和安全层调过的工程参数，不宜直接作为理论精度指标。

## 16. 建议验证顺序

推荐按以下顺序验证，便于定位问题：

1. 检查脚本解析和启动命令：

```bash
cd masar_ws
./run_response_deform_demo.sh --dry-run
./run_tepper_response_demo.sh --dry-run
./run_response_controller_demo.sh --dry-run transport
```

2. 先跑第 0 阶段：

```bash
./run_response_deform_demo.sh --vglrun
```

确认 4 台 `tb_i` 能按 5 个目标点推进，观察 `/response_group/status`、`/tb_i/deform_reached` 和 `/tb_i/cmd_vel`。

3. 单独验证任务控制器：

```bash
./run_response_controller_demo.sh roundflocking-heuristic --vglrun
./run_response_controller_demo.sh transport-checkpoint --vglrun
```

4. 最后跑 Tepper 大系统：

```bash
./run_tepper_response_demo.sh --vglrun
```

第一阶段只看闭环时可使用：

```bash
./run_tepper_response_demo.sh --vglrun use_spin_placeholder:=true
```

第二阶段使用默认 `use_spin_placeholder:=false`，进入 roundflocking/transport 任务控制器。

## 17. 最小排错线索

目标不推进：

- 查看 `/response_group/status` 的 `status/detail`。
- 查看 `/tb_i/deform_reached` 是否全员为 True。
- 查看 `/formation_info` 是否持续发布。
- 检查 `response_group.yaml` 或 `tepper_response_group.yaml` 中的 goal region 半径。

机器人不动：

- 查看 `/response_group/behavior_mode`。
- NAV_FORMATION 模式检查 `/tb_i/deform_cmd_vel`。
- TASK_CONTROL 模式检查 `/tb_i/task_cmd_vel`。
- 最终检查 `/tb_i/raw_selected_cmd_vel`、`/tb_i/safety_filtered_cmd_vel` 和 `/tb_i/cmd_vel`。

DEFORM 把队友当障碍：

- 确认 `response_plan_manager_one_run.xml` 的 `depth_topic` 是 `/tb_i/camera/depth/image_filter`。
- 检查 `drone_detect` 是否启动。
- 检查安全层 teammate scan exclusion 参数。

Tepper 中搜索机器人阻塞响应组：

- 查看 `/response_group/dynamic_obstacle_cloud` 是否有点云。
- 查看 `dynamic_obstacle_cloud_node.py` 的 `obstacle_prefixes` 和 `clear_model_prefixes`。
- 检查 `/gazebo/model_states` 中模型命名是否仍是 `agent_` 和 `tb_` 前缀。

transport 推不动箱体：

- 确认当前任务 behavior 是 `TASK_TRANSPORT`。
- 确认 `transport_task_bypass_safety` 已按需求开启。
- 查看 `/global_info` 中是否存在 `transport_box` 和 `target_zone`。
- 根据箱体尺寸和摩擦重新调 `transport_tb3_*` 槽位参数。
