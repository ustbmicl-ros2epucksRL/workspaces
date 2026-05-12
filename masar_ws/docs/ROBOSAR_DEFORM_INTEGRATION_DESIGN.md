# RoboSAR 与 DEFORM 集成设计说明

## 1. 说明与结论

本文基于当前环境中实际可读到的两个代码库进行分析：

- `RoboSAR`: `/root/workspace/masar_ws`
- `DEFORM`: `/root/workspace/DEFORM`

结论先行：

**可以完成实现**，但不是把 DEFORM “整仓直接并到” RoboSAR 就能跑通，而是需要做一层明确的**任务编排与接口适配**。  
推荐的总体路线是：

1. 保留 RoboSAR 作为**搜索组**主系统，继续负责多机器人 SLAM、探索、覆盖、任务生成与发现。
2. 新增一个**应急任务管理层**，把搜索组发现的受害者、火源、箱子等目标转成结构化应急任务。
3. 引入 5 台 TurtleBot 作为**响应组**，复用 DEFORM 的编队导航与编队重构能力，执行火源环绕、受害者围护、箱体推运等群体子任务。
4. 搜索组与响应组只通过“任务/目标位姿/状态”交互，不强行共用同一个控制器。

如果接受 **ROS1 + RViz + 现有 PyQt GUI** 作为第一阶段可视化方案，则实现风险可控。  
如果要求 **全部迁到 ROS2 + RViz2 原生运行**，工作量会显著增大，因为这两个仓库当前都以 **ROS1 Noetic + catkin** 为主。

---

## 1.1 当前实现状态（已落地）

截至本次迭代，第一步中的“发现事件流与精确位置回填”已经在当前仓库内完成了首版实现，代码位置如下：

- 消息定义：
  - `src/robosar/robosar_messages/msg/EmergencyObject.msg`
  - `src/robosar/robosar_messages/msg/EmergencyObjectArray.msg`
- 仿真与发现逻辑：
  - `src/robosar/robosar_sim_bringup/scripts/AprilTagSim.py`
- Gazebo / Webots 启动适配：
  - `src/robosar/robosar_sim_bringup/launch/robosar_sim_w.launch`
- Webots 场景实体生成：
  - `/root/workspace/ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py`
  - `/root/workspace/ROS2swarm_w/src/launch_webots/launch_webots/environment_generator.py`
  - `/root/workspace/ROS2swarm_w/src/launch_webots/launch_webots/global_info_supervisor.py`
  - `/root/workspace/ROS2swarm_w/start_robosar_webots.sh`
- RViz 可视化：
  - `src/robosar/robosar_sim_bringup/rviz/multi_robot.rviz`
- GUI 摘要统计：
  - `src/robosar/robosar_gui/src/robosar_gui.py`

本次实际新增/补全的能力包括：

1. **victim / fire / box 三类对象统一事件流**
   - 新增事件 topic：
     - `/robosar_sim/emergency_events`
   - 新增状态快照 topic：
     - `/robosar_sim/emergency_objects`

2. **fire / box 的真实发现闭环**
   - 不再只有静态 Marker。
   - 搜索机器人进入检测半径后，会真正触发发现事件，并发布结构化对象消息。

3. **精确位置回填**
   - 当前仿真阶段采用“发现触发 + 场景真值回填”的简单稳定方案。
   - 即：机器人必须先接近对象才算发现，但一旦发现，对外发布的是场景中的精确 `map` 坐标。

4. **箱子目标位姿与场景坐标校验**
   - `sim_scenes.json` 现在可以显式维护 `transport_targets`。
   - 若未显式给出，仿真节点仍会基于 `/sim_map` 自动计算一个：
     - 尽量近
     - 尽量直线
     - 不穿墙
     的目标位置。
   - `AprilTagSim.py` 现在还会对 victim / fire / box / target 做运行时兜底校验：
     - 避开墙边低清障点
     - 避开 Gazebo 初始机器人出生点
     - 箱子目标若不再满足直线可推，则自动重算

5. **Gazebo 中的应急实体**
   - victim：生成 Gazebo 自带 `walk.dae` 人体网格的静态受害者模型
   - fire：生成静态圆柱体
   - transport box：生成可碰撞、可推动的动态箱体
   - target zone：生成目标区域可视化圆盘

6. **Webots 中的应急实体**
   - 直接复用同一份 `sim_scenes.json`
   - 生成 victim / fire / transport box / target zone 对应实体
   - 机器人随机出生位姿会避开这些实体，避免初始重叠

7. **当前项目可视化衔接**
   - RViz 中继续沿用：
     - `/slam_toolbox/victim_markers`
     - `/robosar_sim/fire_markers`
     - `/robosar_sim/transport_markers`
   - 其中 `fire_markers` / `transport_markers` 现在同时包含：
     - ground truth
     - discovered overlay
     - box target zone
   - GUI 中新增了：
     - “发现火源”
     - “发现箱子”
     两个摘要统计框

### 当前事件消息语义

`EmergencyObject.msg` 当前承载：

- 对象类型：`VICTIM / FIRE / BOX`
- 对象 id
- 是否已发现
- 首次发现该对象的机器人名
- 精确 `map` 位姿
- 若为箱子，则额外给出 `goal_pose`

这已经足够支撑下一步的：

- 应急任务管理器
- 响应组派单
- 编队控制器接入

### 当前控制器接入约定（参考，未接线）

当前这一轮只完成了“发现流 + 场景实体 + 坐标清洗”的第一步，响应组控制器还没有真正接入 RoboSAR 的调度链。  
但后续接线约定已经明确如下：

- victim：使用 `roundflocking`
  - 检查点目录：`/root/workspace/ROS2swarm_w/src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/roundflocking`
  - 当前可直接参考：`flocking_genome_package_gen99.pkl`
- fire：第一阶段也先使用 `roundflocking` 占位
  - 但任务类型和调度入口要独立保留，后续便于替换成专用火源环绕控制器
- box：使用 `transport`
  - 检查点目录：`/root/workspace/ROS2swarm_w/src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/transport`
  - 当前可直接参考：`transport_genome_package_gen299.pkl`（另有 `249/264/278` 可做对比）

### 当前已知环境问题

当前工作空间在**全量重新配置**时存在一个与本次改动无关的 Python 环境问题：

- `setuptools` / `importlib_metadata` 版本不匹配
- 触发点：
  - `robosar_task_generator` 的 `catkin_python_setup()`
  - `ROS2swarm_w` 中 `launch_webots` 的 `colcon build`

因此：

- 本次改动已完成：
  - Python 语法检查
  - 场景坐标 / 直线推箱 / 出生点避让脚本检查
- 全量 `catkin_make` / `colcon build` 仍可能因为该环境问题失败

这不是本次实现引入的问题，但后续如果要做全工作空间重编译，建议先修复该 Python 环境。

---

## 2. 当前两个项目的真实实现情况

## 2.1 RoboSAR 当前实现

RoboSAR 已经具备完整的“搜索组”闭环。

### 2.1.1 关键链路

1. **多机器人 SLAM**
  - 入口配置：`src/robosar/robosar_SLAM_toolbox/slam_toolbox/config/robosar_mapping_g.yaml`
  - 已支持 5 台机器人：
    - `agent_0 ... agent_4`
    - 激光话题：`/robosar_agent_bringup_node/agent_i/feedback/scan`
    - AprilTag 话题：`/robosar_agent_bringup_node/agent_i/feedback/apriltag`
  - 地图输出：
    - `/slam_toolbox/map`
    - `/map`（经 `map_filter.py` 处理后转发）
2. **任务生成**
  - 覆盖任务图：`src/robosar/robosar_task_generator/src/robosar_task_generator/task_graph.cpp`
  - Frontier 过滤：`src/robosar/robosar_task_generator/scripts/robosar_task_generator/frontier_filter.py`
  - 任务接口：
    - `/robosar_task_generator/task_graph_getter`
    - `/robosar_task_generator/task_graph_setter`
    - `/frontier_filter/filtered_frontiers`
3. **任务分配**
  - 高层分配器：`src/robosar/robosar_task_allocator/src/high_ta.py`
  - 启停器：`src/robosar/robosar_task_allocator/src/mission_commander.py`
  - 当前发布统一的 `task_allocation` 消息，供执行层消费。
4. **路径规划与执行**
  - 任务执行入口：`src/robosar/robosar_navigation/include/robosar_navigation/mission_executive.hpp`
  - `MissionExecutive` 收到 `task_allocation` 后，调用 `MultiAStar` 规划路径，并通过服务
  `robosar_controller/lazy_traffic_controller`
  把路径发给控制器。
5. **多机器人交通控制**
  - 控制器：`src/robosar/robosar_controller/src/lazy_traffic_controller.cpp`
  - 单机器人控制代理：`src/robosar/robosar_controller/src/lazy_traffic_agent.cpp`
  - 当前采用：
    - Pure Pursuit 跟踪路径
    - RVO 避碰
    - 覆盖任务到点后原地旋转搜索

### 2.1.2 对应急任务已经具备的基础

RoboSAR 当前已经有一部分应急相关基础，不需要从零开始：

1. **仿真中已有受害者/火源/箱子静态配置**
  - 文件：`src/robosar/robosar_sim_bringup/config/sim_scenes.json`
  - 当前场景里已经定义了：
    - `victims`
    - `fire_sources`
    - `transport_objects`
    - `transport_targets`
    - `robot_spawn_positions`（Gazebo 场景）
2. **仿真节点已经发布相关 Marker**
  - 文件：`src/robosar/robosar_sim_bringup/scripts/AprilTagSim.py`
  - 当前行为：
    - 受害者：发布 AprilTag 检测与 Marker
    - 火源：发布 `/robosar_sim/fire_markers`
    - 运输物体：发布 `/robosar_sim/transport_markers`
3. **受害者已经有“发现”闭环**
  - `AprilTagSim.py` 会在机器人进入检测半径后，向该机器人对应的
   `/robosar_agent_bringup_node/<agent>/feedback/apriltag`
   发布检测结果。
4. **火源 / 箱子也已经有“发现”闭环**
  - 机器人进入检测半径后，会触发：
    - `/robosar_sim/emergency_events`
    - `/robosar_sim/emergency_objects`
  - 并回填真值 `map` 坐标与箱子 `goal_pose`
5. **Gazebo / Webots 的实体生成已经同源**
  - Gazebo：由 `AprilTagSim.py` 直接生成
  - Webots：由 `launch_webots` 读取同一份 `sim_scenes.json` 注入世界

### 2.1.3 当前系统的限制

RoboSAR 目前还没有下面这些能力：

1. 没有统一的 `EmergencyTask` 概念。
2. 没有“发现后转交另一组机器人”的调度器。
3. 当前任务分配器默认只服务于搜索机器人，不能直接拿来给 TurtleBot 响应组用。
4. 当前还没有把 `roundflocking / transport` 真正接到响应组自动派单与状态回传链路里。
5. 火源当前虽然已可发现并有实体，但控制行为仍只是后续将接入的 `roundflocking` 占位任务。
6. GUI 目前是 ROS1 PyQt，RViz 目前是 ROS1 RViz，不是 RViz2。

---

## 2.2 DEFORM 当前实现

DEFORM 已经具备完整的“响应组编队导航”基础，但它是一个独立系统。

### 2.2.1 核心入口

1. 入口节点：
  - `src/plan_manager/src/formation_manager_node.cpp`
  - 主类：`src/plan_manager/include/plan_manager/plan_manager.h`
  - 实现：`src/plan_manager/src/plan_manager.cpp`
2. 启动方式：
  - 每台 TurtleBot 一个 `plan_manager_node`
  - 批量启动文件：`src/plan_manager/launch/multi_run.launch`
  - 单机参数模板：`src/plan_manager/launch/one_run.xml`

### 2.2.2 DEFORM 实际工作机制

每台机器人本地都跑一套相同的 `PlanManager`，共享：

- `/move_base_simple/goal` 作为全局目标输入
- `/formation_info` 作为编队形态/重映射信息共享
- `/path_point_sharing` 作为 leader 路点共享
- `/current_width` 作为可通行宽度共享

`PlanManager` 内部状态机大致是：

- `WAIT_ODOM_OR_TARGET`
- `FRM_INIT_PATH`
- `FRM_KEEP`
- `AVOID_OBS`
- `APPROACH_GOAL`
- `EMERGENCY_STOP`

实际控制链路包括：

1. `plan_env2d/grid_map`
  - 本地 2D 占据/ESDF
  - 依赖每台机器人深度图和里程计
2. `path_searching2d`
  - A* 路径搜索
3. `width_check`
  - 估计当前路径前方可通过宽度
  - 决定是否压缩/扩展编队
4. `ot_process`
  - 用最优传输分配机器人到编队槽位
5. `traj_opti`
  - NMPC 轨迹优化控制
6. `to_visualize`
  - RViz marker/path 可视化

### 2.2.3 DEFORM 中对本任务最有价值的部分

1. **5 台以上移动机器人编队导航已经打通**
2. **队形可根据通道宽度动态收缩/展开**
3. **队形槽位重分配已经存在**
4. **仿真 TurtleBot + 深度相机 + odom + RViz 现成**
5. **可定义任意目标编队形状**
  - `task_type=2` 时，`frm_shape.yaml` 中的 `desired_shape_positions` 可描述任意几何结构

### 2.2.4 DEFORM 当前与 RoboSAR 的直接不兼容点

这是本次集成的关键。

1. **机器人命名空间不同**
  - RoboSAR：`agent_i`
  - DEFORM：`tb_i`
2. **控制入口不同**
  - RoboSAR：服务下发路径到 `lazy_traffic_controller`
  - DEFORM：统一监听 `/move_base_simple/goal`
3. **感知来源不同**
  - RoboSAR 搜索组依赖 SLAM 全局地图
  - DEFORM 响应组依赖每台 TurtleBot 的本地深度图和里程计
4. **全局坐标系默认不同**
  - RoboSAR 主坐标系：`map`
  - DEFORM 代码中多数可视化与规划默认以 `world` 为固定系
5. **任务模式是静态参数，不是动态任务接口**
  - `frm_shape.yaml` 中 `task_type` 是启动参数
  - 当前不支持“同一批机器人连续执行 navigation -> fire_ring -> push_box -> 下一个任务”
6. **DEFORM 默认是独立系统，不认识 RoboSAR 的任务消息**

---

## 3. 可行性判断

## 3.1 总体可行

在当前代码基础上，这个需求是**可实现的**，推荐分阶段完成。

最合理的做法不是把两个系统强行揉成一个控制器，而是：

- RoboSAR 继续负责 **搜索、建图、定位、发现、应急任务登记**
- DEFORM 负责 **响应组编队前往与编队执行**
- 中间新增一个 **应急任务管理与桥接层**

## 3.2 不能直接“无改动复用”的部分

下面这些不能直接拿来就跑：

1. DEFORM 的 `/move_base_simple/goal` 手工交互启动流程
2. DEFORM 的静态 `task_type`
3. DEFORM 默认 12 机器人编队 launch
4. RoboSAR 当前任务分配器直接驱动 TurtleBot 响应组
5. RViz2 原生可视化

## 3.3 风险最高的点

1. **真正的多机器人物理推箱子**
  - 这是所有子任务中最难稳定的部分。
  - 路径到点、环绕到点都比“稳定推物体到目标区”简单。
2. **严格要求 RViz2/ROS2 原生化**
  - 两个仓库都是 ROS1 Noetic。
  - 第一阶段建议先用 ROS1 RViz + PyQt GUI 做完功能闭环。
3. **把 DEFORM 的局部感知栈与 RoboSAR 全局地图完全统一**
  - 可以做，但不建议在第一阶段就追求完全统一。

---

## 4. 推荐的总体架构

## 4.1 双机器人组架构

推荐把系统明确分成两组：

### A. 搜索组

- 机器人：现有 `agent_0 ... agent_4`
- 继续使用 RoboSAR 全部现有链路：
  - 多机器人 SLAM
  - frontier / coverage 任务生成
  - 任务分配
  - 路径规划
  - `lazy_traffic_controller`

### B. 响应组

- 机器人：`tb_0 ... tb_4`
- 使用 DEFORM 改造后的编队执行栈：
  - 编队导航
  - 编队重构
  - 队形分配
  - 编队行为执行

### C. 中间桥接层

新增一个独立模块，负责：

1. 接收搜索组发现结果
2. 生成标准化应急任务
3. 去重/确认/排队
4. 给响应组派发任务
5. 跟踪任务状态
6. 更新 GUI/RViz

---

## 4.2 为什么不要把响应组直接塞进 RoboSAR 当前任务分配器

这是一个非常重要的实现边界。

RoboSAR 当前任务分配器默认认为：

- 所有活跃机器人都属于搜索队
- 都应该接 frontier / coverage 任务
- 都应该通过 `lazy_traffic_controller` 执行

如果把 `tb_0 ... tb_4` 也注册成当前“活跃 agent”，会立刻出现两个问题：

1. 响应组会被错误地分配探索任务。
2. 控制链会冲突：
  - RoboSAR 控制器想发 `/robosar_agent_bringup_node/<agent>/cmd_vel`
  - DEFORM 想发 `/tb_i/cmd_vel`

因此：

**响应组必须作为一套独立的机器人池存在，不能并入现有 RoboSAR 搜索任务分配器。**

---

## 5. 建议新增的任务模型

## 5.1 新增统一应急任务对象

建议新增一个独立消息包，例如：

- `robosar_emergency_msgs`

建议消息：

### `EmergencyDetection.msg`

```text
std_msgs/Header header
uint8 VICTIM=1
uint8 FIRE=2
uint8 BOX=3
uint8 task_type
int32 object_id
string detector_robot
geometry_msgs/PoseStamped coarse_pose
geometry_msgs/PoseStamped precise_pose
float32 confidence
```

### `EmergencyTask.msg`

```text
std_msgs/Header header
int32 task_id
uint8 VICTIM=1
uint8 FIRE=2
uint8 BOX=3
uint8 task_type
uint8 NEW=1
uint8 CONFIRMED=2
uint8 DISPATCHED=3
uint8 EN_ROUTE=4
uint8 ACTING=5
uint8 DONE=6
uint8 FAILED=7
uint8 status
int32 object_id
string source_robot
geometry_msgs/PoseStamped precise_pose
geometry_msgs/PoseStamped staging_pose
geometry_msgs/PoseStamped goal_pose
float32 confidence
float32 hold_seconds
```

### `ResponseTask.action`

```text
# goal
robosar_emergency_msgs/EmergencyTask task
---
# result
bool success
int32 task_id
string message
---
# feedback
uint8 stage
string stage_text
geometry_msgs/PoseStamped formation_center
```

---

## 5.2 任务生命周期

建议统一为：

1. `NEW`
  - 搜索组首次发现
2. `CONFIRMED`
  - 已通过简单规则确认并得到精确坐标
3. `DISPATCHED`
  - 已派给响应组
4. `EN_ROUTE`
  - 响应组正在前往
5. `ACTING`
  - 正在执行火源环绕/受害者围护/箱体推运
6. `DONE`
  - 任务完成
7. `FAILED`
  - 超时、队伍异常、目标不可达等

---

## 6. “发现后给出精确位置”的最简单可实现方案

用户已经明确允许“用简单方式给出精确位置”，这对整体实现很关键。

## 6.1 仿真阶段推荐方案

对于当前仿真，最简单且最稳的方案是：

1. 搜索机器人必须先进入目标发现半径，才能算“发现”
2. 一旦发现成功，不再使用粗略近距离点，而是直接读取场景配置中的**真实静态目标坐标**
3. 将该真实坐标作为 `precise_pose`

这在当前代码里非常自然，因为：

- `sim_scenes.json` 已经维护了静态对象真值
- `AprilTagSim.py` 已经按场景加载：
  - `victims`
  - `fire_sources`
  - `transport_objects`

因此可直接扩展为：

- 受害者：继续沿用 AprilTag 检测成功后回填真值坐标
- 火源：新增“进入半径则发布 fire detection”，并回填火源真值坐标
- 箱子：新增“进入半径则发布 transport object detection”，并回填箱子真值坐标

### 这个方案的优点

1. 实现最简单
2. 适合当前静态定义场景
3. 不影响搜索行为的真实性，因为仍然要求机器人先“接近并发现”
4. 不会因为单次噪声导致响应组跑偏

## 6.2 若希望稍微真实一点

可以进一步把搜索机器人当前位姿与目标相对位姿做 TF 变换：

```text
map_pose = T_map_base * T_base_object
```

并对多次观测做均值滤波，再输出 `precise_pose`。

但对当前第一阶段，不建议优先做这个。

---

## 7. 搜索组到响应组的完整任务流

推荐流程如下。

## 7.1 搜索阶段

1. RoboSAR 搜索组继续按现有机制做探索、覆盖、建图。
2. 搜索机器人在任务点附近触发应急发现：
  - victim: 现有 AprilTag 检测
  - fire: 新增 proximity/scene-based detection
  - box: 新增 proximity/scene-based detection

## 7.2 任务确认

新增 `EmergencyTaskManager` 节点：

1. 订阅所有发现事件
2. 根据 `object_id + 半径阈值` 去重
3. 生成 `EmergencyTask`
4. 写入任务池

建议简单优先级：

- `victim`
- `fire`
- `box`

同优先级按发现时间先后执行。

## 7.3 派发给响应组

当响应组空闲时：

1. 从任务池取出一个最高优先级任务
2. 计算 `staging_pose`
3. 发送给响应组 action server
4. 将任务状态置为 `DISPATCHED`

## 7.4 响应组执行

响应组执行分两段：

1. **前往阶段**
  - 以紧凑队形或通行友好队形前往任务点
2. **任务阶段**
  - 按任务类型切换不同编队几何与行为模板

## 7.5 完成与继续下一个任务

1. 响应组完成当前任务后，反馈 `DONE`
2. `EmergencyTaskManager` 更新任务状态
3. 响应组回到 `READY`
4. 自动领取下一个待执行任务

---

## 8. DEFORM 中哪些部分可以直接搬，哪些必须改

## 8.1 可直接复用

这些部分建议直接带过来：

### 1. TurtleBot 多机器人仿真

- `src/utility/tb_simulation/multi_turtlebot3_simulation`
- `src/utility/tb_simulation/turtlebot3`
- `src/utility/tb_simulation/realsense2_description`
- `src/utility/tb_simulation/realsense_gazebo_plugin`

作用：

- 提供 5 台 TurtleBot 的 Gazebo 仿真、里程计、激光、深度相机

### 2. 编队槽位分配

- `src/planning/ot_process`

作用：

- 用最优传输做机器人到编队槽位的映射
- 对 5 台响应组机器人非常合适

### 3. 局部轨迹优化控制

- `src/control/traj_opti`

作用：

- NMPC 局部控制器
- 可直接作为响应组底层局部控制器

### 4. RViz 编队可视化

- `src/utility/to_visualize`

作用：

- 队形、路径、局部目标、宽度检测等 marker

### 5. `PlanManager` 的编队状态机主体

- `src/plan_manager/src/plan_manager.cpp`

可以复用的核心思想：

- leader / follower 协同
- 编队旋转与共享
- 编队收缩/扩展
- 槽位重映射
- 编队一致性局部目标

## 8.2 需要适配后复用

### 1. `plan_manager`

必须修改的点：

1. 不再依赖手工 RViz 发布 `/move_base_simple/goal`
2. 改成 action/service/topic 驱动
3. 从“单一 task_type”改为“每个任务动态切换模式”
4. 从默认 12 机器人改成 5 机器人配置
5. 从 `world` 固定系改成可配置 `global_frame`

### 2. `frm_shape.yaml`

当前文件是静态配置，适合 demo，不适合连续任务。

需要改成：

- 预定义多个队形模板
- 或允许任务消息直接带入目标队形槽位

### 3. `multi_run.launch`

当前默认 12 台机器人，需要精简为 5 台：

- `tb_0 ... tb_4`

### 4. `one_run.xml`

需要改：

- `tb_num=5`
- 目标输入 remap
- global frame remap
- 响应组专用 topic namespace

## 8.3 第一阶段不建议硬搬的部分

### 1. 把 DEFORM 直接当成 RoboSAR 搜索控制器

不建议。

原因：

- 两者任务模型不同
- 控制入口不同
- 机器人模型不同

### 2. 完全复用 DEFORM 当前 demo 启动方式

不建议。

原因：

- 依赖手动 RViz 2D Nav Goal
- 不支持任务队列
- 不支持按任务类型自动切换

### 3. 强行统一 RoboSAR 和 DEFORM 的底层感知地图实现

第一阶段不建议。

更现实的方案是：

- 搜索组：继续用 RoboSAR 的全局 SLAM 地图
- 响应组：保留 DEFORM 的局部深度/局部规避
- 只共享全局任务目标位姿

---

## 9. 推荐的响应组行为设计

以下是最适合当前需求的 3 类行为模板。

## 9.1 受害者任务

### 目标

响应组基于已定位受害者前往并围护。

### 推荐动作

1. 以紧凑编队前往 `staging_pose`
2. 到点后切换到 **半环/小环形编队**
3. 保持若干秒，表示已实施围护/施救

### 建议模板

- 5 机器人半环
- 半径：`0.6 ~ 0.9m`

### 完成条件

- 队形收敛
- 保持 `T_rescue_hold` 秒

## 9.2 火源任务

### 目标

响应组围绕火源形成环状编队，保持若干秒，模拟灭火。

### 推荐动作

1. 以通行友好编队前往火源附近
2. 切换到 **5 点环形编队**
3. 围绕火源保持固定时长

### 建议模板

- 五边形环绕
- 半径：`0.7 ~ 1.0m`

### 完成条件

- 队形收敛
- 保持 `T_fire_hold` 秒

### 与 DEFORM 的关系

这个模式最适合直接复用 DEFORM 的 `task_type=2` 任意目标队形能力。

## 9.3 箱体任务

### 目标

响应组到达箱子后协同推动到目标区域。

### 推荐动作

1. 前往箱子后方 `staging_pose`
2. 排成推送线型
3. 以低速一致向前推
4. 将箱子送到 `target_zone`

### 最简模板

- 5 机器人一字排开
- 队形中心在箱子后方
- 推送方向由 `target_zone - box_pose` 决定

### 完成条件

- 箱体中心进入目标区域

### 现实提醒

这一项是最脆弱的部分。

建议分两阶段：

1. **简化版**
  - 箱体运动可允许低保真
  - 到达后判定成功
2. **增强版**
  - 做真实物理接触推动
  - 再调摩擦、质量、速度限制

---

## 10. 对 DEFORM 代码最关键的改造建议

## 10.1 把手工 goal 改成程序调度接口

当前：

- `PlanManager::subGlobalGoal()` 订阅 `/move_base_simple/goal`

建议改成：

- 新增 action server：`/response_group/execute_task`
- 或新增 service：`/response_group/dispatch_task`

其中任务内容包含：

- `task_type`
- `precise_pose`
- `staging_pose`
- `goal_pose`
- `behavior_mode`
- `formation_template`

这样响应组就能自动领取任务，而不是依赖 RViz 人工点击。

## 10.2 支持运行时切换行为模式

当前 DEFORM 的 `task_type` 来源于启动参数。

建议改为运行时模式：

- `NAV_TO_STAGING`
- `FIRE_RING`
- `VICTIM_GUARD`
- `BOX_PUSH`
- `RETURN_READY`

这样同一批 5 台 TurtleBot 可以反复执行多个任务，而不用每次重启节点。

## 10.3 把全局固定系改成可配置参数

当前 DEFORM 多处写死为 `world`。

建议统一引入：

- `~global_frame`

默认设为：

- `map`

这样更容易与 RoboSAR 搜索组统一。

## 10.4 5 机器人专用模板

建议直接为 5 台响应机器人定义以下模板：

1. `compact5`
2. `line5`
3. `ring5`
4. `arc5`
5. `push5`

其中：

- `ring5` 用于火源
- `arc5` 用于受害者
- `push5` 用于箱体

---

## 11. 仿真集成建议

## 11.1 第一阶段只建议做 Gazebo 集成

原因很简单：

- RoboSAR 的 Gazebo 仿真已经存在
- DEFORM 的 TurtleBot3 仿真也在 Gazebo
- 两边统一到 Gazebo 成本最低

不建议第一阶段做：

- RoboSAR Webots 分支 + DEFORM Gazebo 分支混跑

## 11.2 推荐的统一仿真方式

在一个 Gazebo world 中同时生成：

1. 5 台 RoboSAR 搜索机器人（Khepera）
2. 5 台 TurtleBot 响应机器人
3. 静态 victim / fire / transport box / target_zone

### 推荐命名

- 搜索组：`agent_0 ... agent_4`
- 响应组：`tb_0 ... tb_4`

### 推荐坐标系

- 统一全局固定系：`map`

若 Gazebo 世界坐标与地图坐标一致，可先使用：

- `map <-> world` identity

## 11.3 工作空间集成建议

不建议把两个工作空间的 `build/`、`devel/`、顶层 `src/CMakeLists.txt` 直接混在一起。

建议的做法是：

1. 保留 `masar_ws` 作为主工作空间
2. 只把 DEFORM 中需要的包拷贝或软链接进 `masar_ws/src`
3. 典型需要的包：
  - `plan_manager`
  - `traj_opti`
  - `plan_env2d`
  - `drone_detect`
  - `path_opti`
  - `path_searching2d`
  - `plan_rrt`
  - `width_check`
  - `ot_process`
  - `to_visualize`
  - `multi_turtlebot3_simulation`
  - `turtlebot3_description`
  - `realsense2_description`
  - `realsense_gazebo_plugin`

---

## 12. GUI 与 RViz 的建议

## 12.1 第一阶段建议

第一阶段建议直接沿用：

- RoboSAR PyQt GUI
- ROS1 RViz

这样实现最快。

## 12.2 GUI 需要新增的内容

建议在现有 `src/robosar/robosar_gui/src/robosar_gui.py` 基础上新增：

1. 应急任务列表
  - task id
  - type
  - status
  - source robot
  - assigned response group state
2. 搜索组/响应组分栏状态
  - 搜索组仍显示现有状态
  - 响应组新增 5 台 TurtleBot 状态
3. 当前活动任务信息
  - 目标类型
  - 精确坐标
  - 编队阶段
  - 已完成子任务数

## 12.3 RViz 建议显示项

建议新增以下 topics 的 Marker/Path：

1. `/emergency/tasks_markers`
  - 未发现/已发现/处理中/已完成不同颜色
2. `/response_group/formation_slots`
  - 响应组当前目标槽位
3. `/response_group/center_goal`
  - 编队中心目标
4. `/response_group/current_behavior_text`
  - 当前行为文本标注
5. `/response_group/path`
  - 响应组前往路径
6. `/robosar_sim/fire_markers`
7. `/robosar_sim/transport_markers`
8. `/slam_toolbox/victim_markers`

## 12.4 关于 RViz2

如果“RViz2”是硬性要求，需要明确下面这个现实：

1. 当前 RoboSAR 是 ROS1
2. 当前 DEFORM 也是 ROS1
3. 因此 RViz2 只能通过以下方式之一实现：
  - 用 `ros1_bridge` 把可视化 topic 桥接到 ROS2，再在 RViz2 显示
  - 或把关键模块迁移到 ROS2

所以建议：

- **第一阶段：ROS1 RViz 完成功能**
- **第二阶段：桥接到 RViz2**

---

## 13. 建议新增的桥接模块

建议新增 4 个包即可形成最小闭环。

## 13.1 `robosar_emergency_msgs`

定义：

- `EmergencyDetection.msg`
- `EmergencyTask.msg`
- `ResponseTask.action`

## 13.2 `robosar_emergency_manager`

职责：

1. 订阅搜索组发现事件
2. 去重与状态管理
3. 管理任务队列
4. 调度响应组
5. 发布可视化 markers

## 13.3 `robosar_response_bringup`

职责：

1. 启动 5 台 TurtleBot
2. 启动 DEFORM 相关节点
3. 统一 topic namespace
4. 统一 global frame

## 13.4 `robosar_response_behaviors`

职责：

1. 包装 DEFORM 形成 action server
2. 实现 `NAV_TO_STAGING / FIRE_RING / VICTIM_GUARD / BOX_PUSH`
3. 管理顺序任务执行

---

## 14. 推荐的实现阶段

## 阶段 1：发现与任务化

目标：

- 搜索组发现 victim / fire / box
- 生成结构化 `EmergencyTask`
- 在 GUI / RViz 中显示任务状态

建议改动：

- 扩展 `AprilTagSim.py`
- 新增 `robosar_emergency_manager`

## 阶段 2：响应组最小闭环

目标：

- 5 台 TurtleBot 在同一 Gazebo 场景中生成
- 能接收一个任务并编队前往目标附近

建议改动：

- 精简 DEFORM 为 5 机器人
- 将 `/move_base_simple/goal` 改为程序派发

## 阶段 3：火源与受害者行为

目标：

- fire: 环绕并保持
- victim: 围护并保持

建议优先完成这两个，因为稳定性远高于推箱子。

## 阶段 4：箱体推运

目标：

- 到达箱子
- 完成推运到目标区

建议先做简化版，再做真实物理交互版。

## 阶段 5：RViz2/ROS2 可视化桥接

仅在第一阶段全部功能闭环后再做。

---

## 15. 最现实的 MVP 范围

如果目标是尽快做出能演示、能验证的版本，建议 MVP 只做：

1. 搜索组继续探索与建图
2. victim / fire / box 三类目标被“发现”后生成应急任务
3. 用场景真值给出精确坐标
4. 响应组 5 台 TurtleBot 编队前往目标
5. fire: 环绕完成
6. victim: 围护完成
7. box: 先做“到位 + 队形对齐 + 简化完成判定”

这样能先把系统级链路打通：

**搜索 -> 发现 -> 精确定位 -> 派发 -> 编队前往 -> 协作执行 -> 切下一个任务**

---

## 16. 最终判断

### 可以完成

原因：

1. RoboSAR 已经完成搜索组闭环
2. DEFORM 已经完成响应组编队导航闭环
3. 当前仿真已经有 victim/fire/box 的静态定义
4. 当前项目里已经有受害者发现与 Marker 可视化基础
5. DEFORM 具备编队重构、NMPC、队形分配、仿真和可视化现成实现

### 但实现方式必须是“分层集成”

最合理的实现不是：

- 让 RoboSAR 直接控制 TurtleBot 响应组

而是：

- RoboSAR 负责发现与任务登记
- DEFORM 负责响应组编队执行
- 中间新增应急任务桥接层

### 直接可搬的 DEFORM 部分

- TurtleBot 多机器人仿真
- OT 槽位分配
- NMPC 轨迹优化
- 编队可视化
- 编队状态机主体

### 必须改造的部分

- goal 输入接口
- 动态任务模式
- 5 机器人模板
- global frame 统一
- 任务队列/状态反馈

### 最应谨慎处理的部分

- 真实物理推箱子
- 严格 RViz2 化

---

## 17. 建议后续实施顺序

如果要真正开始编码，建议按下面顺序落地：

1. 扩展 `AprilTagSim.py`，让 fire / box 也能产生日志化 detection
2. 新增 `robosar_emergency_msgs` 与 `robosar_emergency_manager`
3. 在 Gazebo 同场景中生成 5 台 TurtleBot 响应组
4. 把 DEFORM 改造成 5 机器人、程序可调度版
5. 先打通 `fire_ring`
6. 再打通 `victim_guard`
7. 最后做 `box_push`

这条路线风险最低，且每一阶段都能独立验证。
