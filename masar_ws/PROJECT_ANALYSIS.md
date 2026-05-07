# RoboSAR 项目全面分析文档

## 一、项目概述

RoboSAR（Robotic Search And Rescue）是一个基于 ROS Noetic 的**多机器人自主搜索与救援系统**。该系统使用一组移动机器人协同探索和建图未知环境，以搜索受困人员（victims）。系统涵盖多机器人 SLAM、任务生成、任务分配、路径规划、交通控制、仿真和图形界面等完整功能链。

该项目为 CMU MRSD（Master of Science in Robotic Systems Development）2022 年 F 组毕业项目。

---

## 二、工作空间结构

```
catkin_ws2/                            # Catkin 工作空间根目录
├── launch_robosar.sh                  # 顶层一键启动脚本
├── work/build_sequence.sh             # 按依赖顺序编译的构建脚本
├── src/robosar/                       # 所有 ROS 功能包所在目录
│   ├── README.md                      # 项目说明文档
│   ├── robosar_agent_bringup/         # 机器人硬件驱动与通信
│   ├── robosar_controller/            # 多机器人交通控制器
│   ├── robosar_navigation/            # 路径规划与任务执行
│   ├── robosar_task_generator/        # 任务（覆盖点/前沿点）生成
│   ├── robosar_task_allocator/        # 任务分配与任务指挥
│   ├── robosar_SLAM_toolbox/          # 多机器人 SLAM（建图与定位）
│   │   ├── slam_toolbox/              # SLAM 核心包
│   │   ├── slam_toolbox_msgs/         # SLAM 服务消息定义
│   │   └── slam_toolbox_rviz/         # SLAM RViz 插件
│   ├── robosar_messages/              # 自定义消息/服务/动作定义
│   ├── robosar_gui/                   # RViz GUI 插件
│   ├── robosar_sim_bringup/           # 仿真启动（Stage/Gazebo）
│   ├── robosar_gazebo_sim/            # Gazebo 仿真模型与世界
│   ├── robosar_stage_ros/             # Stage 仿真器（修改版）
│   ├── robosar_khepera_code/          # Khepera 机器人固件代码
│   ├── robosar_khepera_lib/           # Khepera 库
│   ├── robosar_khepera_perception/    # Khepera 感知（AprilTag）
│   └── robosar_tf/                    # TF 变换发布
├── build/                             # 编译产物
└── devel/                             # 开发环境（setup.bash 等）
```

---

## 三、各功能包详细分析

---

### 3.1 `robosar_messages` — 自定义消息定义包

提供整个系统使用的自定义 ROS 消息、服务和动作定义。

#### 消息文件（.msg）

| 文件 | 字段 | 用途 |
|------|------|------|
| `task_allocation.msg` | `string[] id`, `float32[] startx/starty/goalx/goaly`, `int64[] goal_type/goal_id` | 任务分配结果：每个机器人的起点、目标、任务类型 |
| `mission_command.msg` | `START=1, STOP=2, HOME=3`, `int32 data` | 系统级任务指令（启动/停止/返航） |
| `controller_status.msg` | `IDLE=1, BUSY=2, SUCCEEDED=3`, `int32 data`, `int32 goal_id` | 控制器反馈每个机器人的执行状态 |
| `agents_status.msg` | `string[] robot_id/ip_adress/status`, `int32[] battery_lvl/feedback_freq` | 机器人群体状态信息 |
| `PointArray.msg` | `geometry_msgs/Point[] points`, `float32[] infoGain` | 前沿点数组（含信息增益） |

#### 服务文件（.srv）

| 文件 | 请求/响应 | 用途 |
|------|----------|------|
| `robosar_controller.srv` | `bool stop, string[] agent_names, Path[] paths, int64[] goal_type/goal_id` → `bool success` | 向控制器下发路径或紧急停止 |
| `agent_status.srv` | 空请求 → `string[] agents_active` | 获取当前活跃机器人列表 |
| `task_graph_getter.srv` | 空请求 → `int32[] task_ids, Point[] points, int32[] task_types, float32[] info_gains` | 获取未完成的任务列表 |
| `task_graph_setter.srv` | `int32[] finished_task_ids` → 空响应 | 标记已完成的任务 |
| `taskgen_getwaypts.srv` | `OccupancyGrid map, float64 threshold, int64 iterations` → `int64 num_pts, int64[] waypoints` | 获取任务路点 |
| `rrt_connect.srv` | `Point point` → `bool free` | 检查 RRT 可达性 |
| `rrt_path_cost.srv` | `float32 robot_x/y, goal_x/y` → `float32 cost` | 计算 RRT 路径代价 |
| `sys_odom_reset.srv` | 空 → 空 | 重置所有机器人里程计 |

#### 动作文件（.action）

| 文件 | Goal → Result / Feedback | 用途 |
|------|--------------------------|------|
| `FrontierFilter.action` | `Point[] frontiers, OccupancyGrid, Pose[] pose_graph` → 空 | 前沿点过滤 |

（另有 `robosar_controller` 包中的动作定义，见 3.4 节）

---

### 3.2 `robosar_agent_bringup` — 机器人硬件驱动与通信

负责初始化硬件机器人、建立 UDP 通信、解析传感器数据并发布为 ROS 话题。

#### 关键文件

| 文件 | 作用 |
|------|------|
| `robosar_agent_bringup_node.cpp` | 主节点入口，创建 `ConfigParser` 对象后自旋 |
| `config_parser.hpp/.cpp` | 读取 JSON 配置文件，为每个机器人创建 `RobotAgent`；提供 `agent_status` 服务和里程计重置服务 |
| `robot_agent.hpp/.cpp` | 单个机器人代理类：管理 UDP 通信、接收传感器数据、发布速度命令、监控死亡计时器 |
| `async_udp_server.hpp` | 异步 UDP 服务器：接收 Protobuf 编码的传感器数据（使用 `robosar.pb.h`），解码后交给 `ROSFeedbackBridge` |
| `ros_feedback_bridge.hpp` | ROS 反馈桥接：将 Protobuf 数据拆包为 IMU、LaserScan、AprilTag、编码器里程计等 ROS 消息并发布 |
| `odom_node.hpp/.cpp` | 差分驱动里程计：根据编码器计数计算位姿，发布欧拉角和四元数两种里程计消息 |
| `robot_status.hpp/.cpp` | 机器人状态管理（ACTIVE/INACTIVE/DEAD） |

#### ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| **发布** | `~<agent>/feedback/IMU` | `sensor_msgs/Imu` | IMU 数据 |
| **发布** | `~<agent>/feedback/scan` | `sensor_msgs/LaserScan` | 激光雷达扫描 |
| **发布** | `~<agent>/feedback/apriltag` | `AprilTagDetectionArray` | AprilTag 检测 |
| **发布** | `odom_data_euler` / `odom_data_quat` | `nav_msgs/Odometry` | 里程计 |
| **发布** | `~status` | `std_msgs/Bool` | 机器人群状态变化通知 |
| **订阅** | `~<agent>/cmd_vel` | `geometry_msgs/Twist` | 接收速度命令转发给硬件 |
| **服务** | `~agent_status` | `agent_status.srv` | 返回活跃机器人列表 |
| **服务** | `~odom_reset` | `sys_odom_reset.srv` | 重置里程计 |
| **TF** | `<agent>/odom` → `<agent>/base_link` | — | 里程计变换 |

---

### 3.3 `robosar_SLAM_toolbox` — 多机器人 SLAM

基于开源 slam_toolbox 大幅修改，支持**多机器人同步建图与定位**。包含三个子包：

#### 3.3.1 `slam_toolbox` — SLAM 核心

##### 运行模式

| 模式 | 节点类 | 说明 |
|------|--------|------|
| 异步建图 | `AsynchronousSlamToolbox` | 接收激光扫描按时间阈值处理，构建位姿图 |
| 同步建图 | `SynchronousSlamToolbox` | 每次扫描都处理 |
| 定位模式 | `LocalizationSlamToolbox` | 加载已有地图，仅做定位 |
| 终身建图 | `LifelongSlamToolbox` | 持续建图并裁剪旧数据（实验性） |

##### 关键文件

| 文件 | 作用 |
|------|------|
| `slam_toolbox_common.hpp/.cpp` | 基类 `SlamToolbox`：管理多传感器激光订阅、TF 发布、位姿图序列化、地图更新；支持多个 `odom_frame`/`base_frame`/`laser_topic` 配置以实现多机器人 |
| `slam_toolbox_async.hpp/.cpp` | 异步建图模式实现 |
| `slam_toolbox_localization.hpp/.cpp` | 定位模式实现 |
| `slam_toolbox_sync.hpp/.cpp` | 同步建图模式实现 |
| `experimental/slam_toolbox_lifelong.hpp/.cpp` | 终身建图（实验性） |
| `laser_utils.cpp` | 激光数据处理工具，管理多传感器的 `LaserRangeFinder` 对象 |
| `slam_mapper.cpp` | 位姿图管理器：维护节点和扫描数据 |
| `loop_closure_assistant.hpp/.cpp` | 回环检测助手：可交互式添加回环约束 |
| `get_pose_helper.hpp` | TF 查询辅助：获取机器人在 odom 坐标系中的位姿 |
| `map_saver.cpp` | 地图保存工具 |
| `visualization_utils.hpp` | 可视化工具：发布位姿图的 MarkerArray |
| `toolbox_types.hpp` | 类型定义和数据结构 |
| `toolbox_msgs.hpp` | 消息辅助 |

##### 关键 ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| **订阅** | 多个 `laser_topics` | `sensor_msgs/LaserScan` | 各机器人的激光扫描 |
| **订阅** | 多个 `apriltag_topics` | `AprilTagDetectionArray` | AprilTag 检测（辅助定位） |
| **发布** | `/map` | `nav_msgs/OccupancyGrid` | 占据栅格地图 |
| **发布** | `karto_graph_visualization` | `MarkerArray` | 位姿图可视化（供 task_generator 使用） |
| **服务** | `dynamic_map` | `nav_msgs/GetMap` | 获取当前地图 |
| **服务** | `serialize_map` / `deserialize_map` | 自定义 | 位姿图序列化/反序列化 |
| **TF** | `map` → `<agent>/odom` | — | 全局定位变换（每个机器人） |

##### 配置文件 (`robosar_mapping.yaml`)

- 支持最多 5 个机器人（`agent_0` ~ `agent_4`）
- 求解器：Ceres Solver（SPARSE_NORMAL_CHOLESKY）
- 地图分辨率：0.05m
- 最大激光范围：5.0m
- 回环检测：启用，支持跨机器人回环（`loop_close_across_agents: true`）

##### Karto SDK (`lib/karto_sdk/`)

底层图优化库（`Karto.h` + `Mapper.cpp`），实现：
- 扫描匹配（scan matching）
- 位姿图构建
- 回环检测与图优化
- 被 RoboSAR 修改以支持多机器人

#### 3.3.2 `slam_toolbox_msgs` — SLAM 服务消息

定义 10 个服务：`SerializePoseGraph`, `DeserializePoseGraph`, `MergeMaps`, `SaveMap`, `LoopClosure`, `Clear`, `ClearQueue`, `Pause`, `ToggleInteractive`, `AddSubmap`。

---

### 3.4 `robosar_controller` — 多机器人交通控制器

集中式多机器人路径跟踪与防碰撞控制器，使用 **Pure Pursuit** 路径跟踪 + **RVO（Reciprocal Velocity Obstacles）** 避碰。

#### 关键文件

| 文件 | 作用 |
|------|------|
| `lazy_traffic_controller.hpp/.cpp` | **核心控制器** `LazyTrafficController`：管理机器人群体，5Hz 定时器驱动控制循环；为每个机器人更新位姿→计算偏好速度→RVO 避碰→发送速度 |
| `lazy_traffic_agent.hpp/.cpp` | **单机器人代理** `Agent`：Pure Pursuit 路径跟踪、RVO 调用、速度发布、搜索旋转行为（到达覆盖目标后原地旋转） |
| `lazy_traffic_rvo.hpp` | **RVO 引擎**：采样 1000 个候选速度，选择最小惩罚（考虑碰撞时间和偏好速度距离）的速度 |
| `Vector2.h` | 2D 向量数学库（来自 UNC RVO Library） |
| `traffic_controller_node.cpp` | 主节点入口 |
| `lg_action_server.hpp` | 遗留：单机器人 ActionServer 控制器 |
| `controller_action_server.hpp` | 遗留：另一种单机器人控制器 |

#### RVO 参数

- 采样数：1000 个随机速度候选
- 机器人半径：0.15m
- 安全系数：20.0
- 排斥半径：0.5m
- 最大邻居数：5

#### 控制流程（每 0.2s 一次）

1. **TF 查询**：`map` → `<agent>/base_link`，获取各机器人位姿
2. **Pure Pursuit**：前视距离 0.4m，计算偏好速度方向
3. **RVO 避碰**：考虑最近 5 个动态邻居 + 占据栅格中的静态障碍物
4. **排斥力**：0.5m 内加权排斥向量
5. **速度转换**：2D 速度向量 → `cmd_vel`（linear.x + angular.z）
6. **状态发布**：IDLE / BUSY / SUCCEEDED

#### ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| **订阅** | `~status` (agent_bringup) | `std_msgs/Bool` | 机器人群状态变化 |
| **订阅** | `/map` | `OccupancyGrid` | 占据栅格（静态障碍物避碰） |
| **发布** | `~<agent>/cmd_vel` | `geometry_msgs/Twist` | 各机器人速度命令 |
| **发布** | `~<agent>/status` | `controller_status` | 各机器人控制器状态 |
| **发布** | `~<agent>/vel_marker` | `Marker` | RViz 速度可视化箭头 |
| **服务** | `robosar_controller/lazy_traffic_controller` | `robosar_controller.srv` | 接收路径分配或紧急停止 |
| **服务客户端** | `~agent_status` | `agent_status.srv` | 获取活跃机器人列表 |
| **TF 查询** | `map` → `<agent>/base_link` | — | 获取机器人实时位姿 |

---

### 3.5 `robosar_navigation` — 路径规划与任务执行

提供代价地图、图搜索和多机器人 A* 规划，以及任务执行节点。

#### 关键文件

| 文件 | 作用 |
|------|------|
| `costmap_2d.hpp/.cpp` | **2D 代价地图**：订阅 `/map`，进行障碍物膨胀（指数衰减），发布膨胀后的代价地图 |
| `cost_inflator.hpp` | **代价膨胀引擎**：使用波前扩展（Dijkstra），从致命障碍物向外传播代价值 |
| `graph_2d_grid.hpp/.cpp` | **2D 栅格图**（空间维度）：8 连通，包含目标缓存实现跨机器人碰撞检测 |
| `graph_3d_grid.hpp/.cpp` | **3D 栅格图**（空间+时间）：9 连通（含原地等待），轨迹缓存实现协同规避 |
| `astar.hpp` | **A* 路径规划器**：加权 A*（启发式权重 10.0），发布规划路径 |
| `multi_astar.hpp` | **优先级多机器人 A\***：按优先级顺序为每个机器人规划，前序轨迹缓存供后续规避 |
| `mission_executive.hpp` / `robosar_mission_execute.cpp` | **任务执行节点**：接收 `task_allocation` 消息，运行 MultiAStar，将路径通过服务调用发送给控制器 |

#### 代价地图参数

- 膨胀半径：5 cells
- 内切半径：0.1m
- 代价缩放因子：5.0
- 碰撞阈值：0.4m（跨机器人目标碰撞检测）

#### ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| **订阅** | `/map` | `OccupancyGrid` | 原始占据栅格 |
| **订阅** | `task_allocation` | `task_allocation.msg` | 任务分配结果 |
| **订阅** | `/system_mission_command` | `mission_command.msg` | 紧急停止命令 |
| **发布** | `robosar_navigation/costmap` | `OccupancyGrid` | 膨胀后的代价地图 |
| **发布** | `{agent}/plan` | `nav_msgs/Path` | 各机器人规划路径 |
| **发布** | `{agent}/endpoints` | `MarkerArray` | 起终点可视化 |
| **服务客户端** | `robosar_controller/lazy_traffic_controller` | `robosar_controller.srv` | 向控制器发送规划路径 |

---

### 3.6 `robosar_task_generator` — 任务生成器

基于 SLAM 位姿图动态生成覆盖任务（coverage tasks）和前沿点（frontiers）。

#### 关键文件

| 文件 | 作用 |
|------|------|
| `task_graph.hpp/.cpp` | **任务图节点** `TaskGraph`：核心类。订阅 SLAM 位姿图，使用 RRT 在自由空间中扩展任务树，生成覆盖点；提供任务获取/设置服务 |
| `robosar_task_graph_node.cpp` | 主节点入口 |
| `task_generator.py` | Python 任务生成工具（离线使用）：最大圆算法生成路点 |
| `frontier_filter.py`（脚本） | 前沿点过滤器：从代价地图中检测前沿点（已知/未知空间边界），聚类并过滤 |
| `rrt_utils/functions.cpp` | RRT 扩展工具函数 |
| `pgm_reader.py` | PGM 地图文件读取工具 |
| `max_circle.py` | 最大内切圆算法 |

#### 任务图工作原理

1. 订阅 `/slam_toolbox/karto_graph_visualization`（位姿图 MarkerArray）
2. 对每个新增的位姿图节点，创建 `TaskVertex`（包含 RRT 树）
3. 周期性定时器（默认 0.05s）调用 `expandRRT()`：在地理围栏内采样随机点，扩展最近的 RRT 树
4. RRT 树的叶节点作为**覆盖任务点**
5. 提供 `task_graph_getter` 服务供任务分配器查询未完成任务
6. 提供 `task_graph_setter` 服务标记已完成任务

#### ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| **订阅** | `/robosar_navigation/costmap`（可配置） | `OccupancyGrid` | 代价地图 |
| **订阅** | `/slam_toolbox/karto_graph_visualization` | `MarkerArray` | SLAM 位姿图 |
| **发布** | `/task_graph/shapes` | `Marker` | RRT 树可视化 |
| **发布** | `/task_graph/coverage_area` | `MarkerArray` | 覆盖区域可视化 |
| **发布** | `/frontier_filter/filtered_frontiers` | `PointArray` | 过滤后的前沿点 |
| **服务** | `/robosar_task_generator/task_graph_getter` | `task_graph_getter.srv` | 获取任务列表 |
| **服务** | `/robosar_task_generator/task_graph_setter` | `task_graph_setter.srv` | 标记完成的任务 |
| **服务** | `/robosar_task_generator/rrt_connect` | `rrt_connect.srv` | 检查 RRT 可达性 |

---

### 3.7 `robosar_task_allocator` — 任务分配

负责将生成的任务分配给各机器人。支持多种分配策略。

#### 关键文件

| 文件 | 作用 |
|------|------|
| `mission_commander.py` | **任务指挥官**：监听 `/system_mission_command`，根据命令启动/停止任务分配器；支持 HOME 返航功能 |
| `frontier_ta.py` | **前沿任务分配器** `FrontierAssignmentCommander`：基于贪心策略分配前沿+覆盖任务 |
| `high_ta.py` | **HIGH 任务分配器** `HIGHAssignmentCommander`：考虑探索/覆盖权重平衡，随着已探索面积增加逐渐转向覆盖任务 |
| `greedy_ta.py` | 贪心分配策略 |
| `mtsp_ta.py` | 多旅行商问题（mTSP）策略 |
| `task_commander.py` | 任务指挥基类 |
| `task_allocator/TA.py` | 任务分配算法实现（贪心、HIGH-OA 等） |
| `task_allocator/CostCalculator.py` | 代价计算器：A* 路径代价 + 障碍物代价 |
| `task_allocator/Environment.py` | 环境模型：管理前沿任务和覆盖任务 |
| `task_transmitter/task_listener_robosar_control.py` | 任务监听器：订阅控制器状态，跟踪各机器人任务完成情况 |

#### 分配策略对比

| 策略 | 说明 |
|------|------|
| `frontier` | 贪心分配：选择对每个机器人代价最低的前沿+覆盖任务 |
| `high` | HIGH-OA 算法：动态权衡探索（前沿）vs 覆盖，权重随已探索面积变化 |
| `greedy` | 简单贪心 |
| `mtsp` | 多旅行商问题求解 |

#### HIGH 分配器核心逻辑

```
探索权重 = min(1.1 - 已覆盖面积/总面积, 1.0)
覆盖权重 = 1 - 探索权重
```

初期探索权重高（优先探索未知区域），随着地图完善逐步转向覆盖任务。

#### ROS 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|------|----------|------|------|
| **订阅** | `/system_mission_command` | `mission_command` | 启动/停止/返航命令 |
| **订阅** | `/robosar_agent_bringup_node/status` | `Bool` | 机器人群状态 |
| **订阅** | `/frontier_filter/filtered_frontiers` | `PointArray` | 前沿点 |
| **发布** | `task_allocation` | `task_allocation.msg` | 任务分配结果 |
| **发布** | `/percent_completed` | `Float32` | 探索完成百分比 |
| **发布** | `/information_map` | `OccupancyGrid` | 信息价值地图 |
| **服务客户端** | `/robosar_agent_bringup_node/agent_status` | `agent_status.srv` | 获取活跃机器人 |
| **服务客户端** | `/robosar_task_generator/task_graph_getter` | `task_graph_getter.srv` | 获取任务 |
| **服务客户端** | `/robosar_task_generator/task_graph_setter` | `task_graph_setter.srv` | 标记已完成任务 |
| **TF 查询** | `map` → `<agent>/base_link` | — | 获取机器人位置 |

---

### 3.8 `robosar_gui` — 图形界面

#### RViz 插件

| 文件 | 作用 |
|------|------|
| `add_agent.cpp` / `add_agent.h` | RViz 工具插件 `AddAgentTool`：在 RViz 中点击添加新机器人，发布 `PoseStamped` 到 `agent_addition` 话题 |
| `robosar_gui.py`（脚本） | 独立 Python GUI：提供任务指令发布（START/STOP/HOME） |

#### ROS 接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| **发布** | `agent_addition` | `geometry_msgs/PoseStamped` | 添加新机器人的位姿 |
| **发布** | `/system_mission_command` | `mission_command` | GUI 发出的任务指令 |

---

### 3.9 `robosar_sim_bringup` — 仿真启动

#### Launch 文件

| 文件 | 作用 |
|------|------|
| `robosar_sim_tepper.launch` | **主仿真启动**：启动 Gazebo + 地图服务器 + AprilTag 仿真 + 多机器人生成 + 静态 TF（map→agent_N/odom）+ RViz |
| `robosar_sim.launch` | 通用仿真启动（Stage 版） |
| `robosar_sim_move_base.launch` | 使用 move_base 的仿真 |

#### 关键配置

- 使用 Gazebo 仿真时，通过 `spawn_robots.launch` 生成多个 Khepera4 机器人
- 为每个机器人发布 `map` → `agent_N/odom` 的静态 TF
- 地图服务器发布仿真地图到 `/sim_map`

---

### 3.10 `robosar_gazebo_sim` — Gazebo 仿真模型

| 文件 | 作用 |
|------|------|
| `urdf/khepera4_lrf.xacro` | Khepera4 机器人 URDF 模型（带激光雷达） |
| `world/tepper.world` | Tepper 建筑世界文件 |
| `launch/spawn_robot.launch` | 生成单个机器人 |
| `launch/spawn_robots.launch` | 生成多个机器人（不同初始位姿） |
| `launch/run_simulation.launch` | 启动 Gazebo 仿真 |

---

### 3.11 `robosar_stage_ros` — Stage 仿真器

修改版的 ROS Stage 仿真器，增加了位姿输出和碰撞检测功能。用于轻量级 2D 多机器人仿真。

---

### 3.12 `robosar_khepera_code` / `robosar_khepera_lib` / `robosar_khepera_perception`

硬件相关包，用于 Khepera IV 机器人：
- **khepera_code**：机器人端固件（Protobuf + nanopb 通信）
- **khepera_lib**：Khepera 驱动库
- **khepera_perception**：AprilTag 检测

---

## 四、话题/数据传输架构

### 4.1 核心话题流

```
                            ┌─────────────────┐
                            │  robosar_gui     │
                            │ (Python GUI)     │
                            └────────┬────────┘
                                     │ /system_mission_command
                                     ▼
┌──────────────────┐     ┌──────────────────────┐
│ agent_bringup    │     │  mission_commander   │
│ (硬件/仿真驱动)   │     │  (任务指挥官)         │
└──┬───┬───┬──────┘     └──────────┬───────────┘
   │   │   │                       │ 动态启动 task_allocator
   │   │   │                       ▼
   │   │   │            ┌──────────────────────┐
   │   │   │            │  task_allocator      │ ◄── /frontier_filter/filtered_frontiers
   │   │   │            │  (HIGH/Frontier TA)  │ ◄── task_graph_getter 服务
   │   │   │            └──────────┬───────────┘
   │   │   │                       │ task_allocation 话题
   │   │   │                       ▼
   │   │   │            ┌──────────────────────┐
   │   │   │            │  mission_executive   │
   │   │   │            │  (Multi A* 路径规划)  │
   │   │   │            └──────────┬───────────┘
   │   │   │                       │ robosar_controller 服务
   │   │   │                       ▼
   │   │   │            ┌──────────────────────┐
   │   │   │            │  traffic_controller  │
   │   │   │            │  (RVO 交通控制)       │
   │   │   │            └──────────┬───────────┘
   │   │   │                       │ <agent>/cmd_vel
   │   │   │                       ▼
   │   │   │            ┌──────────────────────┐
   │   │   └──────────► │  机器人（仿真/真实）   │
   │   │    cmd_vel      └──────────────────────┘
   │   │
   │   │ /feedback/scan     ┌──────────────────┐
   │   └──────────────────► │  slam_toolbox    │
   │                        │  (多机器人SLAM)   │
   │                        └──┬───────────────┘
   │                           │ /map  +  karto_graph_visualization
   │                           ▼
   │   /map              ┌──────────────────────┐
   │   ──────────────►   │  task_generator     │
   │                     │  (RRT任务生成)       │
   │                     └──────────────────────┘
   │                           │
   │                           │ task_graph_getter 服务 + filtered_frontiers
   │                           ▼
   │                     ┌──────────────────────┐
   └────────────────►    │  task_allocator      │
     agent_status 服务    └──────────────────────┘
```

### 4.2 关键话题一览

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/map` | `OccupancyGrid` | slam_toolbox | navigation, controller | 全局占据栅格地图 |
| `/slam_toolbox/karto_graph_visualization` | `MarkerArray` | slam_toolbox | task_generator | SLAM 位姿图 |
| `task_allocation` | `task_allocation.msg` | task_allocator | mission_executive | 任务分配结果 |
| `/system_mission_command` | `mission_command.msg` | GUI | mission_commander, mission_executive | 系统级指令 |
| `robosar_navigation/costmap` | `OccupancyGrid` | navigation | task_generator | 膨胀后代价地图 |
| `/frontier_filter/filtered_frontiers` | `PointArray` | frontier_filter | task_allocator | 过滤后前沿点 |
| `<agent>/cmd_vel` | `Twist` | controller | 机器人 | 速度命令 |
| `<agent>/feedback/scan` | `LaserScan` | agent_bringup | slam_toolbox | 激光扫描数据 |
| `<agent>/feedback/apriltag` | `AprilTagDetectionArray` | agent_bringup | slam_toolbox | AprilTag 检测 |
| `~<agent>/status` (controller) | `controller_status` | controller | task_allocator (listener) | 控制器执行状态 |
| `~status` (agent_bringup) | `Bool` | agent_bringup | controller, task_allocator | 机器人群体状态变化 |

### 4.3 关键服务一览

| 服务 | 调用方 | 提供方 | 说明 |
|------|--------|--------|------|
| `/robosar_agent_bringup_node/agent_status` | controller, task_allocator, mission_commander | agent_bringup | 获取活跃机器人列表 |
| `robosar_controller/lazy_traffic_controller` | mission_executive | controller | 下发路径或紧急停止 |
| `/robosar_task_generator/task_graph_getter` | task_allocator | task_generator | 获取未完成覆盖任务 |
| `/robosar_task_generator/task_graph_setter` | task_allocator | task_generator | 标记已完成任务 |
| `dynamic_map` | — | slam_toolbox | 获取当前地图 |

### 4.4 TF 变换树

```
map
├── agent_0/odom
│   └── agent_0/base_link
├── agent_1/odom
│   └── agent_1/base_link
├── agent_2/odom
│   └── agent_2/base_link
├── agent_3/odom
│   └── agent_3/base_link
└── agent_4/odom
    └── agent_4/base_link
```

- `map` → `agent_N/odom`：由 **slam_toolbox** 发布（建图/定位模式），或由仿真中的**静态 TF 发布器**发布
- `agent_N/odom` → `agent_N/base_link`：由 **agent_bringup** 的里程计发布（真实硬件），或由 **Gazebo 插件**发布（仿真模式）

---

## 五、系统运行流程

### 5.1 启动顺序

系统通过 `launch_robosar.sh` 按以下顺序启动 8 个组件：

```
1. roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch
   → 启动机器人硬件/仿真驱动，建立通信

2. roslaunch robosar_sim_bringup robosar_sim_tepper.launch
   → 启动 Gazebo 仿真器、地图服务器、生成机器人、发布静态 TF

3. roslaunch slam_toolbox robosar_mapping_sim.launch
   → 启动多机器人 SLAM（异步建图模式）

4. roslaunch robosar_task_generator task_generator.launch
   → 启动任务图节点（RRT 覆盖点生成）和前沿点过滤器

5. roslaunch robosar_navigation mission_executive.launch
   → 启动任务执行节点（接收任务分配，运行多机器人 A*）

6. rosrun robosar_controller robosar_traffic_controller_node
   → 启动交通控制器（Pure Pursuit + RVO）

7. rosrun robosar_task_allocator mission_commander.py
   → 启动任务指挥官（等待 GUI 命令来启动分配器）

8. python3 robosar_gui.py
   → 启动 GUI
```

### 5.2 任务执行完整流程

```
用户在 GUI 点击 "Start Mission"
        │
        ▼
GUI 发布 mission_command(START=1) → /system_mission_command
        │
        ▼
MissionCommander 收到 START 命令
        │
        ▼
动态启动任务分配器（如 HIGHAssignmentCommander）
        │
        ├─► 调用 agent_status 服务获取活跃机器人列表
        ├─► TF 查询获取各机器人当前位置
        ├─► 等待 /frontier_filter/filtered_frontiers（前沿点）
        ├─► 调用 task_graph_getter 服务获取覆盖任务
        │
        ▼
HIGH 算法计算任务分配
        │  考虑：
        │  - 探索/覆盖权重（随已探索面积动态变化）
        │  - A* 路径代价
        │  - 信息增益
        │  - 障碍物代价
        │
        ▼
发布 task_allocation 消息（每个机器人的起点+目标+任务类型）
        │
        ▼
MissionExecutive 收到 task_allocation
        │
        ▼
运行 MultiAStar（优先级多机器人 A*）
        │  - 在膨胀代价地图上规划
        │  - 按优先级顺序逐个规划
        │  - 前序轨迹缓存避免碰撞
        │
        ▼
调用 robosar_controller 服务，发送规划路径
        │
        ▼
LazyTrafficController 接收路径
        │
        ▼
5Hz 控制循环开始：
        │
        ├─ 1. TF 查询更新各机器人位姿
        ├─ 2. Pure Pursuit 计算偏好速度
        ├─ 3. RVO 采样 1000 个候选速度，选最优
        ├─ 4. 加权排斥力
        ├─ 5. 转换为 cmd_vel 发布
        └─ 6. 发布 controller_status
                │
                ▼
        机器人执行运动
                │
                ▼
        到达目标时发布 SUCCEEDED
                │
                ▼
        TaskAllocator 检测到 SUCCEEDED
                │
                ▼
        标记任务完成 → 调用 task_graph_setter
                │
                ▼
        重新分配下一个任务（循环）
```

### 5.3 探索与覆盖的渐进策略

系统采用两阶段渐进策略：

1. **早期（探索为主）**：地图未知区域多，前沿点丰富，探索权重接近 1.0。机器人优先前往前沿点探索新区域。

2. **后期（覆盖为主）**：地图逐渐完善，前沿点减少，覆盖权重增大。机器人转向 RRT 生成的覆盖点，进行细致搜索（到达后还会原地旋转 8 次供摄像头捕获 AprilTag）。

---

## 六、编译依赖顺序

由 `work/build_sequence.sh` 定义：

```
1. robosar_agent_bringup      ← 基础：机器人驱动
2. stage_ros_add_pose_and_crash ← 仿真器（修改版 Stage）
3. slam_toolbox                ← SLAM
4. robosar_controller          ← 交通控制
5. robosar_gui                 ← GUI 插件
6. robosar_navigation          ← 导航/规划
7. robosar_sim_bringup         ← 仿真启动
8. robosar_task_allocator      ← 任务分配
9. robosar_task_generator      ← 任务生成
10. robosar_tf                 ← TF 发布
```

`robosar_messages` 作为底层消息包被大多数包依赖，需要首先编译。

---

## 七、总结

RoboSAR 是一个功能完备的多机器人自主搜索救援系统，其核心技术栈包括：

| 模块 | 技术 |
|------|------|
| SLAM | 多机器人 Karto SLAM + Ceres 图优化 + 跨机器人回环检测 |
| 任务生成 | RRT 自由空间探索 + 前沿点检测 + 信息增益评估 |
| 任务分配 | HIGH-OA 算法（探索/覆盖权重动态平衡）+ 贪心/mTSP 策略 |
| 路径规划 | 优先级多机器人 A*（带轨迹缓存的协同规避） |
| 运动控制 | Pure Pursuit 路径跟踪 + RVO 实时避碰 + 排斥力 |
| 通信 | UDP + Protobuf（硬件）/ Gazebo 插件（仿真） |
| 硬件平台 | Khepera IV（差分驱动 + 激光雷达 + AprilTag 相机） |

系统设计遵循模块化原则，各功能包通过 ROS 话题、服务和 TF 变换松耦合连接，支持在仿真和真实硬件之间无缝切换。
