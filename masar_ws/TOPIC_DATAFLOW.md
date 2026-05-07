# RoboSAR 话题数据流详细说明

> 本文档描述 `launch_robosar.sh` 启动系统后，所有 ROS 话题、服务、TF 的发布/订阅关系及数据流走向。
> 系统包含 5 个机器人 agent（`agent_0` ~ `agent_4`），以下用 `{agent_id}` 表示。

---

## 一、系统启动顺序

| 序号 | 启动命令 | 节点 |
|:---:|------|------|
| 1 | `roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch` | agent_bringup (多 agent) |
| 2 | `roslaunch robosar_sim_bringup robosar_sim_tepper.launch` | Gazebo + 机器人模型 + AprilTagSim |
| 3 | `roslaunch slam_toolbox robosar_mapping_sim.launch` | slam_toolbox + map_filter |
| 4 | `roslaunch robosar_task_generator task_generator.launch` | task_graph_node + frontier_filter |
| 5 | `roslaunch robosar_navigation mission_executive.launch` | mission_executive |
| 6 | `rosrun robosar_controller robosar_traffic_controller_node` | lazy_traffic_controller |
| 7 | `rosrun robosar_task_allocator mission_commander.py -s=true -t='high'` | mission_commander (high_ta) |
| 8 | `python3 robosar_gui.py` | GUI |

---

## 二、全局数据流总览

```
┌────────────────────────────────────────────────────────────────────────┐
│                          Gazebo 仿真器                                 │
│  (发布: /cmd_vel → 驱动机器人,  产生: LaserScan, Camera, Odom TF)      │
└───────┬──────────────────────────────┬─────────────────────────────────┘
        │ Gazebo 插件直接发布           │ apriltag_ros 处理相机图像
        ▼                              ▼
┌───────────────────┐     ┌──────────────────────────────┐
│  agent_bringup    │     │  AprilTagSim (仿真模式)       │
│  ┌──────────────┐ │     │  发布: feedback/apriltag       │
│  │feedback/scan │─┼──┐  │  发布: victim_markers          │
│  │odom TF       │ │  │  └──────────────┬───────────────┘
│  │feedback/IMU  │ │  │                 │
│  └──────────────┘ │  │                 │
└───────────────────┘  │                 │
                       ▼                 ▼
              ┌─────────────────────────────┐
              │       SLAM Toolbox          │
              │  订阅: feedback/scan (×5)   │
              │  订阅: feedback/apriltag(×5)│
              │  发布: /slam_toolbox/map     │
              │  发布: TF map→{agent}/odom  │
              │  发布: victim_markers        │
              │  发布: karto_graph_vis       │
              └──────────┬──────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │    map_filter        │
              │  订阅:/slam_toolbox/ │
              │       map            │
              │  发布: /map          │
              └──────────┬───────────┘
                         │
            ┌────────────┼────────────────┐
            ▼            ▼                ▼
    ┌──────────┐  ┌────────────┐  ┌──────────────┐
    │Navigation│  │Task        │  │Controller    │
    │costmap_2d│  │Generator   │  │(LT_CTRL)     │
    │          │  │            │  │              │
    │订阅:/map │  │订阅:/map或 │  │订阅:/map     │
    │发布:cost │  │  costmap   │  │              │
    │    map   │  │发布:可视化  │  │              │
    └──────────┘  └─────┬──────┘  └──────┬───────┘
                        │                │
                        ▼                │
              ┌──────────────────┐       │
              │  Task Allocator  │       │
              │ (mission_cmdr)   │       │
              │  调用: task_graph │       │
              │       getter     │       │
              │  发布:           │       │
              │  task_allocation │       │
              └────────┬─────────┘       │
                       │                 │
                       ▼                 │
              ┌──────────────────┐       │
              │Mission Executive │       │
              │  订阅:           │       │
              │  task_allocation │       │
              │  调用: controller│───────┘
              │       service    │
              └──────────────────┘
                                         │
                                         ▼
                               ┌──────────────────┐
                               │  Controller 发布  │
                               │  cmd_vel → Gazebo │
                               └──────────────────┘
```

---

## 三、话题详细列表

### 3.1 传感器数据话题（Gazebo → agent_bringup → 算法节点）

#### `/robosar_agent_bringup_node/{agent_id}/feedback/scan`

| 属性 | 值 |
|------|-----|
| **消息类型** | `sensor_msgs/LaserScan` |
| **发布者** | `agent_bringup` (ros_feedback_bridge) |
| **订阅者** | SLAM Toolbox, Navigation (costmap) |
| **频率** | 10 Hz |
| **帧 ID** | `{agent_id}/base_link` |

**发布代码**:

```cpp
// robosar_agent_bringup/include/ros_feedback_bridge.hpp:36
lrf_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("feedback/scan", 1, true);
```

**SLAM 订阅代码**:

```cpp
// slam_toolbox/src/slam_toolbox_common.cpp:217
scan_filter_subs_.push_back(
  std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(
    node, laser_topics_[idx], 5));
scan_filters_.push_back(
  std::make_unique<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
    *scan_filter_subs_.back(), *tf_, odom_frames_[idx], 5, node));
scan_filters_.back()->registerCallback(
  boost::bind(&SlamToolbox::laserCallback, this, _1));
```

---

#### `/robosar_agent_bringup_node/{agent_id}/feedback/apriltag`

| 属性 | 值 |
|------|-----|
| **消息类型** | `apriltag_ros/AprilTagDetectionArray` |
| **发布者** | `agent_bringup` (ros_feedback_bridge) 或 `AprilTagSim.py`（仿真模式） |
| **订阅者** | SLAM Toolbox, GUI |

**发布代码（agent_bringup）**:

```cpp
// robosar_agent_bringup/include/ros_feedback_bridge.hpp:37
apriltag_publisher_ = nh_.advertise<apriltag_ros::AprilTagDetectionArray>(
  "feedback/apriltag", 1, true);
```

**发布代码（AprilTagSim 仿真）**:

```python
# robosar_sim_bringup/scripts/AprilTagSim.py:100-101
self.apriltag_feedback_pub[a] = rospy.Publisher(
  "/robosar_agent_bringup_node/" + str(a) + "/feedback/apriltag",
  AprilTagDetectionArray, queue_size=10)
```

**SLAM 订阅代码**:

```cpp
// slam_toolbox/src/slam_toolbox_common.cpp:222
apriltag_subs_.push_back(
  std::make_unique<message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray>>(
    node, apriltag_topics_[idx], 5));
apriltag_subs_.back()->registerCallback(
  boost::bind(&SlamToolbox::apriltagCallback, this, _1));
```

---

#### `/robosar_agent_bringup_node/{agent_id}/feedback/IMU`

| 属性 | 值 |
|------|-----|
| **消息类型** | `sensor_msgs/Imu` |
| **发布者** | `agent_bringup` (ros_feedback_bridge) |
| **订阅者** | 当前无节点主动订阅 |

```cpp
// robosar_agent_bringup/include/ros_feedback_bridge.hpp:35
imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("feedback/IMU", 1, true);
```

---

#### `/robosar_agent_bringup_node/{agent_id}/odom_data_euler`

| 属性 | 值 |
|------|-----|
| **消息类型** | `nav_msgs/Odometry` |
| **发布者** | `agent_bringup` (ros_feedback_bridge) |
| **订阅者** | `odomTF` (robosar_tf，用于发布 TF) |

```cpp
// robosar_agent_bringup/include/ros_feedback_bridge.hpp:30
odom_data_pub_euler(nh_.advertise<nav_msgs::Odometry>("odom_data_euler", 100))
```

---

#### `/robosar_agent_bringup_node/{agent_id}/odom_data_quat`

| 属性 | 值 |
|------|-----|
| **消息类型** | `nav_msgs/Odometry` |
| **发布者** | `agent_bringup` (ros_feedback_bridge) |
| **订阅者** | 备用（与 euler 版本等价） |

```cpp
// robosar_agent_bringup/include/ros_feedback_bridge.hpp:30
odom_data_pub_quat(nh_.advertise<nav_msgs::Odometry>("odom_data_quat", 100))
```

---

### 3.2 地图话题（SLAM → 全系统）

#### `/slam_toolbox/map`

| 属性 | 值 |
|------|-----|
| **消息类型** | `nav_msgs/OccupancyGrid` |
| **发布者** | SLAM Toolbox |
| **订阅者** | `map_filter` |

```cpp
// slam_toolbox/src/slam_toolbox_common.cpp:207
sst_ = node.advertise<nav_msgs::OccupancyGrid>(map_name_, 1, true);
```

---

#### `/map`

| 属性 | 值 |
|------|-----|
| **消息类型** | `nav_msgs/OccupancyGrid` |
| **发布者** | `map_filter`（对 `/slam_toolbox/map` 过滤后重新发布） |
| **订阅者** | Navigation (costmap_2d), Controller, Task Generator |

**发布代码**:

```python
# slam_toolbox/scripts/map_filter.py:8
pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
```

**map_filter 订阅代码**:

```python
# slam_toolbox/scripts/map_filter.py:33
rospy.Subscriber("/slam_toolbox/map", OccupancyGrid, filter_map)
```

**Navigation 订阅**:

```cpp
// robosar_navigation/src/costmap_2d.cpp:22
map_sub_ = nh_.subscribe("map", 1, &Costmap2D::incomingMap, this);
```

**Controller 订阅**:

```cpp
// robosar_controller/src/lazy_traffic_controller.cpp:13
occupancy_grid_subscriber_ = nh_.subscribe("/map", 1,
  &LazyTrafficController::occupancyGridCallback, this);
```

**Task Generator 订阅**:

```cpp
// robosar_task_generator/src/robosar_task_generator/task_graph.cpp:19
map_sub_ = nh_.subscribe(map_topic_, 100, &TaskGraph::mapCallBack, this);
```

---

#### `/robosar_navigation/costmap`

| 属性 | 值 |
|------|-----|
| **消息类型** | `nav_msgs/OccupancyGrid` |
| **发布者** | Navigation (costmap_2d) |
| **订阅者** | Task Generator（可选，通过参数 `map_topic` 配置） |

```cpp
// robosar_navigation/src/costmap_2d.cpp:19
costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("robosar_navigation/costmap", 1);
```

---

### 3.3 控制指令话题（Controller → Gazebo）

#### `/robosar_agent_bringup_node/{agent_id}/cmd_vel`

| 属性 | 值 |
|------|-----|
| **消息类型** | `geometry_msgs/Twist` |
| **发布者** | Controller (lazy_traffic_agent) |
| **订阅者** | `agent_bringup` → 转发到 Gazebo 差速驱动插件 |

**发布代码**:

```cpp
// robosar_controller/include/lazy_traffic_agent.hpp:49
pub_vel_ = nh_.advertise<geometry_msgs::Twist>(
  "/robosar_agent_bringup_node/" + name + "/cmd_vel", 1);
```

**agent_bringup 订阅代码**:

```cpp
// robosar_agent_bringup/src/robot_agent.cpp:69
control_subscriber_ = nh_.subscribe("cmd_vel", 1,
  &RobotAgent::velocityCallback, this);
```

---

### 3.4 任务分配话题（Allocator ↔ Navigation ↔ Controller）

#### `task_allocation`

| 属性 | 值 |
|------|-----|
| **消息类型** | `robosar_messages/task_allocation` |
| **发布者** | Task Allocator (mission_commander.py / high_ta.py) |
| **订阅者** | Mission Executive (robosar_navigation) |

**发布代码**:

```python
# robosar_task_allocator/src/mission_commander.py:172
task_pub = rospy.Publisher("task_allocation", task_allocation, queue_size=10)
```

**订阅代码**:

```cpp
// robosar_navigation/include/robosar_navigation/mission_executive.hpp:29
task_allocation_subscriber = nh_.subscribe("task_allocation", 10,
  &MissionExecutive::taskAllocationCallback, this);
```

---

#### `/lazy_traffic_controller/{agent_id}/status`

| 属性 | 值 |
|------|-----|
| **消息类型** | `robosar_messages/controller_status` |
| **发布者** | Controller (lazy_traffic_agent) |
| **订阅者** | Task Allocator (task_listener) |

**发布代码**:

```cpp
// robosar_controller/include/lazy_traffic_agent.hpp:51
pub_status_ = nh_.advertise<robosar_messages::controller_status>(
  "/lazy_traffic_controller/" + name + "/status", 1);
```

**订阅代码**:

```python
# robosar_task_allocator/src/task_transmitter/task_listener_robosar_control.py:74-78
rospy.Subscriber("/lazy_traffic_controller/" + name + "/status",
  controller_status, self.callback)
```

---

### 3.5 系统状态话题

#### `/robosar_agent_bringup_node/status`

| 属性 | 值 |
|------|-----|
| **消息类型** | `std_msgs/Bool` |
| **发布者** | agent_bringup (robot_status) |
| **订阅者** | Controller, Task Allocator (frontier_ta, mtsp_ta) |

```cpp
// robosar_agent_bringup/src/robot_status.cpp:7
status_pub = nh.advertise<std_msgs::Bool>("status", 1000);
```

---

#### `/robosar_agent_bringup_node/all_agent_status`

| 属性 | 值 |
|------|-----|
| **消息类型** | `robosar_messages/agents_status` |
| **发布者** | agent_bringup (config_parser) |
| **订阅者** | GUI |

```cpp
// robosar_agent_bringup/src/config_parser.cpp:47
agent_status_pub_ = nh.advertise<robosar_messages::agents_status>("all_agent_status", 1);
```

---

### 3.6 GUI 与用户交互话题

#### `/system_mission_command`

| 属性 | 值 |
|------|-----|
| **消息类型** | `robosar_messages/mission_command` |
| **发布者** | GUI (robosar_gui.py) |
| **订阅者** | Mission Executive |

```python
# robosar_gui/src/robosar_gui.py:81-83
self.tc_pub = rospy.Publisher("/system_mission_command", mission_command, queue_size=5)
```

```cpp
// robosar_navigation/include/robosar_navigation/mission_executive.hpp:32
gui_subscriber_ = nh_.subscribe("/system_mission_command", 1,
  &MissionExecutive::eStopCallback, this);
```

---

#### `/task_allocation_image`

| 属性 | 值 |
|------|-----|
| **消息类型** | `sensor_msgs/Image` |
| **发布者** | Task Allocator (frontier_ta.py) |
| **订阅者** | GUI |

```python
# robosar_task_allocator/src/frontier_ta.py:44
self.image_pub = rospy.Publisher("task_allocation_image", Image, queue_size=10)
```

---

#### `/percent_completed`

| 属性 | 值 |
|------|-----|
| **消息类型** | `std_msgs/Float32` |
| **发布者** | Task Allocator (high_ta.py) |
| **订阅者** | GUI |

```python
# robosar_task_allocator/src/high_ta.py:62
self.area_explored_pub = rospy.Publisher("/percent_completed", Float32, queue_size=1)
```

---

### 3.7 可视化话题

| 话题名 | 消息类型 | 发布者 | 用途 |
|--------|---------|--------|------|
| `/slam_toolbox/victim_markers` | `visualization_msgs/Marker` | SLAM Toolbox / AprilTagSim | AprilTag 检测位置标记 |
| `/slam_toolbox/karto_graph_visualization` | `visualization_msgs/MarkerArray` | SLAM Toolbox (loop_closure_assistant) | SLAM 位姿图可视化 |
| `/slam_toolbox/karto_scan_visualization` | `sensor_msgs/LaserScan` | SLAM Toolbox (loop_closure_assistant) | 扫描数据可视化 |
| `/task_graph/coverage_area` | `visualization_msgs/MarkerArray` | Task Generator | RRT 覆盖区域 |
| `/task_graph/shapes` | `visualization_msgs/Marker` | Task Generator | 任务图形状 |
| `/frontier_filter/frontier_centroids` | `visualization_msgs/Marker` | Frontier Filter | 前沿点质心 |
| `/lazy_traffic_controller/{agent_id}/vel_marker` | `visualization_msgs/Marker` | Controller | 速度方向可视化 |
| `/information_map` | `nav_msgs/OccupancyGrid` | Task Allocator (high_ta) | 信息增益地图 |

---

## 四、服务接口列表

### 4.1 agent_bringup 提供的服务

| 服务名 | 服务类型 | 提供者 | 调用者 | 文件 |
|--------|---------|--------|--------|------|
| `/robosar_agent_bringup_node/agent_status` | `robosar_messages/agent_status` | agent_bringup (config_parser) | SLAM, Controller, Task Allocator, AprilTagSim, GUI | `config_parser.cpp:45` |
| `/robosar_agent_bringup_node/sys_odom_reset` | `robosar_messages/sys_odom_reset` | agent_bringup (config_parser) | — | `config_parser.cpp:46` |

### 4.2 SLAM Toolbox 提供的服务

| 服务名 | 服务类型 | 文件 |
|--------|---------|------|
| `/slam_toolbox/dynamic_map` | `nav_msgs/GetMap` | `slam_toolbox_common.cpp:210` |
| `/slam_toolbox/pause_new_measurements` | `slam_toolbox_msgs/Pause` | `slam_toolbox_common.cpp:211` |
| `/slam_toolbox/serialize_map` | `slam_toolbox_msgs/SerializePoseGraph` | `slam_toolbox_common.cpp:212` |
| `/slam_toolbox/deserialize_map` | `slam_toolbox_msgs/DeserializePoseGraph` | `slam_toolbox_common.cpp:213` |
| `/slam_toolbox/clear_queue` | `slam_toolbox_msgs/ClearQueue` | `slam_toolbox_sync.cpp:31` |
| `/slam_toolbox/save_map` | — | `map_saver.cpp:29` |

### 4.3 Task Generator 提供的服务

| 服务名 | 服务类型 | 调用者 | 文件 |
|--------|---------|--------|------|
| `/robosar_task_generator/task_graph_getter` | `robosar_messages/task_graph_getter` | Task Allocator | `task_graph.cpp:35` |
| `/robosar_task_generator/task_graph_setter` | `robosar_messages/task_graph_setter` | Task Allocator | `task_graph.cpp:36` |
| `/robosar_task_generator/rrt_connect` | `robosar_messages/rrt_connect` | Frontier Filter | `task_graph.cpp:37` |

### 4.4 Controller 提供的服务

| 服务名 | 服务类型 | 调用者 | 文件 |
|--------|---------|--------|------|
| `lazy_traffic_controller` | `robosar_messages/robosar_controller` | Mission Executive | `lazy_traffic_controller.cpp:27` |

---

## 五、TF 坐标变换

| 变换 | 发布者 | 文件 |
|------|--------|------|
| `map` → `{agent_id}/odom` | SLAM Toolbox (`publishTransformLoop`) | `slam_toolbox_common.cpp:248-256` |
| `{agent_id}/odom` → `{agent_id}/base_link` | odomTF (robosar_tf，由 agent_bringup 实例化) | `robot_agent.cpp:18` |
| `{agent_id}/base_link` → `{agent_id}/base_laser_link` | static_transform_publisher (launch 文件) | `robosar_agent_bringup_node.launch` |
| `{agent_id}/base_link` → `{agent_id}/camera_link` | static_transform_publisher (launch 文件) | `robosar_agent_bringup_node.launch` |

**TF 树结构**:

```
map
 ├── agent_0/odom
 │    └── agent_0/base_link
 │         ├── agent_0/base_laser_link
 │         └── agent_0/camera_link
 ├── agent_1/odom
 │    └── agent_1/base_link
 │         ├── ...
 ├── agent_2/odom
 ├── agent_3/odom
 └── agent_4/odom
```

---

## 六、完整数据流路径

### 路径 1：传感器 → 建图

```
Gazebo LRF插件 → agent_bringup → /feedback/scan → SLAM Toolbox
                                                      ↓
                                              /slam_toolbox/map
                                                      ↓
                                                 map_filter
                                                      ↓
                                                    /map
                                                      ↓
                                   ┌──────────────────┼──────────────────┐
                                   ▼                  ▼                  ▼
                             Navigation          Task Generator      Controller
                             (costmap)           (RRT 探索)       (避障参考)
```

### 路径 2：任务分配 → 运动控制

```
Task Generator ──(service)──► Task Allocator
                                    │
                                    ▼ task_allocation 话题
                             Mission Executive
                                    │
                                    ▼ (service) controller
                             Lazy Traffic Controller
                                    │
                                    ▼ cmd_vel
                              agent_bringup → Gazebo
```

### 路径 3：控制反馈闭环

```
Controller 执行路径 → 发布 /status
                           │
                           ▼
                     Task Allocator 监听完成状态
                           │
                           ▼
                     分配新任务 → Mission Executive → Controller → ...
```

### 路径 4：AprilTag 检测

```
Gazebo Camera → apriltag_ros / AprilTagSim.py → /feedback/apriltag
                                                       │
                                        ┌──────────────┼──────────┐
                                        ▼                         ▼
                                  SLAM Toolbox                   GUI
                                (关联标签到扫描)            (显示检测结果)
```

### 路径 5：用户交互

```
GUI ──► /system_mission_command ──► Mission Executive (急停/恢复)
GUI ◄── /task_allocation_image  ◄── Task Allocator   (任务可视化)
GUI ◄── /percent_completed      ◄── Task Allocator   (探索进度)
GUI ◄── /all_agent_status       ◄── agent_bringup    (机器人状态)
```

---

## 七、Frontier Filter 话题（Task Generator 子节点）

| 话题名 | 消息类型 | 方向 | 文件 |
|--------|---------|------|------|
| `/frontier_filter/frontier_centroids` | `visualization_msgs/Marker` | 发布 | `frontier_filter.py:44-46` |
| `/frontier_filter/filtered_frontiers` | `robosar_messages/PointArray` | 发布 | `frontier_filter.py:47-49` |

**Task Allocator 订阅前沿点**:

```python
# robosar_task_allocator/src/frontier_ta.py:183-187
msg = rospy.wait_for_message("/frontier_filter/filtered_frontiers",
  PointArray, timeout=self.reassign_period)
```
