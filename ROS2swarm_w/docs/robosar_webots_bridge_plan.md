# RoboSAR × Webots 双容器桥接说明（当前项目口径）

本文描述当前项目中 ROS1 RoboSAR 与 ROS2 Webots 的真实桥接方式。两个业务项目可以
使用 Docker Hub 的运行环境镜像分别启动，源码和构建结果通过 workspace 挂载进入容器；
统一平台只是把这两个容器和本项目入口脚本纳入一次 Run 的可复现编排。本文中的容器
内路径不代表宿主机固定路径。

## 1. 目标与边界

当前 Webots 搜救闭环采用双容器架构：

- ROS2 容器：`masar_ros2_bridge-w`
  - 运行环境：ROS2 Humble + Webots + `ros1_bridge`
  - 工作区：`/root/workspace/ROS2swarm_w`
  - 入口脚本：`start_robosar_webots.sh`
  - 主要职责：启动 Webots，生成 `robot_N` 机器人节点，运行 `robosar_bridge_adapter`，启动 `ros1_bridge dynamic_bridge --bridge-all-topics`
- ROS1 容器：`masar_ros1_srv-g`
  - 运行环境：ROS1 Noetic
  - 工作区：`/root/workspace/masar_ws`
  - 入口脚本：`launch_robosar_w.sh`
  - 主要职责：启动 RoboSAR 主任务流，包括 SLAM、任务生成、任务分配、导航、控制、多机器人 GUI

当前设计原则：

- ROS1 侧保留 RoboSAR 既有话题名、节点组织和任务流。
- ROS2 侧保留 Webots/ROS2swarm 的 `robot_N` 话题命名。
- 两侧业务节点不互相侵入，话题命名映射与 TF 适配集中在 `robosar_bridge_adapter.py`。
- 两侧统一使用 Webots `/clock`，ROS1 侧必须在 `/use_sim_time=true` 的前提下启动。

## 2. 部署与迁移口径

平台部署入口位于 `masar-simulation-platform`：

```bash
cd <workspace>/masar-simulation-platform
make docker-start
```

Compose 使用以下公开且按 digest 固定的业务镜像：

```text
talent18/masar_ros1_srv-g:v2@sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59
talent18/masar_ros2_bridge-w:v2@sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c
```

镜像只提供 ROS/Webots/Gazebo/bridge 等运行环境。无论独立运行还是由 Compose 编排，
都应将 workspace 读写挂载到 `/root/workspace`，源码和构建 overlay 不固化进镜像。首次启动若缺少
`masar_ws/devel/setup.bash` 或 `ROS2swarm_w/install/setup.bash`，同一个
`make docker-start` 会在对应容器内自动执行 `catkin_make` 或 `colcon build`；后续
复用挂载目录中的结果。宿主 workspace 路径由当前 checkout 自动推导，只有目录
不是标准同级布局时才设置 `MASAR_WORKSPACE_ROOT`。

完整前提、端口和远程访问方式见
[平台运维手册](../../masar-simulation-platform/docs/OPERATIONS.md)。不再使用手工
`docker commit` 或 Compose 外预先存在的业务容器作为部署前提。

## 3. 启动顺序

平台固定启动顺序为先 ROS2 Webots + bridge，再 ROS1 RoboSAR，并在停止 Run 时按
相反顺序结束该次运行的全部进程。以下命令只用于进入两个业务容器后的独立调试；
平台存在活动 Run 时不要重复执行。

### 3.1 ROS2 侧

在 `masar_ros2_bridge-w` 容器中：

```bash
cd /root/workspace/ROS2swarm_w
./start_robosar_webots.sh
```

常用 dry-run：

```bash
./start_robosar_webots.sh --dry-run
```

`start_robosar_webots.sh` 当前会做这些事：

1. 加载 `run_vnc_gpu.sh`，对齐容器内 VNC/GPU/X11 环境。
2. 解析场景状态和地图路径，并写入共享 scene state 文件。
3. `colcon build --symlink-install --packages-select launch_webots`。
4. `source ./install/setup.bash`。
5. 启动 `ros2 launch launch_webots robosar_webots_sim.launch.py ...`。
6. 设置 `ROS_MASTER_URI=http://${ROS1_MASTER_IP}:11311`。
7. 启动 `ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`。

当前脚本默认值：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `NUM_ROBOTS` | `5` | Webots 中生成 `robot_0 ... robot_4` |
| `ROBOT_TYPE` | `e_puck` | 可按当前代码切换到 burger |
| `WEBOTS_WORLD` | `robosar_apartment_smallv1.wbt` | Webots world 模板 |
| `PUBLISH_STATIC_MAP` | `false` | ROS2 侧默认不发布 `/sim_map`，ROS1 侧是地图权威来源 |
| `ROS1_MASTER_IP` | `localhost` | host network 下通常可用 |

注意：`STATIC_MAP` 仍会被解析并传入 launch，用于场景上下文和共享状态；只有 `PUBLISH_STATIC_MAP=true` 时 ROS2 侧才会额外启动 `nav2_map_server` 发布 `/sim_map`。

### 3.2 ROS1 侧

在 `masar_ros1_srv-g` 容器中：

```bash
cd /root/workspace/masar_ws
./launch_robosar_w.sh
```

常用 dry-run：

```bash
./launch_robosar_w.sh --dry-run
```

`launch_robosar_w.sh` 当前会做这些事：

1. 加载 `run_vnc_gpu.sh`，对齐 VNC/GPU 环境。
2. 启动 `roscore`。
3. 设置 `rosparam set /use_sim_time true`。
4. 启动 `robosar_agent_bringup`。
5. 启动 `robosar_sim_bringup robosar_sim_w.launch`，其中包含 `/sim_map` map server、`AprilTagSim.py`、`bootstrap_map_tf.py` 和可选 RViz。
6. 启动 `slam_toolbox robosar_mapping_sim_w.launch`。
7. 启动 `robosar_task_generator`、`robosar_navigation`、`robosar_controller`、`mission_commander.py`；直接运行时还启动原 Qt `robosar_gui.py`。

统一平台为子进程设置 `MASAR_PLATFORM_MODE=1`，因此不会启动原 Qt GUI，也不会让
脚本创建脱离 Runtime Agent 的子会话或执行原生全局清理。该差异只作用于平台
Run；容器内直接运行脚本时保留原行为。

这样启动的原因是：ROS1 侧节点需要在 `/use_sim_time=true` 且 `/clock` 已可用的条件下进入运行，否则容易出现 TF extrapolation、epoch 时间戳和 scan 队列积压问题。

## 4. 当前架构

### 4.1 ROS2 Webots 侧

`robosar_webots_sim.launch.py` 当前负责：

- 基于 world 模板生成 `generated_robosar.wbt`。
- 随机生成机器人初始位姿，并避开 world 中的障碍和任务对象。
- 为每个 `robot_N` 启动 Webots 控制器节点。
- 启动 `robosar_bridge_adapter`，传入 `spawn_x/spawn_y/spawn_yaw`、`apply_spawn_offset`、`normalize_scan` 等参数。
- 可选启动 ROS2 本地 `nav2_map_server`，但默认关闭。

### 4.2 ROS1 RoboSAR 侧

`robosar_sim_w.launch` 当前负责 Webots 桥接场景下的 ROS1 仿真上下文：

- 设置 `/use_sim_time=true`。
- 用 `map_server` 发布 `/sim_map`。
- 启动 `AprilTagSim.py` 发布 victim/fire/transport 等仿真对象与 marker。
- 启动 `bootstrap_map_tf.py`，在 SLAM 初期临时发布 identity `map -> agent_N/odom`，避免导航控制链 TF 死锁。
- 可选启动 RViz。

`launch_robosar_w.sh` 在此基础上继续启动 SLAM、任务生成、导航、交通控制、任务分配和 GUI。

## 5. 话题与 TF 映射

### 5.1 话题映射

每台机器人保持如下映射：

| 方向 | ROS2 Webots 侧 | 中间适配层 | ROS1 RoboSAR 侧 |
| --- | --- | --- | --- |
| 激光 | `/robot_N/scan` | `robosar_bridge_adapter` | `/robosar_agent_bringup_node/agent_N/feedback/scan` |
| 里程计 | `/robot_N/odom` | `robosar_bridge_adapter` | `/robosar_agent_bringup_node/agent_N/odom` |
| 速度控制 | `/robosar_agent_bringup_node/agent_N/cmd_vel` | `robosar_bridge_adapter` | `/robot_N/cmd_vel` |

当前 `robosar_bridge_adapter` 不做速度裁剪，只负责把 RoboSAR 的 `cmd_vel` 转发给 Webots 侧 `robot_N/cmd_vel`。速度限制应由上游控制器、机器人模型或专门安全层处理。

### 5.2 TF 责任划分

当前系统中正确的 TF 责任划分如下：

- `map -> agent_N/odom`
  - 责任侧：ROS1。
  - 启动初期可由 `bootstrap_map_tf.py` 临时发布 identity TF。
  - SLAM 收敛后由 `slam_toolbox` 发布校正后的全局关系。
  - ROS2 侧不发布该 TF。
- `agent_N/odom -> agent_N/base_link`
  - 发布者：ROS2 `robosar_bridge_adapter`。
  - 来源：Webots `/robot_N/odom`，默认叠加 spawn offset。
  - 时间戳：使用 odom 原始 stamp；stamp 缺失时才回退到节点时钟。
- `agent_N/base_link -> agent_N/base_laser_link`
  - 发布者：ROS2 `robosar_bridge_adapter`。
  - 通道：`/tf_static`。
- `agent_N/base_link -> agent_N/base_footprint`
  - 发布者：ROS2 `robosar_bridge_adapter`。
  - 通道：`/tf_static`。

`robosar_bridge_adapter` 会周期性重发 `/tf_static`，目的是让后启动的 `ros1_bridge` 或晚订阅者也能收到静态 TF。这里仍然是静态 TF 语义，不应改成 `/tf` 上的动态 10 Hz 固定外参。

## 6. `robosar_bridge_adapter.py` 当前职责

当前文件：

```text
ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py
```

该节点当前承担这些职责：

1. 话题重命名：`robot_N` 与 `agent_N` 双向映射。
2. Odom 坐标修正：当 `apply_spawn_offset=true` 时，把 Webots 本地 odom 叠加 spawn offset，变成 RoboSAR 期望的全局起点。
3. TF 生成：从 odom 生成 `agent_N/odom -> agent_N/base_link`。
4. 静态 TF：发布并周期性重发 `base_laser_link`、`base_footprint`。
5. scan 适配：保留 Webots 原始时间戳；必要时规范负 `angle_increment`，补齐 `scan_time/time_increment`。

当前约束：

- `scan.header.stamp` 优先保留 Webots 原始时间戳。
- `odom.header.stamp` 优先保留 Webots 原始时间戳。
- `odom -> base_link` TF 与该 odom 使用同一 stamp。
- `map -> agent_N/odom` 不在 ROS2 侧生成。

## 7. `/sim_map` 与场景状态

Webots 桥接模式下，ROS1 侧 `robosar_sim_w.launch` 的 `map_server` 是 `/sim_map` 的默认权威来源。ROS2 侧 `start_robosar_webots.sh` 会解析 `STATIC_MAP` 并写入共享 scene state，方便 ROS1 侧选择对应地图和任务上下文；但 `PUBLISH_STATIC_MAP=false` 时，ROS2 不发布 `/sim_map`。

如果调试时显式设置 `PUBLISH_STATIC_MAP=true`，需要确认 ROS1 侧没有同时发布同名 `/sim_map`，否则会出现重复地图源。

## 8. 排障基线

### 8.1 时钟

检查两侧是否都能看到 Webots `/clock`：

```bash
# ROS1
rostopic echo /clock -n 1

# ROS2
ros2 topic echo /clock --once
```

两侧必须看到同一仿真时钟推进。ROS1 侧应确认：

```bash
rosparam get /use_sim_time
```

结果应为 `true`。

### 8.2 TF

重点检查：

- `agent_N/odom -> agent_N/base_link` 连续更新。
- `agent_N/base_link -> agent_N/base_laser_link` 来自 `/tf_static`。
- `map -> agent_N/odom` 只由 ROS1 侧的 bootstrap/SLAM 链路负责。
- ROS2 侧没有额外发布 `map -> agent_N/odom`。

### 8.3 scan 队列

如果 ROS1 日志再次出现以下信息，说明 `slam_toolbox` 仍可能过载：

- `Input scan queue saturated`
- `Failed to compute odom pose`
- `Lookup would require extrapolation into the past`

优先从以下方向收敛：

- 降低 Webots 激光频率。
- 提高 SLAM 的 `throttle_scans`。
- 暂时禁用跨机器人回环。
- 必要时从同步 SLAM 切到异步 SLAM。

## 9. 禁止继续沿用的旧假设

以下内容已经不符合当前实现，不应再继续使用：

- 旧的 `docker run ... osrf/ros:humble-desktop-full` 可直接复现当前 ROS2 环境。
- ROS1 侧入口是旧的 `launch_robosar.sh`。
- ROS1 侧 `use_sim_time=false`。
- ROS2 侧默认发布 `/sim_map`。
- ROS2 侧发布 `map -> agent_N/odom`。
- 固定传感器外参通过 `/tf` 周期性动态发布即可。
- `robosar_bridge_adapter` 负责速度裁剪。
- ROS1/ROS2 两个业务容器必须预先在 Compose 外手工创建并保持不重启。
- 通过手工 `docker commit` 迁移当前运行环境。

## 10. 关键文件

- ROS2 总入口：`ROS2swarm_w/start_robosar_webots.sh`
- ROS2 Webots launch：`ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py`
- ROS2 适配层：`ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py`
- ROS1 Webots 总入口：`masar_ws/launch_robosar_w.sh`
- ROS1 Webots 场景上下文：`masar_ws/src/robosar/robosar_sim_bringup/launch/robosar_sim_w.launch`
- ROS1 Webots SLAM launch：`masar_ws/src/robosar/robosar_SLAM_toolbox/slam_toolbox/launch/robosar_mapping_sim_w.launch`
- ROS1 Webots SLAM 参数：`masar_ws/src/robosar/robosar_SLAM_toolbox/slam_toolbox/config/robosar_mapping_w.yaml`
- 平台部署与操作：`masar-simulation-platform/docs/OPERATIONS.md`

以上文件共同定义当前双容器系统的真实行为，应以它们为准。
