# RoboSAR × Webots 桥接实现说明（按当前代码重写）

## 1. 目标与边界

本方案描述的是当前实际运行的双容器系统，而不是早期设计草案。

- ROS1 容器：`/root/catkin_ws2`
  - 运行环境：ROS1 Noetic
  - 入口脚本：`/root/catkin_ws2/launch_robosar.sh`
  - 主要职责：任务生成、任务分配、导航、控制、多机器人 SLAM、GUI
- ROS2 容器：`/root/ROS2swarm_w`
  - 运行环境：ROS2 Humble + Webots
  - 入口脚本：`/root/ROS2swarm_w/start_robosar_webots.sh`
  - 主要职责：Webots 仿真、`robot_N` 机器人节点、ROS1/ROS2 桥接适配
- 桥接方式：`ros1_bridge dynamic_bridge --bridge-all-topics`

当前设计原则：

- ROS1 侧继续保留 RoboSAR 既有话题名和节点组织。
- ROS2 侧继续保留 ROS2swarm/Webots 的 `robot_N` 话题命名。
- 两侧不直接改业务节点，全部命名映射与 TF 适配放在 `robosar_bridge_adapter.py`。
- 两侧统一使用 Webots 的仿真时钟，不再采用 `use_sim_time=false` 的旧方案。

## 2. 部署前提

ROS2 容器当前按下面命令启动：

```bash
docker run -it --name robosar_ros2_bridge \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --ipc=host \
  --network=host \
  --group-add video \
  --device=/dev/dri \
  osrf/ros:humble-desktop-full
```

这带来两个直接结论：

- 由于使用了 `--network=host`，ROS1 容器和 ROS2 容器可以直接通过主机网络通信，`ROS1_MASTER_IP` 默认可设为 `localhost`。
- 不要再额外引入独立的跨容器时钟同步方案，系统已经以 `/clock` 为统一时钟源。

## 3. 当前真实架构

### 3.1 ROS1 侧

`launch_robosar.sh` 会先启动 `roscore`，然后立即执行：

```bash
rosparam set /use_sim_time true
```

之后再启动以下核心节点：

- `robosar_agent_bringup`
- `map_server`（发布 `/sim_map`）
- `AprilTagSim.py`
- `slam_toolbox`（`robosar_mapping_sim.launch`）
- `robosar_task_generator`
- `robosar_navigation`
- `robosar_controller`
- `mission_commander.py -s=true`
- `robosar_gui.py`

注意：当前 ROS1 侧已经是 `/sim_map` 的权威来源，因此 ROS2 侧不应默认再起一个同名 `map_server`。

### 3.2 ROS2 侧

`start_robosar_webots.sh` 会：

1. `colcon build --packages-select launch_webots`
2. 启动 `robosar_webots_sim.launch.py`
3. 等待 Webots 就绪
4. 启动 `ros1_bridge dynamic_bridge --bridge-all-topics`

`robosar_webots_sim.launch.py` 当前负责：

- 随机生成 Webots 世界中的机器人初始位姿
- 为每个 `robot_N` 启动对应控制器节点
- 启动 `robosar_bridge_adapter`
- 可选启动 ROS2 本地 `nav2_map_server`

当前脚本已修改为：

- `STATIC_MAP` 默认关闭，避免与 ROS1 侧 `/sim_map` 重复发布
- 只有显式设置 `STATIC_MAP=tepper.yaml` 之类的参数时，才在 ROS2 侧额外起本地地图服务

## 4. 话题与 TF 映射

### 4.1 话题映射

每台机器人保持如下映射：

| 方向 | ROS2 Webots 侧 | 中间适配层 | ROS1 RoboSAR 侧 |
|------|----------------|------------|-----------------|
| 激光 | `/robot_N/scan` | `robosar_bridge_adapter` | `/robosar_agent_bringup_node/agent_N/feedback/scan` |
| 里程计 | `/robot_N/odom` | `robosar_bridge_adapter` | `/robosar_agent_bringup_node/agent_N/odom` |
| 速度控制 | `/robosar_agent_bringup_node/agent_N/cmd_vel` | `robosar_bridge_adapter` | `/robot_N/cmd_vel` |

### 4.2 TF 链

当前系统中正确的 TF 责任划分如下：

- `map -> agent_N/odom`
  - 发布者：ROS1 `slam_toolbox`
  - 说明：全局 SLAM 结果，ROS2 侧不应重复发布
- `agent_N/odom -> agent_N/base_link`
  - 发布者：ROS2 `robosar_bridge_adapter`
  - 说明：基于 Webots `/robot_N/odom`，并叠加机器人 spawn offset
- `agent_N/base_link -> agent_N/base_laser_link`
  - 发布者：ROS2 `robosar_bridge_adapter`
  - 说明：固定传感器外参，走 `/tf_static`
- `agent_N/base_link -> agent_N/base_footprint`
  - 发布者：ROS2 `robosar_bridge_adapter`
  - 说明：固定 TF，走 `/tf_static`

这里最关键的一点是：

- `map -> odom` 不在 ROS2 侧生成
- 固定 TF 不应以 10 Hz 动态方式反复发布
- `scan`、`odom`、动态 TF 必须共享同一个 Webots 仿真时间戳

## 5. `robosar_bridge_adapter.py` 的当前职责

`/root/ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py`

该节点当前承担四件事：

1. 话题重命名：`robot_N` 与 `agent_N` 双向映射
2. Odom 坐标修正：把 Webots 本地 odom 叠加 spawn offset，变成 RoboSAR 期望的全局起点
3. TF 生成：从 odom 生成 `agent_N/odom -> agent_N/base_link`
4. 速度裁剪：把 ROS1 给出的 `cmd_vel` 限制到 e-puck / burger 的可执行范围

本轮修改后的约束：

- `scan.header.stamp` 保留 Webots 原始时间戳，不再强制改成 adapter 当前时间
- `odom.header.stamp` 保留 Webots 原始时间戳；若上游 stamp 缺失才回退到节点时钟
- `odom -> base_link` TF 与该 odom 使用同一 stamp
- `base_laser_link`、`base_footprint` 改为一次性通过 `StaticTransformBroadcaster` 发布

这些改动的目的，是消除“scan 是一个时刻、TF 是另一个时刻”的问题。

## 6. ROS1 SLAM 侧的当前约束

`/root/catkin_ws2/src/robosar/robosar_SLAM_toolbox/slam_toolbox/config/robosar_mapping.yaml`

当前系统在 5 台机器人下，`sync_slam_toolbox_node` 很容易出现 scan 队列积压。为降低过期 scan 参与匹配的概率，当前配置已经做了保守收敛：

- `throttle_scans: 8`
- `tag_publish_period: 1.0`
- `map_update_interval: 1.0`
- `minimum_time_interval: 0.7`
- `transform_timeout: 0.35`
- `tf_buffer_duration: 90.0`
- `minimum_travel_distance: 0.35`
- `minimum_travel_heading: 0.35`
- `loop_close_across_agents: false`

设计意图：

- 先保证时间一致性与轨迹稳定
- 再追求跨机器人回环和更高刷新率

如果后续仍有明显积压，再考虑两项进一步措施：

1. 把 Webots 激光更新频率继续下调到 `6-8 Hz`
2. 将 `robosar_mapping_sim.launch` 从 `sync_slam_toolbox_node` 切到 `async_slam_toolbox_node`

## 7. 启动顺序

推荐顺序如下：

1. 启动 ROS2 容器
2. 在 ROS2 容器中运行 `start_robosar_webots.sh`
3. 确认 Webots 已启动，`robosar_bridge_adapter` 已运行
4. 确认 `ros1_bridge` 已启动
5. 确认 `/clock` 已经桥接成功
6. 再启动 ROS1 容器中的 `launch_robosar.sh`

原因：

- ROS1 侧节点必须在 `/use_sim_time=true` 且 `/clock` 可用的前提下启动
- `slam_toolbox` 如果先在 wall-time 下启动，再切到 sim-time，很容易留下 epoch 时间戳和 TF extrapolation 问题

## 8. 排障基线

如果再次出现 RViz 中位姿漂移、地图重叠、墙体毛边，优先检查以下项目：

### 8.1 时钟

- ROS1：`rostopic echo /clock -n 1`
- ROS2：`ros2 topic echo /clock --once`

两边都必须能看到 Webots 时钟推进。

### 8.2 TF

- `agent_N/odom -> agent_N/base_link` 必须连续更新
- `agent_N/base_link -> agent_N/base_laser_link` 必须来自 `/tf_static`
- 不允许 ROS2 侧再有额外的 `map -> agent_N/odom` 发布者

### 8.3 scan 队列

如果 `rosout` 中再次出现以下日志，说明 `slam_toolbox` 仍然过载：

- `Input scan queue saturated`
- `Failed to compute odom pose`
- `Lookup would require extrapolation into the past`

此时不要先去改时钟方案，优先从以下方向收敛：

- 降低激光频率
- 进一步提高 `throttle_scans`
- 暂时禁用跨机器人回环
- 必要时切 `async_slam_toolbox_node`

## 9. 禁止继续沿用的旧假设

以下内容已经不符合当前实现，不应再继续使用：

- “ROS1 侧 `use_sim_time=false`”
- “ROS2 侧发布静态 `map -> agent_N/odom`”
- “固定 TF 通过 `/tf` 周期性发布即可”
- “ROS2 侧默认应始终启动自己的 `/sim_map`”

这些旧假设会直接导致时钟不一致、TF 树冲突或重复地图源。

## 10. 关键文件

- ROS2 适配层：
  - `/root/ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py`
- ROS2 启动脚本：
  - `/root/ROS2swarm_w/start_robosar_webots.sh`
- ROS2 Webots 主 launch：
  - `/root/ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py`
- ROS1 总启动脚本：
  - `/root/catkin_ws2/launch_robosar.sh`
- ROS1 SLAM 参数：
  - `/root/catkin_ws2/src/robosar/robosar_SLAM_toolbox/slam_toolbox/config/robosar_mapping.yaml`

以上五个文件共同定义了当前双容器系统的真实行为，应以它们为准，而不是以旧文档或旧启动习惯为准。
