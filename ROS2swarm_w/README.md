# ROS2swarm_w

`ROS2swarm_w` 是当前工作区中的 ROS2 Humble + Webots 群体行为验证环境。它承担两类工作：

- 独立运行 ROS2swarm 群体行为模式，在 Webots 中验证 `aggregation`、`dispersion`、`navigation`、`roundflocking`、`transport` 等规则/学习策略。
- 在双容器 RoboSAR 系统中作为 ROS2/Webots 侧，启动 Webots 仿真、`robosar_bridge_adapter` 和 `ros1_bridge`，把 Webots 机器人接入 ROS1 RoboSAR 搜救系统。

当前部署默认位于 `4a6000` 设备的 `masar_ros2_bridge-w` 容器中。`4a6000` 是无头服务器，图形界面通过 TurboVNC 显示，Webots 通过 VirtualGL/vglrun 使用 GPU 加速。迁移到其他设备时，不要假设裸 ROS2 镜像能直接运行；需要配置 Webots、ROS2 Humble、`ros1_bridge` 和本工作区依赖。若新设备有本地显示/渲染环境，可以不配置 TurboVNC/VirtualGL，使用 `--no-vglrun` 直接运行。

相关依赖配置资料：
- webots安装：
官网下载deb包：https://github.com/cyberbotics/webots/releases
或wget -P /root https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo dpkg -i webots_2025a_amd64.deb
- webots_ros2软件包安装：https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html
软件包示例：https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started
- 两个容器ros1-ros2消息桥接工具包：https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/36

## 快速开始

### 1. 构建

```bash
cd /ROS2swarm_w
colcon build --symlink-install
source install/setup.bash
```

常用脚本会自动构建所需包并 source `install/setup.bash`，日常验证可以直接从下面入口开始。

### 2. 运行普通 Webots 群体仿真

```bash
cd /ROS2swarm_w
./start_simulation.sh --vglrun
```

这是 `4a6000` 无头服务器上的推荐方式。若在有本地显示/渲染能力的设备上运行，且未安装 `vglrun`，使用：

```bash
./start_simulation.sh --no-vglrun
```

已在另一台安装好 Webots、ROS2 等依赖但没有配置 vglrun 的设备上验证：直接运行 `./start_simulation.sh` 会因为脚本默认尝试 vglrun 而报 `env: 'vglrun': No such file or directory`；改用 `./start_simulation.sh --no-vglrun` 后可以正常启动 Webots 仿真和 ROS2 节点。

常用调试：

```bash
./start_simulation.sh --dry-run
./start_simulation.sh --no-vglrun
```

`start_simulation.sh` 当前默认参数在脚本开头配置：

```text
WEBOTS_WORLD=command_world.wbt
PATTERN=roundflocking_pattern
NUMBER_ROBOTS=5
ROBOT=e_puck
SENSOR_TYPE=lidar
WEBOTS_MODE=realtime
WEBOTS_GUI=true
```

如需无界面运行，修改脚本顶部 `WEBOTS_GUI=false`，或手动 launch 时传入 `webots_gui:=false`。

要切换实验，直接修改脚本顶部变量，或绕过脚本手动 launch：

```bash
ros2 launch launch_webots create_enviroment.launch.py \
  webots_world:=command_world.wbt \
  pattern:=transport_pattern \
  number_robots:=5 \
  robot:=e_puck \
  sensor_type:=lidar \
  x_min:=-1.0 x_max:=1.0 y_min:=-1.0 y_max:=1.0
```

### 3. 运行 RoboSAR Webots 桥接侧

先在 ROS2 容器启动 Webots 和 bridge：

```bash
cd /ROS2swarm_w
./start_robosar_webots.sh --vglrun
```

再到 ROS1 容器启动 RoboSAR：

```bash
cd /masar
./launch_robosar_w.sh
```

推荐先 dry-run 看真实命令：

```bash
./start_robosar_webots.sh --dry-run
```

`start_robosar_webots.sh` 当前默认：

```text
NUM_ROBOTS=5
ROBOT_TYPE=e_puck
WEBOTS_WORLD=robosar_apartment_smallv1.wbt
PUBLISH_STATIC_MAP=false
ROS1_MASTER_IP=localhost
```

启动顺序很重要：先 ROS2 Webots + `ros1_bridge`，再 ROS1 RoboSAR。ROS1 侧是 `/sim_map` 和 `map -> agent_N/odom` 的权威来源；ROS2 侧默认不发布 `/sim_map`。

## 项目结构

```text
ROS2swarm_w/
├── start_simulation.sh             # 普通 ROS2swarm Webots 群行为仿真入口
├── start_robosar_webots.sh         # RoboSAR 双容器架构中的 ROS2/Webots 入口
├── run_vnc_gpu.sh                  # VNC/X11/VirtualGL/GPU 环境对齐
├── add_robots_to_simulation.sh     # 已有仿真中追加机器人
├── docs/
│   └── robosar_webots_bridge_plan.md
└── src/
    ├── communication_interfaces/   # ROS2 自定义消息
    ├── launch_webots/              # Webots world、robot 生成、RoboSAR bridge adapter
    └── ros2swarm/                  # 群体行为模式、配置、checkpoint 适配
```

## ROS2 包说明

### communication_interfaces

定义 ROS2swarm 内部通信消息，包括：

```text
DoubleMessage.msg
FloatArray.msg
Int8Message.msg
ModelStatesStamped.msg
OpinionMessage.msg
OpinionMACMessage.msg
RangeData.msg
StringMessage.msg
```

其中 `/swarm_command` 常用 `communication_interfaces/msg/Int8Message` 手动触发群体行为：

```bash
ROS_DOMAIN_ID=42 ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message "{data: 1}"
```

### launch_webots

Webots 仿真包装层，负责：

- 根据 world 模板生成带机器人和任务物体的 `generated_*.wbt`。
- 支持 `e_puck` 和 `burger` 机器人。
- 为 transport、roundflocking、navigation 等任务自动生成目标物体。
- 发布 `/global_info`，供学习策略读取全局机器人/物体状态。
- 在 RoboSAR 模式下运行 `robosar_bridge_adapter`，完成 `robot_N` 与 `agent_N` 的话题/TF 映射。

关键文件：

```text
src/launch_webots/launch/create_enviroment.launch.py
src/launch_webots/launch/robosar_webots_sim.launch.py
src/launch_webots/launch_webots/environment_generator.py
src/launch_webots/launch_webots/global_info_supervisor.py
src/launch_webots/launch_webots/robosar_bridge_adapter.py
src/launch_webots/worlds/
src/launch_webots/maps/
```

### ros2swarm

群体行为模式实现包，包含规则控制器、学习策略适配、传感器层和安全层。

常用行为：

```text
aggregation_pattern
attraction_pattern
dispersion_pattern
drive_pattern
magnetometer_pattern
minimalist_flocking_pattern
navigation_pattern
navigation_pattern_neat
random_walk_pattern
rat_search_pattern
roundflocking_pattern
roundflocking_pattern_neat
transport_pattern
transport_pattern_neat
majority_rule_pattern
voter_model_pattern
voter_model_with_limiter_pattern
```

学习策略资源位于：

```text
src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/
```

当前包含 navigation、roundflocking、transport 的 `best_actor.pt` 和 NEAT genome package。`multi-agent-neat` 是独立训练仓库，本工作区主要消费训练结果并在 Webots/ROS2 中验证。

## RoboSAR 桥接口径

双容器模式下，ROS2 侧保持 Webots/ROS2swarm 命名，ROS1 侧保持 RoboSAR 命名，适配集中在：

```text
src/launch_webots/launch_webots/robosar_bridge_adapter.py
```

每台机器人 `N` 的映射为：

```text
/robot_N/scan
  -> /robosar_agent_bringup_node/agent_N/feedback/scan

/robot_N/odom
  -> /robosar_agent_bringup_node/agent_N/odom

/robosar_agent_bringup_node/agent_N/cmd_vel
  -> /robot_N/cmd_vel
```

TF 责任：

- ROS2 发布 `agent_N/odom -> agent_N/base_link`。
- ROS2 通过 `/tf_static` 发布 `base_link -> base_laser_link` 和 `base_link -> base_footprint`。
- ROS2 不发布 `map -> agent_N/odom`，该关系由 ROS1 的 bootstrap/SLAM 链路负责。

更详细的双容器启动、地图和 TF 说明见：

```text
docs/robosar_webots_bridge_plan.md
```

## VNC/GPU 注意事项

当前 VNC/GPU 设计是为 `4a6000` 无头服务器准备的：服务器没有直接可见的桌面显示，Webots 窗口通过 TurboVNC 显示，渲染通过 VirtualGL/vglrun 走 GPU。脚本会自动 source：

```text
run_vnc_gpu.sh
```

这样做的原因是宿主机或 VNC 重启后，`DISPLAY`、`XAUTHORITY` 和可用 GPU 设备可能变化；脚本会尽量自动对齐这些变量，并默认启用 vglrun。

常用检查：

```bash
./run_vnc_gpu.sh --check
```

常用环境变量：

```text
ROS2SWARM_VNC_DISPLAY=:1
ROS2SWARM_XAUTHORITY=/tmp/.docker.xauth
ROS2SWARM_USE_VGLRUN=true
ROS2SWARM_VGLRUN_DEVICE=/dev/dri/card2
ROBOSAR_USE_VGLRUN=true
ROBOSAR_VGLRUN_DEVICE=/dev/dri/card2
```

如果 Webots 窗口不可见或退回 llvmpipe，先检查 `DISPLAY`、`XAUTHORITY`、`vglrun` 和 `/dev/dri/*`。

在有本地显示/渲染能力的设备上，如果没有安装或不需要 vglrun，直接禁用它：

```bash
export ROS2SWARM_USE_VGLRUN=false
export ROBOSAR_USE_VGLRUN=false

./start_simulation.sh --no-vglrun
./start_robosar_webots.sh --no-vglrun
```

这种模式仍要求 Webots、ROS2 Humble、相关 Python/ROS 包等项目依赖已经安装好；禁用 vglrun 只是不走 TurboVNC/VirtualGL GPU 包装，不会改变 ROS2swarm 的算法逻辑。

## 结果和视频

`videos/` 中保留了当前各行为模式的演示视频，例如：

```text
aggregation_pattern.mp4
dispersion_pattern.mp4
navigation_pattern_neat.mp4
roundflocking_pattern_neat.mp4
transport_pattern_neat.mp4
```

这些视频适合快速判断每个行为模式的预期效果。`videos/.logs/` 是批量录制产生的日志。

## 测试

当前可运行的轻量测试：

```bash
cd /ROS2swarm_w
pytest tests/test_webots_start_scripts.py
pytest src/launch_webots/test/test_environment_generator.py
pytest src/ros2swarm/test
```

部分测试依赖已构建的 ROS2 环境、Python 包路径或特定系统依赖；若在新容器中失败，先确认已经执行 `colcon build --symlink-install` 并 source `install/setup.bash`。

## 常见问题

Webots/bridge 能启动，但 ROS1 侧没有机器人数据：

- 确认 ROS2 侧 `ros1_bridge dynamic_bridge --bridge-all-topics` 已启动。
- 确认两容器使用 host network 或 ROS master/DDS 可互通。
- 检查 ROS2 `/robot_N/scan`、`/robot_N/odom` 是否存在。
- 检查 ROS1 `/robosar_agent_bringup_node/agent_N/feedback/scan` 是否存在。

RoboSAR Webots 模式 TF 报错：

- 确认 ROS1 侧 `/use_sim_time=true`。
- 确认 `/clock` 在 ROS1/ROS2 两侧都能看到。
- 确认 `map -> agent_N/odom` 由 ROS1 侧 bootstrap/SLAM 发布，不要在 ROS2 侧重复发布。

机器人初始位置明显偏移：

- 检查 `robosar_bridge_adapter` 的 `apply_spawn_offset`。
- 当前 RoboSAR Webots pipeline 默认启用 spawn offset；如果 Webots driver 已输出全局 odom，需要关闭它。

切换 world 或地图：

- 普通仿真修改 `start_simulation.sh` 中的 `WEBOTS_WORLD`。
- RoboSAR 模式设置 `WEBOTS_WORLD`、`STATIC_MAP`、`SIM_SCENE` 或让 ROS2 侧写 `.robosar_scene_state.json` 后由 ROS1 侧读取。
