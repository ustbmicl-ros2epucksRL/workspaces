# ROS2swarm_w

`ROS2swarm_w` 是一个可独立运行的 ROS2 Humble + Webots 群体行为验证项目。它承担两类工作：

- 独立运行 ROS2swarm 群体行为模式，在 Webots 中验证 `aggregation`、`dispersion`、`navigation`、`roundflocking`、`transport` 等规则/学习策略。
- 在双容器 RoboSAR 系统中作为 ROS2/Webots 侧，启动 Webots 仿真、`robosar_bridge_adapter` 和 `ros1_bridge`，把 Webots 机器人接入 ROS1 RoboSAR 搜救系统。

项目源码、构建结果和运行脚本都位于本目录。Docker Hub 镜像
`talent18/masar_ros2_bridge-w:v2` 提供 ROS2 Humble、Webots、`ros1_bridge` 和系统
依赖；源码通过挂载进入容器，因此修改本项目通常不需要重建镜像。文中的
`/root/workspace/ROS2swarm_w` 是推荐的容器内挂载路径，不是宿主机固定路径。

相关依赖配置资料：
- webots安装：
官网下载deb包：https://github.com/cyberbotics/webots/releases
或wget -P /root https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo dpkg -i webots_2025a_amd64.deb
- webots_ros2软件包安装：https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html
软件包示例：https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started
- 两个容器ros1-ros2消息桥接工具包：https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/issues/36

## 快速开始

先取得独立运行环境镜像：

```bash
docker pull talent18/masar_ros2_bridge-w:v2
```

创建容器时需要把包含本项目的 workspace 挂载到 `/root/workspace`，并按运行设备提供
host network、GPU、X11 和 `/dev/dri`。这些参数依赖宿主显示环境，本项目不假定固定的
宿主路径或显卡节点。以下命令默认在该镜像创建的容器中执行。

### 1. 构建

```bash
cd /root/workspace/ROS2swarm_w
colcon build --symlink-install
source install/setup.bash
```

常用脚本会自动构建所需包并 source `install/setup.bash`，日常验证可以直接从下面入口开始。

### 2. 运行普通 Webots 群体仿真

```bash
cd /root/workspace/ROS2swarm_w
./start_simulation.sh
```

脚本默认使用 `ROS2SWARM_USE_VGLRUN=auto`：检测到 `vglrun` 时使用 VirtualGL，未检测
到时直接启动。也可以显式选择：

```bash
./start_simulation.sh --vglrun
./start_simulation.sh --no-vglrun
```

常用调试：

```bash
./start_simulation.sh --dry-run
./start_simulation.sh --no-vglrun
```

`start_simulation.sh` 当前默认参数在脚本开头配置：

```text
WEBOTS_WORLD=command_world_2m.wbt
PATTERN=navigation_pattern
NUMBER_ROBOTS=3
RANDOM_SEED=42
ROBOT=e_puck
SENSOR_TYPE=lidar
WEBOTS_MODE=realtime
WEBOTS_GUI=true
ROS_DOMAIN_ID=42
```

如需无界面运行，修改脚本顶部 `WEBOTS_GUI=false`，或手动 launch 时传入 `webots_gui:=false`。

要切换实验，直接修改脚本顶部变量，或绕过脚本手动 launch：

```bash
ROS_DOMAIN_ID=42 ros2 launch launch_webots create_enviroment.launch.py \
  webots_world:=command_world.wbt \
  pattern:=transport_pattern \
  number_robots:=5 \
  robot:=e_puck \
  sensor_type:=lidar \
  x_min:=-1.0 x_max:=1.0 y_min:=-1.0 y_max:=1.0
```

### 2.1 运行时追加机器人

先通过 `start_simulation.sh` 启动普通仿真，再运行：

```bash
bash add_robots_to_simulation.sh
```

追加机器人会读取 `/global_info`，在 `[-1.0, 1.0]` 生成范围内避开已有机器人、任务物体和随机障碍物。该入口依赖 `create_enviroment.launch.py` 启动的 `GlobalInfoSupervisor`，不能单独配合 `start_webots.launch.py` 使用。默认 `command_world.wbt` 没有内部墙体，且生成范围位于 3.5 m arena 内部；使用带内部墙体的自定义 world 时，应配置不与墙体相交的生成范围。

当区域无法容纳新机器人时，启动会报错并停止，不会使用重叠位置强制生成。

### 3. 运行 RoboSAR Webots 桥接侧

先在 ROS2 容器启动 Webots 和 bridge：

```bash
cd /root/workspace/ROS2swarm_w
./start_robosar_webots.sh --vglrun
```

再到 ROS1 容器启动 RoboSAR：

```bash
cd /root/workspace/masar_ws
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

直接调试时启动顺序很重要：先 ROS2 Webots + `ros1_bridge`，再 ROS1 RoboSAR。
ROS1 侧是 `/sim_map` 和 `map -> agent_N/odom` 的权威来源；ROS2 侧默认不发布
`/sim_map`。

## 平台集成

同级的 `masar-simulation-platform` 已集成本项目。平台 Compose 以固定 digest 复用
`talent18/masar_ros2_bridge-w:v2`，把 workspace 挂载到 `/root/workspace`，并在首次
缺少 `install/setup.bash` 时自动执行 `colcon build`。

平台的 `swarm.webots.ros2swarm` 场景调用 `start_simulation.sh` 对应的 ROS2 launch，
`exploration.webots.robosar` 场景先调用 `start_robosar_webots.sh`，再启动 ROS1 侧。
完整平台入口见[平台 README](../masar-simulation-platform/README.md)。平台存在活动 Run
时不要手工启动第二套 Webots 或 ROS 图。

## 项目结构

```text
ROS2swarm_w/
├── start_simulation.sh             # 普通 ROS2swarm Webots 群行为仿真入口
├── start_robosar_webots.sh         # RoboSAR 双容器架构中的 ROS2/Webots 入口
├── run_vnc_gpu.sh                  # VNC/X11/VirtualGL/GPU 环境对齐
├── add_robots_to_simulation.sh     # 已有仿真中追加机器人
├── docs/
│   ├── robosar_webots_bridge_plan.md
│   ├── EPUCK2_PI_PUCK_END_TO_END_DEPLOYMENT.md
│   ├── EPUCK2_REAL_DEPLOYMENT_FLOW.md
│   ├── PT_NEAT_REAL_ROBOT_DEPLOYMENT_GUIDE.md
│   └── REAL_ROBOT_EXPERIMENT_METRICS.md
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
discussed_dispersion_pattern
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

直接运行本项目脚本时会自动 source：

```text
run_vnc_gpu.sh
```

它用于对齐容器内 X11、VirtualGL 和 GPU 环境。直接运行时应按当前设备检查
`DISPLAY`、`XAUTHORITY` 和 vglrun target，不要照搬旧服务器的显卡节点编号。
这样做的原因是宿主机或 VNC 重启后，`DISPLAY`、`XAUTHORITY` 和可用 GPU 设备可能变化；脚本会尽量自动对齐这些变量，并默认启用 vglrun。

常用检查：

```bash
./run_vnc_gpu.sh --check
```

常用环境变量：

```text
ROS2SWARM_VNC_DISPLAY=:1
ROS2SWARM_XAUTHORITY=/tmp/.docker.xauth
ROS2SWARM_USE_VGLRUN=auto
ROS2SWARM_VGLRUN_DEVICE=:0
ROBOSAR_USE_VGLRUN=auto
ROBOSAR_VGLRUN_DEVICE=:0
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

通过统一平台运行时，平台会在独立 Xvfb/noVNC 桌面中显示 Webots，并负责所需的
VirtualGL 和浏览器代理；这些平台端口不属于本项目独立运行接口。

## 结果和视频

`videos/` 中按行为类型和机器人型号保留各模式的演示视频，例如：

```text
movement_patterns/basic/e_puck/aggregation_pattern.mp4
movement_patterns/basic/e_puck/navigation_pattern_neat.mp4
movement_patterns/basic/e_puck/transport_pattern_neat.mp4
movement_patterns/basic/burger/transport_pattern.mp4
movement_patterns/combined/e_puck/discussed_dispersion_pattern.mp4
voting_patterns/e_puck/voter_model_pattern.mp4
```

这些视频适合快速判断每个行为模式的预期效果。`videos/.logs/` 是批量录制产生的日志。

## 文档职责

- [RoboSAR Webots 桥接说明](docs/robosar_webots_bridge_plan.md)：当前双容器启动、地图与 TF 口径。
- [e-puck2 + Pi-puck 端到端部署指南](docs/EPUCK2_PI_PUCK_END_TO_END_DEPLOYMENT.md)：当前唯一正式实物启动、标定和验收流程。
- [e-puck2 实物部署现场流程](docs/EPUCK2_REAL_DEPLOYMENT_FLOW.md)：现场按顺序执行的精简部署清单。
- [PT/NEAT 实机策略契约](docs/PT_NEAT_REAL_ROBOT_DEPLOYMENT_GUIDE.md)：策略输入、输出和 sim-to-real 约束。
- [实物实验指标记录规范](docs/REAL_ROBOT_EXPERIMENT_METRICS.md)：现场实验的成功判定和资源指标记录格式。
- [ROS2swarm 配置参考](src/ros2swarm/config/CONFIG.md)：包级参数含义；不替代本 README 或平台运维手册。
- `src/launch_webots/assets/**/docs` 是 Webots 传感器资产资料，不是 MASAR 操作手册。

## 测试

当前发布门使用的轻量测试：

```bash
cd /root/workspace/ROS2swarm_w
python3 -m pytest tests/test_webots_start_scripts.py
python3 -m pytest src/launch_webots/test/test_environment_generator.py
python3 -m pytest src/ros2swarm/test/test_real_robot_geometry.py \
       src/ros2swarm/test/test_real_robot_policy_mode.py \
       src/ros2swarm/test/test_real_robot_state.py \
       src/ros2swarm/test/test_global_info_bridge.py \
       src/ros2swarm/test/test_fiducial_detector.py \
       src/ros2swarm/test/test_hardware_protection_layer.py \
       src/ros2swarm/test/test_roundflocking_neat_safety.py
python3 scripts/validate_epuck2_mock_chain.py --all-patterns --robot-count 3
```

这些入口覆盖平台启动、世界生成、顶视定位、共享保护层、世界位姿桥、PT/NEAT
输入边界，以及三机器人六种 PT/NEAT 实物 mock 控制链。
完整的 `src/ros2swarm/test` 还保留上游旧接口、launch 和代码风格检查，当前不作为平台发布门；
若要运行它，先确认已经执行 `colcon build --symlink-install` 并 source `install/setup.bash`，
再单独处理其中的历史测试债务。

## 常见问题

当前已知的 ROS2swarm 行为限制：

- `aggregation_pattern` 已验证可由 `e_puck` 完成；`burger` 当前不能完成同一模式，
  属于 ROS2swarm/机器人模型适配问题，不是平台编排故障。
- 手动选择 `sensor_type=ir` 时仍有传感器层问题。平台默认使用 `lidar`，不会走该
  已知异常路径。

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
