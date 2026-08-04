# masar_ws

`masar_ws` 是一个可独立运行的 ROS1 Noetic catkin 工作空间，主要用于未知环境下
多机器人搜救系统 RoboSAR，以及 4 台 TurtleBot 响应组的 DEFORM 可变编队和任务
控制验证。

项目源码、构建结果、Qt GUI 和顶层脚本都位于本目录。Docker Hub 镜像
`talent18/masar_ros1_srv-g:v2` 提供 ROS1 Noetic、Gazebo 和系统依赖；源码通过挂载
进入容器，因此修改本项目通常不需要重建镜像。文中的 `/root/workspace/masar_ws`
是推荐的容器内挂载路径，不是宿主机固定路径。

## 快速开始

先取得独立运行环境镜像：

```bash
docker pull talent18/masar_ros1_srv-g:v2
```

创建容器时需要把包含本项目的 workspace 挂载到 `/root/workspace`，并按运行设备提供
host network、GPU、X11 和 `/dev/dri`。这些参数依赖宿主显示环境，本项目不假定固定的
宿主路径或显卡节点。以下命令默认在该镜像创建的容器中执行。

### 1. 构建工作空间

```bash
cd /root/workspace/masar_ws
./build_sequence.sh -j8
source devel/setup.bash
```

`build_sequence.sh` 会自动扫描 `src` 中的有效 catkin 包，清理旧 `build/devel/install/logs` 后全量构建。新增包后通常不需要手动维护构建列表。

### 2. 跑 Gazebo/Tepper 搜救主系统

```bash
cd /root/workspace/masar_ws
./launch_robosar_g.sh --vglrun
```

无界面或调试：

```bash
./launch_robosar_g.sh --headless --no-vglrun
./launch_robosar_g.sh --dry-run --vglrun
```

该入口会按顺序启动：

```text
roscore
rosparam set /use_sim_time true
robosar_agent_bringup
robosar_sim_bringup/robosar_sim_tepper.launch
slam_toolbox/robosar_mapping_sim_g.launch
robosar_task_generator
robosar_navigation/mission_executive.launch
robosar_controller/robosar_traffic_controller_node
robosar_task_allocator/mission_commander.py
robosar_gui.py
```

GUI 启动后通过 START/STOP/HOME 控制任务流程。

### 3. 跑 Webots 桥接搜救主系统

先在 ROS2 容器启动 Webots 和 bridge：

```bash
cd /root/workspace/ROS2swarm_w
./start_robosar_webots.sh --vglrun
```

再在 ROS1 容器启动 RoboSAR：

```bash
cd /root/workspace/masar_ws
./launch_robosar_w.sh
```

常用调试：

```bash
./launch_robosar_w.sh --dry-run
./launch_robosar_w.sh --no-rviz --no-vglrun
./launch_robosar_w.sh --sim-scene tepper
```

Webots 模式下，ROS2 侧负责 Webots、`robot_N` 和 `ros1_bridge`；ROS1 侧负责 `/sim_map`、SLAM、任务生成、任务分配、导航、控制和 GUI。

### 4. 跑响应组 DEFORM 编队导航 demo

```bash
cd /root/workspace/masar_ws
./run_response_deform_demo.sh --vglrun
```

常用调试：

```bash
./run_response_deform_demo.sh --headless
./run_response_deform_demo.sh --dry-run
./run_response_deform_demo.sh --cleanup-only
```

该 demo 只启动 4 台 `tb_0 ... tb_3` TurtleBot3 burger 响应组，不依赖搜索组。默认 world 为：

```text
src/robosar/robosar_emergency_response/worlds/deform_navigation_demo.world
```

目标序列在：

```text
src/robosar/robosar_emergency_response/config/response_goal_sequence.yaml
```

### 5. 跑响应组任务控制器 demo

```bash
cd /root/workspace/masar_ws
./run_response_controller_demo.sh roundflocking
./run_response_controller_demo.sh roundflocking-heuristic
./run_response_controller_demo.sh transport
./run_response_controller_demo.sh transport-checkpoint
```

这些 demo 不经过 DEFORM 导航和应急任务队列，直接验证生产任务控制器与 `/tb_i/task_cmd_vel`、最终 `/tb_i/cmd_vel` 输出。普通 `roundflocking` 使用 Tepper 的 `heuristic` 控制器，普通 `transport` 使用 `synchronized_slots`；显式 `*-checkpoint`/`*-heuristic` 名称用于算法对照。独立 world 没有探索组，因此不会查询 `agent_i` TF；fixture 会直接打开本 demo 的运动门。

### 6. 跑 Tepper + 响应组完整叠加

```bash
cd /root/workspace/masar_ws
./run_tepper_response_demo.sh --vglrun
```

该入口默认使用标准 ROS/Gazebo master（未另行设置时为 `11311/11345`），启动前通过 ROS API 关闭旧节点、清除失效注册和旧参数，并确认 `/clock` 无发布者后再启动 Gazebo。这样 ROS2 侧 `ros1_bridge` 仍可按标准 `11311` 连接，不需要知道 Tepper 专用端口。若已有 ROS 图由外部进程管理，可显式使用 `--shared-master`，此模式不清理节点，但发现已有 `/clock` 时会拒绝启动。完整 Tepper 入口不再维护独立端口分支，避免形成第二套主系统运行语义。

需要与 `/root/workspace/ROS2swarm_w` 桥接时，推荐先启动默认 Tepper 入口完成清理，再启动 `ros1_bridge`；若桥已经启动且图中没有其他仿真时钟，则使用 `--shared-master`。统一平台会自动使用共享主站并按 Run 管理进程，无需手工执行这一顺序。第二个终端按标准主站观测即可：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
```

第一阶段只验证“到达后切任务控制器”时，可用 6 秒自转占位：

```bash
./run_tepper_response_demo.sh --vglrun use_spin_placeholder:=true
```

默认 `use_spin_placeholder:=false` 时，victim/fire 进入 roundflocking，box 进入 transport。

## 平台集成

同级的 `masar-simulation-platform` 已集成本项目。平台 Compose 以固定 digest 复用
`talent18/masar_ros1_srv-g:v2`，把 workspace 挂载到 `/root/workspace`，并在首次缺少
`devel/setup.bash` 时自动执行 `catkin_make`。

平台调用本项目现有的 Gazebo/Webots、DEFORM、控制器和完整搜救脚本，并以一次 Run
统一管理这些脚本启动的进程。Qt GUI 只在平台 Run 中禁用，直接执行本项目脚本时仍然
保留。完整平台入口见[平台 README](../masar-simulation-platform/README.md)。平台存在
活动 Run 时不要手工启动第二套仿真。

## 项目结构

```text
masar_ws/
├── build_sequence.sh                  # catkin 全量/增量构建脚本
├── launch_robosar_g.sh                # ROS1 + Gazebo/Tepper 搜救主系统
├── launch_robosar_w.sh                # ROS1 + Webots 桥接搜救主系统
├── run_response_deform_demo.sh        # 响应组 DEFORM 编队导航 demo
├── run_tepper_response_demo.sh        # Tepper 主系统 + 响应组 overlay
├── run_response_controller_demo.sh    # roundflocking/transport 独立控制器 demo
├── docs/
│   ├── ROBOSAR_DEFORM_INTEGRATION_DESIGN.md
│   ├── RESPONSE_FORMATION_REQUIREMENTS.md
│   ├── RESPONSE_FORMATION_REFACTOR_PLAN.md
│   ├── RESPONSE_FORMATION_REFACTOR_PROGRESS.md
│   └── superpowers/                    # 已确认的历史需求与完整实施计划快照
├── videos/                            # 搜救系统演示视频
└── src/
    ├── robosar/                       # RoboSAR 主系统与响应组包装层
    └── robosar_deform_vendor/         # DEFORM ROS1 vendor 拷贝
```

## RoboSAR 主系统

RoboSAR 的原始搜救任务流保持在 `src/robosar` 中：

```text
robosar_agent_bringup       # 机器人/仿真 agent 接入，scan/odom/cmd_vel 桥接
robosar_sim_bringup         # Gazebo/Webots 场景上下文、/sim_map、AprilTagSim
slam_toolbox                # 多机器人 SLAM，项目内定制 launch/config
robosar_task_generator      # frontier/coverage 任务生成
robosar_task_allocator      # high-level task allocation / mission commander
robosar_navigation          # mission executive、MultiAStar、路径执行
robosar_controller          # 多机器人交通控制和避碰
robosar_gui                 # Mission Control GUI
robosar_messages            # 任务、应急对象、响应组等消息定义
```

Gazebo/Tepper 模式由 `robosar_sim_tepper.launch` 启动 Gazebo、map server、AprilTagSim、Khepera 仿真机器人和 RViz。

Webots 模式由 `robosar_sim_w.launch` 提供 ROS1 侧上下文：发布 `/sim_map`，运行 `AprilTagSim.py`，启动 `bootstrap_map_tf.py` 临时解除 `map -> agent_N/odom` 的 TF 死锁。真正的 Webots 和 `robot_N` 节点在 ROS2 容器中启动。

## 响应组与 DEFORM

响应组相关代码位于：

```text
src/robosar/robosar_emergency_response/
```

它负责把 4 台 TurtleBot3 burger 响应组接入 RoboSAR 应急任务：

```text
搜索组发现 victim/fire/box
-> emergency_task_manager_node.py 生成 EmergencyTask
-> formation_navigation_node 统一规划编队中心路径和连续形变槽位
-> response_cmd_mux_node.py 输出 NAV_FORMATION 速度
-> 导航到 staging pose 后切换 TASK_CONTROL
-> response_task_controller_node.py 执行 coordinated roundflocking / synchronized transport
-> /response_group/task_result 回传 DONE/FAILED
-> 全部任务完成后等待显式响应组 HOME
```

当前只有一条响应组运行主链：一个中心规划器、一个共享形变状态和一个
`/tb_i/cmd_vel` 最终发布者。旧的目标转发桥、四车独立规划器和独立到达协调器
已经退出运行链。

关键文件：

```text
src/robosar/robosar_emergency_response/launch/response_deform_demo.launch
src/robosar/robosar_emergency_response/launch/response_group_deform.launch
src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch
src/robosar/robosar_emergency_response/config/formation_navigation.yaml
src/robosar/robosar_emergency_response/config/response_group.yaml
src/robosar/robosar_emergency_response/config/tepper_response_group.yaml
src/robosar/robosar_emergency_response/config/response_safety.yaml
src/robosar/robosar_emergency_response/src/robosar_emergency_response/checkpoint_controller.py
```

更详细的实现说明见：

```text
docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md
```

## DEFORM Vendor

DEFORM 不直接使用顶层独立仓库运行，而是通过本工作空间 vendor 拷贝参与 catkin 构建：

```text
src/robosar_deform_vendor/
├── plan_manager
├── control/traj_opti
├── perception/drone_detect
├── perception/plan_env2d
├── planning/ot_process
├── planning/path_opti
├── planning/path_searching2d
├── planning/plan_rrt
├── planning/width_check
└── utility/to_visualize
```

响应组共享一套中心规划与连续形变配置：

```text
src/robosar/robosar_emergency_response/config/formation_navigation.yaml
```

`formation_navigation_node` 不会直接写 `/tb_i/cmd_vel`，而是写
`/tb_i/deform_cmd_vel`，由 `response_cmd_mux_node.py` 统一选择和过滤。

## Webots 双容器口径

Webots 搜救闭环使用 ROS1 和 ROS2 两个业务容器：

```text
masar_ros2_bridge-w:
  /root/workspace/ROS2swarm_w/start_robosar_webots.sh
  Webots + robot_N + robosar_bridge_adapter + ros1_bridge

masar_ros1_srv-g:
  /root/workspace/masar_ws/launch_robosar_w.sh
  RoboSAR 主任务流 + /sim_map + SLAM + GUI
```

无论直接运行还是由平台编排，都必须遵守以下启动顺序；平台场景
`exploration.webots.robosar` 会自动执行并按相反顺序停止：

```text
1. ROS2: ./start_robosar_webots.sh
2. ROS1: ./launch_robosar_w.sh
```

话题映射由 ROS2 侧 `robosar_bridge_adapter.py` 负责：

```text
/robot_N/scan
  -> /robosar_agent_bringup_node/agent_N/feedback/scan

/robot_N/odom
  -> /robosar_agent_bringup_node/agent_N/odom

/robosar_agent_bringup_node/agent_N/cmd_vel
  -> /robot_N/cmd_vel
```

桥接细节见：

```text
../ROS2swarm_w/docs/robosar_webots_bridge_plan.md
```

## 常用话题

RoboSAR 搜索主链路：

```text
/sim_map
/frontier_filter/filtered_frontiers
/task_allocation
/system_mission_command
/robosar_agent_bringup_node/agent_N/feedback/scan
/robosar_agent_bringup_node/agent_N/odom
/robosar_agent_bringup_node/agent_N/cmd_vel
```

应急对象与响应组：

```text
/robosar_sim/emergency_events
/robosar_sim/emergency_objects
/emergency/tasks
/emergency/active_task
/response_group/active_task
/response_group/behavior_mode
/response_group/status
/response_group/task_result
/tb_i/deform_cmd_vel
/tb_i/task_cmd_vel
/tb_i/cmd_vel
```

## VNC/GPU 注意事项

直接运行本项目脚本时，顶层入口会自动加载：

```text
run_vnc_gpu.sh
```

用于对齐容器内 X11/VirtualGL/GPU 环境。直接运行时应按当前设备检查 `DISPLAY`、
`XAUTHORITY` 和 vglrun target，不要照搬旧服务器的显卡节点编号。

常用检查：

```bash
./run_vnc_gpu.sh --check
```

常用环境变量：

```text
ROBOSAR_USE_VGLRUN=true
ROBOSAR_VGLRUN_DEVICE=:0
ROBOSAR_AUTO_VNC_GPU=1
ROBOSAR_RVIZ=true
ROBOSAR_GZ_GUI=true
```

如果 Gazebo/RViz/Webots 图形界面不可见，先检查 `DISPLAY`、`XAUTHORITY`、`vglrun` 和 `/dev/dri/*`。

如果换到有本地显示/渲染能力的设备，并且已经安装好 ROS、Gazebo/RViz、DEFORM 依赖等必要运行环境，可以不使用 vglrun：

```bash
export ROBOSAR_AUTO_VNC_GPU=0
export ROBOSAR_USE_VGLRUN=false

./launch_robosar_g.sh --no-vglrun
./launch_robosar_w.sh --no-vglrun
./run_response_deform_demo.sh --no-vglrun
./run_tepper_response_demo.sh --no-vglrun
```

这只改变图形显示/渲染包装方式，不改变 RoboSAR、SLAM、任务分配、DEFORM 编队导航或响应组控制器逻辑。若使用 Webots 双容器模式，ROS2 侧也需要对应使用 `./start_robosar_webots.sh --no-vglrun`。

通过统一平台运行时，平台会为 Gazebo、RViz 和 Webots 分配独立 Xvfb/noVNC 桌面，
并负责 VirtualGL 和浏览器代理；平台的 `127.0.0.1:18090` 不属于本项目独立运行接口。

## 结果与资料

```text
docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md
docs/TeamF_PDR.pdf
docs/TeamF_FinalReport.pdf
docs/robosar-基于图的多机器人快速搜索的覆盖与探索联合规划.pdf
docs/assets/deform_navigation_direction_reference.png
videos/
```

演示视频已放入 [videos](videos/)：

- [launch_robosar_g.mp4](videos/launch_robosar_g.mp4)：ROS1 + Gazebo/Tepper 搜救主系统启动与运行演示。
- [launch_robosar_w.mp4](videos/launch_robosar_w.mp4)：ROS1 + Webots 桥接搜救闭环启动与运行演示。

当前文档职责：

- [ROBOSAR_DEFORM_INTEGRATION_DESIGN.md](docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md)：当前响应组实现和运行链。
- [RESPONSE_FORMATION_REQUIREMENTS.md](docs/RESPONSE_FORMATION_REQUIREMENTS.md)：稳定需求与验收边界。
- [RESPONSE_FORMATION_REFACTOR_PLAN.md](docs/RESPONSE_FORMATION_REFACTOR_PLAN.md)：阶段划分和退出条件。
- [RESPONSE_FORMATION_REFACTOR_PROGRESS.md](docs/RESPONSE_FORMATION_REFACTOR_PROGRESS.md)：历史实跑证据与仍未完成的算法验收。
- [RoboSAR 包级索引](src/robosar/README.md)：`src/robosar` 当前组成和顶层入口指引；各子包 README 可能保留上游或包级历史说明，不作为平台操作手册。
- [平台运维手册](../masar-simulation-platform/docs/OPERATIONS.md)：当前 Compose 部署和 Web 操作入口。

## 测试与检查

轻量检查：

```bash
./build_sequence.sh --list-packages
./launch_robosar_g.sh --dry-run
./launch_robosar_w.sh --dry-run
./run_response_deform_demo.sh --dry-run
./run_response_controller_demo.sh --dry-run transport
```

当前响应组和任务状态快照的源码回归可直接运行：

```bash
python3 -m pytest -q \
  src/robosar/robosar_emergency_response/test \
  src/robosar/robosar_task_allocator/test/test_state_snapshot_publishers.py
```

运行测试前需要先 source ROS/catkin 环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 常见问题

`devel/setup.bash` 不存在：

- 先运行 `./build_sequence.sh -j8`。
- 确认容器内存在 `/opt/ros/noetic/setup.bash`。

Gazebo/Webots 图形界面不可见：

- 运行 `./run_vnc_gpu.sh --check`。
- 检查 `ROBOSAR_VGLRUN_DEVICE` 是否对应当前可用 GPU/X display。
- 可先用 `--headless` 或 `--no-rviz` 跑逻辑链路。

有本地显示设备但没装 vglrun：

- 设置 `ROBOSAR_AUTO_VNC_GPU=0` 和 `ROBOSAR_USE_VGLRUN=false`。
- 启动脚本加 `--no-vglrun`。
- 确认系统本身的 `DISPLAY`、OpenGL、Gazebo/RViz/Webots 安装可用。

Webots 模式 TF 报错：

- 确认 ROS2 侧先启动 `start_robosar_webots.sh`。
- 确认 ROS1 侧 `/use_sim_time=true`。
- 确认 ROS1 和 ROS2 都能收到 `/clock`。
- `map -> agent_N/odom` 应由 ROS1 bootstrap/SLAM 负责，不应在 ROS2 侧重复发布。

响应组不动：

- 未点击 START 时 `/response_group/motion_enabled=false` 且不动属于正确行为。
- 查看 `/response_group/behavior_mode`。
- 查看 `/response_group/motion_enabled`。
- NAV_FORMATION 看 `/tb_i/deform_cmd_vel`。
- TASK_CONTROL 看 `/tb_i/task_cmd_vel`。
- 最终确认 `/tb_i/cmd_vel` 是否被 `response_cmd_mux_node.py` 发布。

响应组目标不推进：

- 查看 `/response_group/status`。
- 查看 `/response_group/center_path` 和 `/response_group/formation_slots`。
- 检查 `/sim_map` 是否存在、`/response_group/active_task` 是否有效，以及状态中是否明确报告 `MAP_UNAVAILABLE` 或 `NO_STATIC_PATH`。
- 子任务阶段查看 `/response_group/task_result`；surround 必须四车分别完成两圈，transport 必须形成前排—箱体、后排—前排实体接触链。
