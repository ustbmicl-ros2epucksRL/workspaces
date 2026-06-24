# masar_ws

`masar_ws` 是当前项目中的 ROS1 Noetic catkin 工作空间，主要用于未知环境下多机器人搜救系统 RoboSAR，以及 4 台 TurtleBot 响应组的 DEFORM 可变编队/任务控制验证。

当前部署默认位于 `4a6000` 设备的 `masar_ros1_srv-g` 容器中。`4a6000` 是无头服务器，图形界面通过 TurboVNC 显示，Gazebo/RViz/Webots 相关渲染通过 VirtualGL/vglrun 使用 GPU 加速。它可以独立跑 Gazebo/Tepper 搜救闭环，也可以和 `ROS2swarm_w` 所在的 `masar_ros2_bridge-w` 容器配合，用 Webots 作为 3D 物理仿真器。若迁移到有本地显示/渲染能力的设备，可以禁用 VNC/GPU 自动包装，使用 `--no-vglrun` 运行。

## 快速开始

### 1. 构建工作空间

```bash
cd /masar
./build_sequence.sh -j8
source devel/setup.bash
```

`build_sequence.sh` 会自动扫描 `src` 中的有效 catkin 包，清理旧 `build/devel/install/logs` 后全量构建。新增包后通常不需要手动维护构建列表。

### 2. 跑 Gazebo/Tepper 搜救主系统

```bash
cd /masar
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
cd /ROS2swarm_w
./start_robosar_webots.sh --vglrun
```

再在 ROS1 容器启动 RoboSAR：

```bash
cd /masar
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
cd /masar
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
cd /masar
./run_response_controller_demo.sh roundflocking
./run_response_controller_demo.sh roundflocking-heuristic
./run_response_controller_demo.sh transport
./run_response_controller_demo.sh transport-checkpoint
```

这些 demo 不经过 DEFORM 导航和应急任务队列，直接验证 roundflocking/transport 控制器与 `/tb_i/task_cmd_vel` 输出。

### 6. 跑 Tepper + 响应组完整叠加

```bash
cd /masar
./run_tepper_response_demo.sh --vglrun
```

第一阶段只验证“到达后切任务控制器”时，可用 6 秒自转占位：

```bash
./run_tepper_response_demo.sh --vglrun use_spin_placeholder:=true
```

默认 `use_spin_placeholder:=false` 时，victim/fire 进入 roundflocking，box 进入 transport。

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
│   └── ROBOSAR_DEFORM_INTEGRATION_DESIGN.md
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
-> deform_dispatch_bridge_node.py 发布 /move_base_simple/goal
-> DEFORM plan_manager_node x4 编队导航
-> response_group_coordinator_node.py 判定到达
-> response_cmd_mux_node.py 切换 NAV_FORMATION / TASK_CONTROL
-> response_task_controller_node.py 执行 roundflocking / transport / spin
-> /response_group/task_result 回传 DONE/FAILED
```

关键文件：

```text
src/robosar/robosar_emergency_response/launch/response_deform_demo.launch
src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch
src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml
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

响应组使用专用 4 机器人配置：

```text
src/robosar/robosar_emergency_response/config/deform/frm_shape_response4.yaml
```

当前 DEFORM 输出不会直接写 `/tb_i/cmd_vel`，而是写 `/tb_i/deform_cmd_vel`，由 `response_cmd_mux_node.py` 统一选择和过滤。

## Webots 双容器口径

Webots 搜救闭环需要两个容器配合：

```text
masar_ros2_bridge-w:
  /ROS2swarm_w/start_robosar_webots.sh
  Webots + robot_N + robosar_bridge_adapter + ros1_bridge

masar_ros1_srv-g:
  /masar/launch_robosar_w.sh
  RoboSAR 主任务流 + /sim_map + SLAM + GUI
```

推荐启动顺序：

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

当前 VNC/GPU 配置是为 `4a6000` 无头服务器准备的：服务器没有直接可见的桌面显示，仿真器图形界面通过 TurboVNC 显示，渲染通过 VirtualGL/vglrun 使用 GPU。顶层启动脚本会自动加载：

```text
run_vnc_gpu.sh
```

用于对齐容器内 VNC/X11/VirtualGL/GPU 环境。这样做的原因是宿主机或 VNC 重启后，`DISPLAY`、`XAUTHORITY` 和 `/dev/dri/*` 可能变化；脚本会尽量自动修正这些变量，避免 Gazebo/RViz/Webots 窗口发到不可见的 X server，或退回软件渲染。

常用检查：

```bash
./run_vnc_gpu.sh --check
```

常用环境变量：

```text
ROBOSAR_USE_VGLRUN=true
ROBOSAR_VGLRUN_DEVICE=/dev/dri/card0
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

## 测试与检查

轻量检查：

```bash
./build_sequence.sh --list-packages
./launch_robosar_g.sh --dry-run
./launch_robosar_w.sh --dry-run
./run_response_deform_demo.sh --dry-run
./run_response_controller_demo.sh --dry-run transport
```

当前 `robosar_emergency_response` 中显式保留的源码测试主要是：

```bash
pytest src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py
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

- 查看 `/response_group/behavior_mode`。
- NAV_FORMATION 看 `/tb_i/deform_cmd_vel`。
- TASK_CONTROL 看 `/tb_i/task_cmd_vel`。
- 最终确认 `/tb_i/cmd_vel` 是否被 `response_cmd_mux_node.py` 发布。

响应组目标不推进：

- 查看 `/response_group/status`。
- 查看 `/tb_i/deform_reached`。
- 检查 `response_group.yaml` 或 `tepper_response_group.yaml` 中的 goal region 半径和 timeout。

