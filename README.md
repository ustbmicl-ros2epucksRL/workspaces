# 多智能体群行为与搜救仿真工作区

本工作区把“群行为策略学习”和“机器人仿真验证”放在同一个工程视角下组织：

- `multi-agent-neat` 负责在 VMAS 向量化多智能体环境中训练和评估群行为策略，重点是 Pure-NEAT 与 EA-RL warmup 等学习算法。
- `VectorizedMultiAgentSimulator` 是 `multi-agent-neat` 训练和评估使用的 VMAS 向量化环境代码，主要作为上游开源环境依赖，不是当前工作的主要开发对象。
- `ROS2swarm_w` 负责把群行为策略落到 ROS2 Humble + Webots 的 3D 物理仿真中验证，包括势场/规则策略和从 VMAS/NEAT 训练得到的策略。
- `masar_ws` 负责未知环境下的多智能体搜救系统，保留 RoboSAR 的搜索、SLAM、任务生成、任务分配、导航、控制和 GUI 主链路，并扩展 Gazebo/Webots 验证、ROS1-ROS2 桥接和应急响应组能力。
- `masar-simulation-platform` 是上述 ROS1/ROS2 仿真的统一 Web 控制与观测入口，按“群行为验证、探索组运行、完整搜救”三个模块组织场景。
- `DEFORM` 是可变编队导航的参考/上游工作区；`masar_ws` 内当前使用的是 `src/robosar_deform_vendor` 里的 ROS1 vendor 拷贝。

## 目录说明


| 目录                                                                         | 定位                         | 主要内容                                                                                      |
| -------------------------------------------------------------------------- | -------------------------- | ----------------------------------------------------------------------------------------- |
| [multi-agent-neat](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/README.md) | 独立 Git 仓库，作为本工作区的学习算法子模块索引 | VMAS 任务上的 Pure-NEAT、EA-RL、warm-start EA-RL、BenchMARL/heuristic baseline、结果分析与视频导出         |
| [VectorizedMultiAgentSimulator](VectorizedMultiAgentSimulator/README.md) | VMAS 开源环境                  | PyTorch 向量化 2D 多智能体仿真器，为 `multi-agent-neat` 提供 `flocking`、`transport` 等训练/评估环境            |
| [ROS2swarm_w](ROS2swarm_w/README.md)                                     | ROS2 + Webots 群行为验证        | Webots 场景、`robot_N` 节点、e-puck/Burger 支持、ROS2swarm 行为策略、NEAT/VMAS 策略加载、RoboSAR Webots 桥接适配 |
| [masar_ws](masar_ws/README.md)                                           | ROS1 RoboSAR 搜救系统          | 多机器人 SLAM、任务生成、任务分配、导航控制、GUI、Gazebo 搜救闭环、Webots 桥接闭环、应急响应组与 DEFORM 集成                     |
| [masar-simulation-platform](masar-simulation-platform/PLATFORM_DESIGN.md) | 统一仿真验证与可视化平台 | React 工作台、运行编排 API、Gazebo/RViz noVNC、Webots streaming、ROS Web sidecar 和运行归档 |
| [DEFORM](DEFORM/)                                                        | 可变编队导航参考项目                 | ROS1 Noetic 下的多机器人可变编队规划与局部避障参考实现                                                         |

## 统一平台预览

当前统一前端、场景目录 API、Gazebo/RViz 显示 sidecar 和 Caddy 单入口已经可以运行。服务器端启动/更新预览：

```bash
cd /home/zjc/workspace/masar-simulation-platform/frontend
npm ci
npm run build

cd /home/zjc/workspace/masar-simulation-platform/deploy
docker compose --profile dev-api up -d --build api display webots-guard caddy
```

SSH 配置保持 `LocalForward 18089 127.0.0.1:18090` 后，在本地浏览器打开 `http://127.0.0.1:18089/`。`dev-api` 在未挂载 Runtime Agent secret 时可安全查看真实的三模块/六场景目录，但不能启动 ROS 业务进程；完整场景联调将在平台 Agent 与 Robot Web Tools 适配阶段接通。


## 总体关系

```mermaid
flowchart LR
  VMAS[VectorizedMultiAgentSimulator<br/>VMAS 向量化环境]
  NEAT[multi-agent-neat<br/>群行为策略学习]
  ROS2[ROS2swarm_w<br/>ROS2 Humble + Webots]
  ROS1[masar_ws<br/>ROS1 Noetic RoboSAR]
  DEFORM[DEFORM / robosar_deform_vendor<br/>应急组可变编队导航]

  VMAS --> NEAT
  NEAT -->|策略 checkpoint / genome| ROS2
  NEAT -->|checkpoint 控制器语义| ROS1
  ROS2 <-->|ros1_bridge dynamic_bridge| ROS1
  DEFORM --> ROS1
```



学习链路先在 VMAS 中用大规模并行环境快速获得群行为策略，再把策略带到 Webots/Gazebo 中做物理仿真验证。ROS1 的 RoboSAR 主链路负责未知环境搜索救援任务，ROS2 侧主要承担 Webots 机器人仿真和桥接适配。

## 运行环境

当前容器环境部署在 `4a6000` 这台设备上。下面的容器名和本地 conda 环境名是这台设备上的运行口径；换到其他设备时，不应假设这些名称和依赖已经存在。


| 子系统                             | 推荐环境                                                                                      | 说明                                                                                                                      |
| ------------------------------- | ----------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `multi-agent-neat`              | `NEAT-ROBO` conda 环境（这里的 `NEAT-ROBO` 是 `4a6000` zjc用户下的本地 conda 环境名；其他设备上请替换为重新创建后的实际环境。） | `4a6000` 当前用户环境中的本地 conda 环境，用于训练、benchmark、结果分析和 VMAS rollout 导出；迁移到其他设备时需要按依赖重新创建                                     |
| `VectorizedMultiAgentSimulator` | Python + PyTorch                                                                          | 作为 VMAS 环境源码依赖，可按上游 README 安装                                                                                           |
| `ROS2swarm_w`                   | `4a6000` 上的 `masar_ros2_bridge-w` 容器，ROS2 Humble + Webots + `ros1_bridge`                 | 容器内通常对应 `/ROS2swarm_w`，负责 Webots、`robot_N` 节点和 ROS1/ROS2 桥接；旧的裸 `docker run osrf/ros:humble-desktop-full` 命令已不能代表当前完整环境 |
| `masar_ws`                      | `4a6000` 上的 `masar_ros1_srv-g` 容器，ROS1 Noetic                                             | 容器内通常对应 `/masar` 或 catkin workspace，负责 RoboSAR 主链路、Gazebo/Webots 搜救闭环和应急组                                               |


如果需要迁移 ROS 容器环境，建议优先把 `4a6000` 上已配置好的容器提交为镜像并推送到 Docker Hub，再在新设备上拉取使用。示例命令如下，镜像名按实际 Docker Hub 账号调整：

```bash
docker commit masar_ros2_bridge-w <dockerhub-user>/masar_ros2_bridge-w:humble-webots-bridge
docker push <dockerhub-user>/masar_ros2_bridge-w:humble-webots-bridge

docker commit masar_ros1_srv-g <dockerhub-user>/masar_ros1_srv-g:noetic-robosar
docker push <dockerhub-user>/masar_ros1_srv-g:noetic-robosar
```

注意：`docker commit` 只保存容器文件系统，不会保存通过 volume 绑定挂载进容器的宿主目录；迁移时/迁移后需要同步本工作区代码和需要保留的结果文件。

## 双容器 Webots 桥接

Webots 搜救闭环采用 ROS1/ROS2 双容器架构：


| 侧      | 工作区           | 主要职责                                                                                                                         |
| ------ | ------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| ROS2 侧 | `ROS2swarm_w` | 启动 Webots、生成 `robot_N` 机器人节点、发布 scan/odom/TF、运行 `robosar_bridge_adapter`、启动 `ros1_bridge dynamic_bridge --bridge-all-topics` |
| ROS1 侧 | `masar_ws`    | 启动 RoboSAR 原有任务流，包括 SLAM、任务生成、任务分配、导航、控制、多机器人 GUI                                                                            |


推荐启动顺序：

```bash
# 1. 在 masar_ros2_bridge-w / ROS2swarm_w 中启动 Webots 与 bridge
cd ROS2swarm_w
./start_robosar_webots.sh

# 2. 在 masar_ros1_srv-g / masar_ws 中启动 RoboSAR Webots 侧任务流
cd masar_ws
./launch_robosar_w.sh
```

桥接设计要点见 [ROS2swarm_w/docs/robosar_webots_bridge_plan.md](ROS2swarm_w/docs/robosar_webots_bridge_plan.md)。当前 `masar_ros2_bridge-w` 容器已经包含 ROS1/ROS2 桥接能力，运行时由 `start_robosar_webots.sh` 启动 `ros1_bridge dynamic_bridge --bridge-all-topics`。关键约束是：两侧统一使用 Webots `/clock`，ROS1 侧保留 RoboSAR 话题和节点组织，ROS2 侧保留 Webots/ROS2swarm 的 `robot_N` 命名，命名映射与 TF 适配集中在 `robosar_bridge_adapter.py` 中。

核心话题映射：


| 方向  | ROS2 Webots 侧                                 | ROS1 RoboSAR 侧                                      |
| --- | --------------------------------------------- | --------------------------------------------------- |
| 激光  | `/robot_N/scan`                               | `/robosar_agent_bringup_node/agent_N/feedback/scan` |
| 里程计 | `/robot_N/odom`                               | `/robosar_agent_bringup_node/agent_N/odom`          |
| 控制  | `/robosar_agent_bringup_node/agent_N/cmd_vel` | `/robot_N/cmd_vel`                                  |


## 主要工作流

### 1. VMAS 策略学习与评估

`multi-agent-neat` 在 `vmas/flocking` 和 `vmas/transport` 等任务上比较：

- Pure-NEAT recurrent / feed-forward。
- EA-RL cold start。
- Pure-NEAT warm-start 后的 EA-RL，当前 dashboard 选择口径为 `rec + layerwise + 50 gen`。
- heuristic 与 BenchMARL baseline。

常用入口：

```bash
cd multi-agent-neat
conda activate NEAT-ROBO

# 单次实验、链路验证或 debug
python train.py ...

# 统一 benchmark 和结果分析说明
less benchmark/README.md

# 重新生成 transport dashboard
python benchmark/analysis/ea_rl_warmup_dashboard.py \
  --task vmas/transport \
  --warmup-generations 10 30 50 70 \
  --budget-frames 362400000
```

更多训练、baseline、视频导出说明见 [multi-agent-neat/benchmark/README.md](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/benchmark/README.md)。

### 2. Webots 群行为验证

`ROS2swarm_w` 在 ROS2 Humble + Webots 中实现/验证多种群行为策略：

- 规则或势场类策略：aggregation、attraction、dispersion、drive、magnetometer、minimalist flocking、navigation、random walk、rat search、round flocking、transport 等。
- 学习策略：`navigation_pattern_neat`、`roundflocking_pattern_neat`、`transport_pattern_neat` 等，从 VMAS/NEAT 训练产物加载策略。
- RoboSAR 桥接：`launch_webots` 包负责 Webots 场景、机器人节点、地图上下文、`robosar_bridge_adapter` 和 ROS1/ROS2 bridge 运行。

策略演示视频已放在 [ROS2swarm_w/videos/movement_patterns/basic](ROS2swarm_w/videos/movement_patterns/basic/)：

- [navigation_pattern_neat.mp4](ROS2swarm_w/videos/movement_patterns/basic/navigation_pattern_neat.mp4)
- [roundflocking_pattern_neat.mp4](ROS2swarm_w/videos/movement_patterns/basic/roundflocking_pattern_neat.mp4)
- [transport_pattern_neat.mp4](ROS2swarm_w/videos/movement_patterns/basic/transport_pattern_neat.mp4)
- [minimalist_flocking_pattern.mp4](ROS2swarm_w/videos/movement_patterns/basic/minimalist_flocking_pattern.mp4)

### 3. RoboSAR 搜救系统与应急响应

`masar_ws` 保留 RoboSAR 原有未知环境搜救主链路：

```text
SLAM / 地图
-> frontier 与 coverage 任务生成
-> 任务分配
-> 导航规划
-> 多机器人控制
-> GUI 监控与任务启动
```

Gazebo 搜救闭环入口：

```bash
cd masar_ws
./launch_robosar_g.sh
```

Webots 桥接搜救闭环入口：

```bash
cd masar_ws
./launch_robosar_w.sh
```

应急响应组/DEFORM demo 入口：

```bash
cd masar_ws
./run_response_deform_demo.sh
```

应急组当前目标是让搜索组发现 victim/fire/box 等应急对象后，生成应急任务；4 台 TurtleBot 响应组接单后通过 DEFORM 风格可变编队导航到 staging pose，再切换到任务控制器。详细设计见 [masar_ws/docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md](masar_ws/docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md)。

未知环境下搜救策略已有部分演示视频放在 [masar_ws/videos](masar_ws/videos/)；当前索引见 [masar_ws/videos/README.md](masar_ws/videos/README.md)：

- [launch_robosar_g.mp4](masar_ws/videos/launch_robosar_g.mp4)：ROS1 + Gazebo/Tepper 搜救主系统启动与运行演示。
- [launch_robosar_w.mp4](masar_ws/videos/launch_robosar_w.mp4)：ROS1 + Webots 桥接搜救闭环启动与运行演示。

## 当前结果与展示

### VMAS 算法结果

`multi-agent-neat` 是独立 Git 子模块，GitHub 上的结果链接会跳转到子模块仓库查看。当前 [`multi-agent-neat/results/analysis`](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/tree/main/results/analysis) 已包含 flocking 与 transport 的结果图、CSV、dashboard 和视频索引。


| 任务               | 最佳/当前选中策略                                | 结果摘要                                                                                | 入口                                                                                                                                                                                                                                              |
| ---------------- | ---------------------------------------- | ----------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `vmas/flocking`  | EA-RL warmup, `rec + layerwise + 50 gen` | 6 seeds，Final IQM `-24.486`；相对 heuristic 提升 `38.896%`，相对 Pure NEAT rec 提升 `10.991%` | [算法对比](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/flocking/algorithm_training_comparison.md), [训练曲线](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/flocking/algorithm_training_comparison.png), [视频索引](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/flocking/video_index.md)    |
| `vmas/transport` | EA-RL warmup, `rec + layerwise + 50 gen` | 6 seeds，Final IQM `79.642`；相对 heuristic 提升 `3.521%`，相对 Pure NEAT rec 提升 `44.221%`   | [算法对比](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/transport/algorithm_training_comparison.md), [训练曲线](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/transport/algorithm_training_comparison.png), [视频索引](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/transport/video_index.md) |


可视化结果还包括：

- [flocking warmup 配置热力图](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/flocking/warmup_selection_heatmap.png)
- [flocking 最优策略轨迹](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/flocking/best_policy_trajectory.png)
- [flocking 最优策略时间线](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/flocking/best_policy_timeline.png)
- [transport warmup 配置热力图](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/transport/warmup_selection_heatmap.png)
- [transport 最优策略轨迹](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/transport/best_policy_trajectory.png)
- [transport 最优策略时间线](https://github.com/ustbmicl-ros2epucksRL/multi-agent-neat/blob/main/results/analysis/transport/best_policy_timeline.png)

### Webots 行为视频

`ROS2swarm_w/videos` 中已有 Webots 物理仿真录屏，覆盖规则策略与学习策略：

```text
movement_patterns/basic/aggregation_pattern.mp4
movement_patterns/basic/dispersion_pattern.mp4
movement_patterns/basic/navigation_pattern.mp4
movement_patterns/basic/navigation_pattern_neat.mp4
movement_patterns/basic/roundflocking_pattern.mp4
movement_patterns/basic/roundflocking_pattern_neat.mp4
movement_patterns/basic/transport_pattern.mp4
movement_patterns/basic/transport_pattern_neat.mp4
movement_patterns/combined/discussed_dispersion_pattern.mp4
voting_patterns/majority_rule_pattern.mp4
```

完整列表见 [ROS2swarm_w/videos](ROS2swarm_w/videos/)。

### RoboSAR 与应急响应资料

`masar_ws/docs` 中保留系统设计、报告和集成说明：

- [ROBOSAR_DEFORM_INTEGRATION_DESIGN.md](masar_ws/docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md)
- [TeamF_FinalReport.pdf](masar_ws/docs/TeamF_FinalReport.pdf)
- [TeamF_PDR.pdf](masar_ws/docs/TeamF_PDR.pdf)
- [robosar-基于图的多机器人快速搜索的覆盖与探索联合规划.pdf](masar_ws/docs/robosar-基于图的多机器人快速搜索的覆盖与探索联合规划.pdf)

## 开发与验证提示

ROS2/Webots 侧可先做 dry-run 检查命令和环境变量：

```bash
cd ROS2swarm_w
./start_robosar_webots.sh --dry-run
```

ROS1/RoboSAR 侧也提供 dry-run：

```bash
cd masar_ws
./launch_robosar_g.sh --dry-run
./launch_robosar_w.sh --dry-run
./run_response_deform_demo.sh --dry-run
```

如果运行 Webots 桥接闭环，建议固定顺序：先 ROS2 Webots + bridge，再 ROS1 RoboSAR。这样 ROS1 节点在 `/use_sim_time=true` 且 `/clock` 已桥接的前提下启动，可以减少 TF 时间外推和 scan 队列积压问题。
