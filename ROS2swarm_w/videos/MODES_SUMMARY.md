# ROS2swarm 行为模式视频与说明

当前视频先按行为类型、再按机器人型号分开存放：

- `movement_patterns/<类型>/e_puck/`：既有 e-puck Webots 俯视录制。
- `movement_patterns/<类型>/burger/`：TurtleBot3 Burger Webots 俯视录制。
- `voting_patterns/e_puck/`：基于 e-puck 场景的投票/共识模式；不发布速度指令，视频展示观点传播和共识形成过程。

`magnetometer_pattern` 当前不再生成视频：现有 Webots e-puck/lidar 场景没有磁力计传感器，也没有 `/robot_i/magnetic_field` 输入，录制它会误导为“机器人不动”。该模式只保留文字说明。

## 运动模式


| 行为模式                           | 视频                                                                                                                         | 行为与运动解释                                                                                               |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `aggregation_pattern`          | [movement_patterns/basic/e_puck/aggregation_pattern.mp4](movement_patterns/basic/e_puck/aggregation_pattern.mp4)                         | 聚集行为。机器人根据局部距离数据判断附近机器人/物体位置，倾向于靠近群体中心，最终形成更紧凑的群体。                                                    |
| `attraction_pattern`           | [movement_patterns/basic/e_puck/attraction_pattern.mp4](movement_patterns/basic/e_puck/attraction_pattern.mp4)                           | 吸引行为。检测到邻近目标后，机器人朝目标方向移动；若没有检测到目标，则按配置的“独处”速度运动或停止。                                                   |
| `attraction_pattern2`          | [movement_patterns/basic/e_puck/attraction_pattern2.mp4](movement_patterns/basic/e_puck/attraction_pattern2.mp4)                         | 吸引行为变体。与 `attraction_pattern` 思路接近，但直接使用 `attraction_field` 计算方向，机器人对局部目标的吸引响应更直接。                    |
| `dispersion_pattern`           | [movement_patterns/basic/e_puck/dispersion_pattern.mp4](movement_patterns/basic/e_puck/dispersion_pattern.mp4)                           | 分散行为。机器人检测到邻近机器人/障碍物后产生排斥速度，逐渐拉开彼此距离；达到局部空旷状态后运动会减弱或停止。                                               |
| `drive_pattern`                | [movement_patterns/basic/e_puck/drive_pattern.mp4](movement_patterns/basic/e_puck/drive_pattern.mp4)                                     | 固定驱动行为。按配置周期持续发布固定线速度/角速度，是最基础的运动链路验证模式。                                                              |
| `magnetometer_pattern`         | 未生成视频                                                                                                                      | 磁力计朝向行为。理论上接收 `/robot_i/magnetic_field` 后计算当前磁场方向，再发布旋转指令使机器人朝目标磁北角对齐。当前 Webots 场景没有磁场传感输入，所以不适合录制视频。 |
| `minimalist_flocking_pattern`  | [movement_patterns/basic/e_puck/minimalist_flocking_pattern.mp4](movement_patterns/basic/e_puck/minimalist_flocking_pattern.mp4)         | 规则式 flocking。根据前后左右分区的距离信息进入避障、分散、吸引等状态，目标是在避免碰撞的同时保持群体协同行进。                                          |
| `navigation_pattern`           | [movement_patterns/basic/e_puck/navigation_pattern.mp4](movement_patterns/basic/e_puck/navigation_pattern.mp4)                           | VMAS actor 导航。每个机器人根据自身、目标和局部感知构造观测，由 `best_actor.pt` 输出动作并转换为差速速度，朝各自导航目标移动。                         |
| `navigation_pattern_neat`      | [movement_patterns/basic/e_puck/navigation_pattern_neat.mp4](movement_patterns/basic/e_puck/navigation_pattern_neat.mp4)                 | NEAT 版本导航。使用旧 NEAT 控制器链路完成导航，主要用于和当前 VMAS actor 版本做行为对比。                                              |
| `random_walk_pattern`          | [movement_patterns/basic/e_puck/random_walk_pattern.mp4](movement_patterns/basic/e_puck/random_walk_pattern.mp4)                         | 随机游走。机器人周期性随机选择转向/前进动作，适合验证运动发布、避障保护和基本探索效果。                                                          |
| `rat_search_pattern`           | [movement_patterns/basic/e_puck/rat_search_pattern.mp4](movement_patterns/basic/e_puck/rat_search_pattern.mp4)                           | 迷宫搜索行为。设计目标是沿墙、识别通道节点并寻找视觉目标；当前开放棋盘场景缺少墙体和目标线索，因此可见运动不如迷宫环境明显。                                        |
| `roundflocking_pattern`        | [movement_patterns/basic/e_puck/roundflocking_pattern.mp4](movement_patterns/basic/e_puck/roundflocking_pattern.mp4)                     | VMAS actor 环绕 flocking。机器人以目标位置和局部感知为输入，结合 Webots 适配后围绕目标形成较平滑的环绕/队形运动。                               |
| `roundflocking_pattern_neat`   | [movement_patterns/basic/e_puck/roundflocking_pattern_neat.mp4](movement_patterns/basic/e_puck/roundflocking_pattern_neat.mp4)           | NEAT 版本环绕 flocking。使用旧 pkl/NEAT 控制器链路，作为 Webots 中已验证可顺滑环绕的对照版本。                                       |
| `transport_pattern` (e-puck)   | [movement_patterns/basic/e_puck/transport_pattern.mp4](movement_patterns/basic/e_puck/transport_pattern.mp4)                             | VMAS actor 运输。机器人围绕运输物体和目标区域协同移动，推动或调整物体位置。                                                           |
| `transport_pattern_neat`       | [movement_patterns/basic/e_puck/transport_pattern_neat.mp4](movement_patterns/basic/e_puck/transport_pattern_neat.mp4)                   | NEAT 版本运输。使用旧 NEAT 控制器链路完成运输任务，用于和当前 actor 版本比较。                                                      |
| `transport_pattern` (Burger)   | [movement_patterns/basic/burger/transport_pattern.mp4](movement_patterns/basic/burger/transport_pattern.mp4)                             | 3 台 TurtleBot3 Burger 使用 actor 网络从 0.872 m 初始距离协作推动 `triangular_prism`；seed 252999 下物体中心进入 0.22 m 完成阈值。 |
| `transport_pattern_neat` (Burger) | [movement_patterns/basic/burger/transport_pattern_neat.mp4](movement_patterns/basic/burger/transport_pattern_neat.mp4)                 | 3 台 TurtleBot3 Burger 使用 NEAT 网络从 1.017 m 初始距离推动 `triangular_prism`；seed 33709 下物体中心进入 0.22 m 完成阈值。      |
| `discussed_dispersion_pattern` | [movement_patterns/combined/e_puck/discussed_dispersion_pattern.mp4](movement_patterns/combined/e_puck/discussed_dispersion_pattern.mp4) | 组合行为：先讨论，后分散。前半段通过多数规则形成共同观点，随后把观点映射为分散距离并转发 `dispersion_pattern` 的速度指令。视频左侧是 Webots 场景，右侧是讨论话题过程。    |


## 投票/共识模式


| 行为模式                               | 视频                                                                                                           | 行为与过程解释                                                                                                         |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------- |
| `voter_model_pattern`              | [voting_patterns/e_puck/voter_model_pattern.mp4](voting_patterns/e_puck/voter_model_pattern.mp4)                           | 选民模型。每个机器人周期性广播自己的观点，并随机采用收到的某个邻居观点；视频展示 `/voting_broadcast` 中观点从少数差异逐步收敛到一致。                                   |
| `majority_rule_pattern`            | [voting_patterns/e_puck/majority_rule_pattern.mp4](voting_patterns/e_puck/majority_rule_pattern.mp4)                       | 多数规则。机器人收集邻居观点后选择出现次数最多的观点；视频展示少数观点被多数观点覆盖并形成共识。                                                                |
| `voter_model_with_limiter_pattern` | [voting_patterns/e_puck/voter_model_with_limiter_pattern.mp4](voting_patterns/e_puck/voter_model_with_limiter_pattern.mp4) | 带邻居限制的选民模型。理论上只从 WiFi 信号足够强的近邻中采样观点。当前 Webots/容器没有真实 ad-hoc WiFi station dump，因此视频是该限制规则的过程可视化，而不是 Webots 运动录制。 |


投票/共识模式本身只处理通信和观点状态，不发布 `/drive_command` 或 `/cmd_vel`，所以在 Webots 里单独运行时机器人不应移动。它们的价值在于为组合行为提供“先达成共识，再执行运动”的上层决策，例如 `discussed_dispersion_pattern`。

## 录制与兼容性

`movement_patterns` 下的视频来自 Webots 场景录制；`voting_patterns` 下的视频是终端/状态可视化。
