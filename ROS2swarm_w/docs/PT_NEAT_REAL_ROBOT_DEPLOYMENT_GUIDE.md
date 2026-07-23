# Transport、RoundFlocking、Navigation 的 PT/NEAT 输入与实机部署指南

> 分析日期：2026-07-19  
> 分析范围：当前仓库中的 6 个 ROS 2 控制节点、随仓库分发的 3 个 `best_actor.pt`、3 个 NEAT genome package、Webots `/global_info` 发布器、launch/config、安全保护层与实机启动脚本。  
> 本文描述的是**当前代码实际执行的逻辑**。当代码注释、配置注释和执行逻辑不一致时，以执行逻辑和模型文件元数据为准。

## 1. 先给出结论

### 1.1 网络输入、输出维度总表


| 任务            | ROS 2 节点/可执行文件               | 模型文件                                            | 固定网络输入维度                | 网络输出维度 | 主要 ROS 输入                                       |
| ------------- | ---------------------------- | ----------------------------------------------- | ----------------------- | ------ | ----------------------------------------------- |
| Transport     | `transport_pattern`          | `transport/best_actor.pt`                       | **11**                  | **2**  | `/global_info`、`/swarm_command`                 |
| Transport     | `transport_pattern_neat`     | `transport/transport_genome_package.pkl`        | **11**                  | **2**  | `/global_info`、`/swarm_command`                 |
| RoundFlocking | `roundflocking_pattern`      | `roundflocking/best_actor.pt`                   | **18** = 6 + 12 路 Lidar | **2**  | `/global_info`、`/<robot>/scan`、`/swarm_command` |
| RoundFlocking | `roundflocking_pattern_neat` | `roundflocking/flocking_genome_package_0.3.pkl` | **18** = 6 + 12 路 Lidar | **2**  | `/global_info`、`/<robot>/scan`、`/swarm_command` |
| Navigation    | `navigation_pattern`         | `navigation/best_actor.pt`                      | **18** = 6 + 12 路 Lidar | **2**  | `/global_info`、`/<robot>/scan`、`/swarm_command` |
| Navigation    | `navigation_pattern_neat`    | `navigation/navigation_genome_package.pkl`      | **18** = 6 + 12 路 Lidar | **2**  | `/global_info`、`/<robot>/scan`、`/swarm_command` |


模型文件本身也确认了上述维度：

- 3 个 `.pt` 的 `actor_spec` 分别为 `obs_dim = 11/18/18`、`action_dim = 2`；
- 3 个 `.pkl` 的 NEAT 配置分别为 `num_inputs = 11/18/18`、`num_outputs = 2`；
- 当前所有模型都是**有内部状态的 recurrent NEAT 网络**。`.pt` 并不是普通的全连接 PyTorch MLP，而是导出成 PyTorch checkpoint 的 `kind=neat_recurrent` 网络；原生 `.pkl` 节点使用 `neat.nn.RecurrentNetwork`。

因此，输入维度只回答了一半问题。实机端必须同时复现：

1. 每一维的排列顺序；
2. 坐标系（世界坐标还是机器人本体坐标）；
3. 向量方向（例如 `robot - target` 而不是 `target - robot`）；
4. 单位和归一化；
5. Lidar 的角度顺序和数值语义；
6. 10 Hz 控制节拍及 recurrent state 的生命周期。

**同为 11 维或 18 维，不代表 PT 与原生 NEAT 的观测可以互换。** 当前实现中，两者在 Transport 和 Navigation 的坐标表达上存在明显差异。

### 1.2 三类任务需要的物理信息


| 任务            | 机器人自身状态                   | 任务对象状态        | Lidar                              | 是否显式输入其他机器人状态                 |
| ------------- | ------------------------- | ------------- | ---------------------------------- | ----------------------------- |
| Transport     | 全局位置、平面速度、朝向（朝向用于转换或动作映射） | 包裹位置/速度、目标区位置 | 网络本身不需要                            | 否；其他机器人只通过共同影响包裹间接耦合          |
| RoundFlocking | 全局位置、平面速度、朝向              | 共同环绕目标的位置     | 12 个方向，训练输入上限仅 0.20 m              | 没有结构化邻居状态；实物 Lidar 会把邻居当作普通回波 |
| Navigation    | 全局位置、平面速度、朝向、机器人 ID       | 每台机器人对应目标的位置  | 12 个方向，PT 上限 0.35 m，NEAT 上限 0.20 m | 没有结构化邻居状态；邻居只能出现在 Lidar 中     |


这里的“全局位置”目前不是由 `odom` 自动提供，而是 Webots Supervisor 直接读取所有实体真实状态后组成 `/global_info`。实机部署必须替换这一个关键仿真特权接口。

## 2. 当前系统的数据流

```text
Webots Supervisor
  -> /global_info (std_msgs/String，JSON，全局机器人/物体真值)

Webots/实物 Lidar driver
  -> /robot_N/scan (sensor_msgs/LaserScan)

/swarm_command
  -> pattern node
       -> 网络观测 -> recurrent PT/NEAT -> 2 维动作 -> geometry_msgs/Twist
       -> /robot_N/drive_command
            -> HardwareProtectionLayer + /robot_N/range_data
            -> /robot_N/cmd_vel

Transport 是例外：配置中的 bypass_protection=true
  -> pattern node 直接发布 /robot_N/cmd_vel
```

公共 `MovementPattern` 根据 `bypass_protection` 选择 `/drive_command` 或 `/cmd_vel`，见 `[movement_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/movement_pattern.py#L21)`。`/swarm_command` 为 START 时才运行控制回调，否则持续发布零速度，见 `[abstract_pattern.py](../src/ros2swarm/ros2swarm/abstract_pattern.py#L28)`。

### 2.1 `/global_info` 的当前 JSON 契约

Webots Supervisor 发布的数据结构是：

```json
{
  "timestamp": 12.345,
  "robots": {
    "robot_0": {
      "position": {"x": 0.2, "y": -0.4, "z": 0.0},
      "velocity": {"x": 0.05, "y": 0.01},
      "orientation": [
        0.866, -0.5, 0.0,
        0.5,    0.866, 0.0,
        0.0,    0.0,   1.0
      ]
    }
  },
  "objects": {
    "transport_box": {
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "velocity": {"x": 0.0, "y": 0.0}
    },
    "target_zone": {
      "position": {"x": 1.0, "y": 0.0, "z": 0.0},
      "velocity": {"x": 0.0, "y": 0.0}
    }
  }
}
```

机器人朝向是**行优先 3x3 旋转矩阵**，节点按 `yaw = atan2(R10, R00)` 提取航向角。Webots 发布器用位置差分计算机器人和物体的全局 `vx, vy`，实现见 `[global_info_supervisor.py](../src/launch_webots/launch_webots/global_info_supervisor.py#L412)`。

实机兼容发布器至少必须保持下列约束：

- 机器人字典键必须和 ROS namespace 去掉开头 `/` 后完全相同，例如 `robot_0`；
- Navigation 从 namespace 最后一个下划线后的字符串提取整数 ID，并寻找 `navigation_target_<ID>`；
- 物体键必须匹配参数：`transport_box`、`target_zone`、`flocking_target` 或 `navigation_target_N`；
- 位置单位为米，速度单位为 m/s，yaw 为绕 `+z` 的右手系角度；
- 所有机器人、包裹和目标必须位于同一个稳定的世界坐标系中。



## 3. 统一记号和坐标变换

本文使用：

- `p_r = (x_r, y_r)`：机器人在世界坐标系的位置；
- `v_r = (vx_r, vy_r)`：机器人在世界坐标系的平面速度；
- `p_b, v_b`：运输包裹的世界位置和速度；
- `p_t`：目标点/目标区的世界位置；
- `s_p = world_scale`：位置缩放，当前所有配置均为 `1.75`；
- `R(-yaw)`：将世界坐标向量旋转到机器人本体坐标；
- 本体坐标约定为 `+x` 向前、`+y` 向左；
- 表中索引从 0 开始。

当前代码中的相对向量大多采用：

```text
robot_to_box = p_b - p_r      # 从机器人指向包裹
box_to_target = p_b - p_t     # 从目标指向包裹，名字容易误解
robot_to_target = p_r - p_t   # 从目标指向机器人，名字也容易误解
```

不要根据变量的自然语言名称擅自改变符号。

## 4. Transport 输入详解

Transport 不读取 Lidar。它依赖自身状态、包裹状态和目标区状态，固定输入 11 维。

### 4.1 `transport_pattern`：PT 版，11 维

观测由 `[vmas_transport_adapter.py](../src/ros2swarm/ros2swarm/vmas_transport_adapter.py#L39)` 构造：


| 索引  | 实际输入              | 坐标系 | 归一化/范围                          | 含义            |
| --- | ----------------- | --- | ------------------------------- | ------------- |
| 0   | `x_r / s_p`       | 世界  | `/ world_scale`                 | 机器人绝对 x       |
| 1   | `y_r / s_p`       | 世界  | `/ world_scale`                 | 机器人绝对 y       |
| 2   | `vx_r / s_v`      | 世界  | `velocity_scale<=0` 时 `s_v=s_p` | 机器人世界 x 速度    |
| 3   | `vy_r / s_v`      | 世界  | 同上                              | 机器人世界 y 速度    |
| 4   | `(x_b-x_t) / s_p` | 世界  | `/ world_scale`                 | 目标指向包裹的 x 分量  |
| 5   | `(y_b-y_t) / s_p` | 世界  | `/ world_scale`                 | 目标指向包裹的 y 分量  |
| 6   | `(x_b-x_r) / s_p` | 世界  | `/ world_scale`                 | 机器人指向包裹的 x 分量 |
| 7   | `(y_b-y_r) / s_p` | 世界  | `/ world_scale`                 | 机器人指向包裹的 y 分量 |
| 8   | `vx_b / s_v`      | 世界  | `/ velocity scale`              | 包裹世界 x 速度     |
| 9   | `vy_b / s_v`      | 世界  | 同上                              | 包裹世界 y 速度     |
| 10  | `on_goal`         | 标量  | `0.0` 或 `1.0`                   | `             |


当前 e-puck YAML 设置 `world_scale=1.75`、`velocity_scale=0.0`，所以位置和速度都除以 1.75。注意这里**没有把速度或相对向量旋转到本体坐标系**。

PT 网络输出是归一化二维平面动作 `[a_x, a_y]`。默认 `action_mapping=heading` 将它解释为**世界坐标期望移动方向**，结合 `/global_info` 中的 yaw 转为差速底盘 `linear.x/angular.z`，见 `[vmas_transport_adapter.py](../src/ros2swarm/ros2swarm/vmas_transport_adapter.py#L129)`。

### 4.2 `transport_pattern_neat`：原生 NEAT 版，11 维

原生 NEAT 使用相同字段数量，但从第 2 维开始大量使用本体坐标，构造见 `[transport_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern_neat.py#L182)`：


| 索引  | 实际输入                          | 坐标系 | 归一化/范围                 |
| --- | ----------------------------- | --- | ---------------------- |
| 0   | `x_r / s_p`                   | 世界  | `/ world_scale`        |
| 1   | `y_r / s_p`                   | 世界  | `/ world_scale`        |
| 2   | `R(-yaw) v_r` 的 x             | 本体  | **不再除以** `world_scale` |
| 3   | `R(-yaw) v_r` 的 y             | 本体  | 不缩放                    |
| 4   | `R(-yaw)(p_b-p_t)` 的 x `/s_p` | 本体  | `/ world_scale`        |
| 5   | `R(-yaw)(p_b-p_t)` 的 y `/s_p` | 本体  | `/ world_scale`        |
| 6   | `R(-yaw)(p_b-p_r)` 的 x `/s_p` | 本体  | `/ world_scale`        |
| 7   | `R(-yaw)(p_b-p_r)` 的 y `/s_p` | 本体  | `/ world_scale`        |
| 8   | `R(-yaw) v_b` 的 x             | 本体  | 不缩放                    |
| 9   | `R(-yaw) v_b` 的 y             | 本体  | 不缩放                    |
| 10  | `on_goal`                     | 标量  | `0.0` 或 `1.0`          |


网络的两个 sigmoid 输出直接映射为：

```text
linear.x = (output[0] * 2 - 1) * max_linear_velocity
angular.z = (output[1] * 2 - 1) * max_angular_velocity
```

这和 PT 版的“二维世界平面动作再做航向转换”不是同一种输出语义，见 `[transport_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern_neat.py#L312)`。

### 4.3 Transport 实机所需传感与状态估计

最低需求：

1. 每台机器人 `x, y, yaw, vx, vy`；
2. 包裹 `x, y, vx, vy`；
3. 目标区 `x, y`；
4. 全部状态处于同一全局坐标系；
5. 10 Hz 左右稳定更新，且能够检测状态过期。

最接近 Webots 真值的实机方案是 OptiTrack/Vicon 或顶视相机 + AprilTag/Aruco：每台机器人、包裹各贴一个标记，目标区使用已标定的静态坐标。只依靠轮式里程计无法稳定得到包裹状态，多机器人地图原点也难以天然一致。

## 5. RoundFlocking 输入详解

两个 RoundFlocking 模型均固定需要 18 维：前 6 维运动/目标状态 + 12 维 Lidar。

### 5.1 `roundflocking_pattern`：PT 版，18 维

输入由节点内部 `_get_observation()` 构造，见 `[roundflocking_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern.py#L233)`：


| 索引    | 实际输入                          | 坐标系     | 当前默认处理                                 |
| ----- | ----------------------------- | ------- | -------------------------------------- |
| 0     | `x_r / s_p`                   | 世界      | `target_centric_position=false` 时为绝对 x |
| 1     | `y_r / s_p`                   | 世界      | 绝对 y                                   |
| 2     | `R(-yaw) v_r` 的 x `/s_v`      | 本体      | `velocity_scale=1.0`                   |
| 3     | `R(-yaw) v_r` 的 y `/s_v`      | 本体      | `velocity_scale=1.0`                   |
| 4     | `R(-yaw)(p_r-p_t)` 的 x `/s_p` | 本体      | 目标指向机器人的相对 x                           |
| 5     | `R(-yaw)(p_r-p_t)` 的 y `/s_p` | 本体      | 目标指向机器人的相对 y                           |
| 6..17 | 12 路 Lidar 原始距离               | 雷达/本体方向 | 每维裁剪到 `[0, 0.20]` m                    |


若把 `target_centric_position` 改为 true，索引 0、1 会从机器人绝对位置变成 `(p_r-p_t)/s_p`。当前 e-puck 配置为 false，不能在不重新验证模型的情况下随意修改。

Lidar 当前按数组长度均分 12 段，并取每段中间的一束；它**忽略** `LaserScan.angle_min` **和** `angle_increment`，见 `[roundflocking_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern.py#L178)`。因此它隐含假设 `ranges[0]` 对应训练侧的机器人 `+x` 方向，并且后续数组方向与训练一致。

PT 版的实际动作还不是纯网络输出。默认启用环半径稳定器：

```text
最终二维动作 = 0.25 * actor_action + 0.75 * ring_action
target_radius = 0.6 m
ring_radial_gain = 6.0
ring_tangential_gain = 1.0
```

实现见 `[roundflocking_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern.py#L321)`。所以 PT 实机表现高度依赖目标位置和 yaw 的准确度；它并非只部署 checkpoint 就能复现。

### 5.2 `roundflocking_pattern_neat`：原生 NEAT 版，18 维

输入顺序为：


| 索引    | 实际输入                          | 坐标系     | 当前默认处理                                         |
| ----- | ----------------------------- | ------- | ---------------------------------------------- |
| 0     | `x_r / s_p`                   | 世界      | 绝对 x                                           |
| 1     | `y_r / s_p`                   | 世界      | 绝对 y                                           |
| 2     | `R(-yaw) v_r` 的 x             | 本体      | 不缩放                                            |
| 3     | `R(-yaw) v_r` 的 y             | 本体      | 不缩放                                            |
| 4     | `R(-yaw)(p_r-p_t)` 的 x `/s_p` | 本体      | 目标指向机器人                                        |
| 5     | `R(-yaw)(p_r-p_t)` 的 y `/s_p` | 本体      | 同上                                             |
| 6..17 | 12 路 Lidar 原始距离               | 按扫描数组顺序 | `[0, lidar_input_max_range]`，当前为 `[0, 0.20]` m |


实现见 `[roundflocking_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern_neat.py#L251)`。当前代码已明确区分“物理雷达量程”和“网络训练输入量程”：e-puck2 扩展雷达即使具有更大量程，送入网络的最大值仍是 0.20 m，见 `[roundflocking_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern_neat.py#L45)`。

其 Lidar 同样按数组索引分段取中间束，不使用角度元数据。两个网络输出被解释为本体平面动作 `[vx, vy]`，再通过期望方向角转成差速底盘命令。原生 NEAT 版没有 PT 版的环半径外环。

### 5.3 RoundFlocking 实机所需传感与状态估计

最低需求：

1. 每台机器人的 `x, y, yaw, vx, vy`；
2. 公共 `flocking_target` 的全局 `x, y`；
3. 每台机器人的 360 度 Lidar，最终严格生成 12 维训练语义输入；
4. 若使用 PT 版，目标距离误差应明显小于环半径控制允许误差。

模型没有直接输入邻居 ID、邻居速度或通信消息。实物 Lidar 会同时看到墙、机器人和其他物体；而训练环境可能对可被 Lidar 感知的实体做过过滤。因此“相同 12 路距离”仍可能存在显著 sim-to-real 分布差异。

## 6. Navigation 输入详解

两个 Navigation 模型的 checkpoint 都固定为 18 输入、2 输出，但 PT 与原生 NEAT 的坐标系和 Lidar 上限不同。

### 6.1 `navigation_pattern`：PT 版，18 维

观测由 `[vmas_planar_adapter.py](../src/ros2swarm/ros2swarm/vmas_planar_adapter.py#L26)` 构造：


| 索引    | 实际输入                              | 坐标系                | 当前默认处理                         |
| ----- | --------------------------------- | ------------------ | ------------------------------ |
| 0     | `x_r / s_p`                       | 世界                 | `/ 1.75`                       |
| 1     | `y_r / s_p`                       | 世界                 | `/ 1.75`                       |
| 2     | `vx_r / s_v`                      | 世界                 | `velocity_scale<=0`，所以 `/1.75` |
| 3     | `vy_r / s_v`                      | 世界                 | 同上                             |
| 4     | `(x_r-x_t) / s_p`                 | 世界                 | 目标指向机器人                        |
| 5     | `(y_r-y_t) / s_p`                 | 世界                 | 同上                             |
| 6..17 | `max_range - sampled_distance[i]` | 机器人 0、30、...、330 度 | 当前 `max_range=0.35` m          |


PT Navigation 的 Lidar 不是原始距离，而是“障碍接近量”：

```text
0.00 代表该方向没有 0.35 m 内的障碍
0.35 代表障碍距离接近 0 m
```

这里的采样器是六个节点中较稳健的一个：它使用 `angle_min` 和 `angle_increment`，主动寻找机器人 `+x` 起始的 0、30、...、330 度方向，见 `[vmas_planar_adapter.py](../src/ros2swarm/ros2swarm/vmas_planar_adapter.py#L79)`。

网络输出 `[a_x, a_y]` 按世界平面向量处理，结合 yaw 经默认 `heading` 映射转成差速速度，见 `[navigation_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/navigation_pattern.py#L183)`。

### 6.2 `navigation_pattern_neat`：原生 NEAT 版，18 维

输入构造见 `[navigation_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/navigation_pattern_neat.py#L243)`：


| 索引    | 实际输入                              | 坐标系     | 当前默认处理                |
| ----- | --------------------------------- | ------- | --------------------- |
| 0     | `x_r / s_p`                       | 世界      | `/ 1.75`              |
| 1     | `y_r / s_p`                       | 世界      | `/ 1.75`              |
| 2     | `R(-yaw) v_r` 的 x                 | 本体      | 不缩放                   |
| 3     | `R(-yaw) v_r` 的 y                 | 本体      | 不缩放                   |
| 4     | `R(-yaw)(p_r-p_t)` 的 x `/s_p`     | 本体      | 目标指向机器人               |
| 5     | `R(-yaw)(p_r-p_t)` 的 y `/s_p`     | 本体      | 同上                    |
| 6..17 | `max_range - sampled_distance[i]` | 按扫描数组顺序 | 当前 `max_range=0.20` m |


该节点按数组均分扇区并取中间束，忽略 Lidar 的角度元数据。输出 `[vx, vy]` 被当作本体平面期望方向，再转成差速 `Twist`。

`collisions=true` 是该模型的硬约束，不只是功能开关。当前 `.pkl` 固定需要 18 维；若设为 false，节点只构造 6 维却仍调用 18 输入网络，推理将失败。

### 6.3 Navigation 实机所需传感与状态估计

最低需求：

1. 每台机器人统一全局坐标下的 `x, y, yaw, vx, vy`；
2. `navigation_target_<ID>` 的全局 `x, y`；
3. 360 度 Lidar 和正确的 12 方向采样；
4. namespace 必须以可解析整数结尾，例如 `/robot_0`；
5. 局部避障不能改变 Lidar 的数值定义。

目标可以是配置中的静态 map 坐标，不一定需要实时视觉跟踪；但地图坐标必须与机器人定位坐标完全对齐。

## 7. 配置参数对比



### 7.1 当前速度上限


| 平台/任务  | Transport PT/NEAT       | RoundFlocking PT/NEAT   | Navigation PT/NEAT      |
| ------ | ----------------------- | ----------------------- | ----------------------- |
| e-puck | `0.12 m/s`, `1.0 rad/s` | `0.12 m/s`, `1.0 rad/s` | `0.12 m/s`, `1.0 rad/s` |


这些是节点输出限幅，不是对每个轮电机的最终强制限幅。HardwareProtectionLayer 只在生成避障方向时调用轮速联合限制；障碍为空时会原样转发神经网络命令，见 `[hardware_protection_layer.py](../src/ros2swarm/ros2swarm/hardware_protection_layer.py#L163)`。实机底层仍需独立的轮速/电流/加速度限制和命令超时。

### 7.2 当前 Lidar 网络语义


| 控制器                | 维度  | 网络截断上限 | 送入网络的值   | 是否使用 `LaserScan` 角度元数据 |
| ------------------ | --- | ------ | -------- | ---------------------- |
| RoundFlocking PT   | 12  | 0.20 m | 原始距离 `d` | 否                      |
| RoundFlocking NEAT | 12  | 0.20 m | 原始距离 `d` | 否                      |
| Navigation PT      | 12  | 0.35 m | `0.35-d` | 是                      |
| Navigation NEAT    | 12  | 0.20 m | `0.20-d` | 否                      |


不得把物理雷达的 `range_max` 直接替换上述训练上限，也不得把原始距离和反向距离混用。

## 8. 从 Webots 到实机：必须替换的接口



### 8.1 用实机状态桥替代 Webots Supervisor

推荐保留现有控制节点，先实现一个 `real_global_info_bridge`：

```text
OptiTrack/Vicon/顶视视觉/融合定位
  -> 每台机器人 PoseStamped/Odometry
包裹视觉标记
  -> 包裹 PoseStamped
静态任务配置
  -> target_zone/flocking_target/navigation_target_N
状态桥
  -> 兼容 JSON /global_info
```

状态桥需要完成：

- 建立唯一的 `world` 坐标原点和轴方向；
- 把所有定位源变换到 `world`；
- 用滤波后的差分或状态估计器生成世界 `vx, vy`；
- 把 quaternion yaw 转为当前代码期望的行优先旋转矩阵；
- 对定位丢失、遮挡、跳变设置 validity 和超时；
- 保持 `robot_N` 和任务对象命名契约；
- 以至少 10 Hz 发布，最好高于控制频率并带有稳定时间戳。

只修改 `world_scale` 并不能自动解决任意场地迁移。模型输入含绝对 `x,y`，所以还必须让实机场地中心、轴方向和训练场景一致。若实物场地尺寸不是 3.5 m × 3.5 m，可在状态桥中定义一个明确的仿真等效坐标变换：

```text
p_model = A * (p_physical - p_world_origin)
v_model = A * v_physical
```

同一个线性变换必须用于机器人、包裹、目标和速度，不能只缩放机器人位置。

### 8.2 增加统一的 Lidar 适配层

实机雷达常见差异包括：

- `angle_min` 可能为 `-pi`、`0` 或其他值；
- 数组可能顺时针或逆时针；
- 雷达 `+x` 与 `base_link +x` 可能有安装偏角；
- `0.0`、`NaN`、`Inf` 对不同驱动可能分别表示无回波或非法回波；
- 机器人外壳、支架和运输包裹可能产生自反射；
- 实际盲区和噪声远大于 Webots。

部署前应把四个需要 Lidar 的节点统一到一个角度感知的采样函数，并通过 TF 把每个期望射线定义为 `base_link` 的 0、30、...、330 度。需要明确规定：

```text
非法/无回波 -> 网络训练上限（表示该方向在关注距离内无障碍）
有效回波     -> clip(distance, 0, network_lidar_max)
Navigation   -> network_lidar_max - clipped_distance
RoundFlocking-> clipped_distance
```

是否用单束中值、扇区最小值或百分位数必须通过仿真录包和实机录包对比决定。当前网络用“每扇区中间单束”，直接改成最小值会更保守，但也会改变训练分布。

### 8.3 底盘和动作适配

当前所有控制节点发布 `geometry_msgs/Twist`。实机要确认：

- 驱动订阅的是 `/robot_N/cmd_vel`，消息类型是 `Twist` 而不是 `TwistStamped`；
- `linear.x > 0` 确实向前，`angular.z > 0` 确实逆时针；
- 轮距、轮径和电机上限采用实测值；
- `v_left = v - omega*L/2`、`v_right = v + omega*L/2` 均不超限；
- 底盘有 200--500 ms 的命令 watchdog，控制节点或网络断开时自动停车；
- 加速度和 jerk 受限，避免仿真中瞬时动作在实物上打滑；
- recurrent 网络的 10 Hz 输出与底盘控制器频率之间有确定的保持策略。



## 9. 当前实现中必须正视的风险

以下不是泛化建议，而是当前代码的具体行为。

### 9.1 P0：Transport 绕过 HardwareProtectionLayer

e-puck Transport YAML 设置了 `bypass_protection: true`，因为机器人需要接触包裹；实机入口会覆盖该参数并让新的 safety supervisor 独占最终 `cmd_vel`。

这意味着 Transport 神经网络命令直接进入 `/cmd_vel`。实机至少需要独立于行为控制器的：急停、保险杠/碰撞条、场地边界、人员检测、速度/加速度/轮速限制、定位失效停车和通信失效停车。安全层应允许“接触被跟踪包裹”，而不是关闭全部碰撞保护。

### 9.2 P0：非 Transport 模式的 STOP 零速度不一定能穿过 HPL

行为节点在 `/swarm_command` 为 STOP 时会向 `/drive_command` 发布零 `Twist`；但是 HardwareProtectionLayer 的 `command_callback` 和 `range_data_callback` 自己也被 `swarm_command_controlled(...)` 包装。HPL 收到 STOP 后会跳过这些回调，而且没有覆写 `swarm_command_false_case()` 向 `/cmd_vel` 主动发布零速度。因此，Navigation/RoundFlocking 的行为节点虽然在发布停止命令，HPL 却可能不把它转发到底盘。

此外，HPL 在尚未收到任何 range data 时采用 fail-open：`vector_calc()` 返回“不需要避障”，原始运动命令会直接通过；range data 过期后也没有超时判断。实机前应：

- 让 HPL 收到 STOP 时立即向 `/cmd_vel` 发布零 `Twist`；
- 给 `/drive_command` 和 `/range_data` 都设置超时，任一过期即停车；
- 在底盘驱动中保留更低一层的独立命令 watchdog；
- 增加测试，覆盖 START 运动、STOP 停车、Lidar 未就绪和 Lidar 中断。

相关包装逻辑见 `[hardware_protection_layer.py](../src/ros2swarm/ros2swarm/hardware_protection_layer.py#L81)` 和 `[abstract_pattern.py](../src/ros2swarm/ros2swarm/abstract_pattern.py#L47)`。

### 9.3 P0：原生 Transport/Navigation 在输入缺失时使用随机观测

`transport_pattern_neat` 和 `navigation_pattern_neat` 在等待 `/global_info`、等待 Lidar 或部分查找失败时会生成 `[-1,1]` 随机向量并继续推理。Navigation 相关逻辑见 `[navigation_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/navigation_pattern_neat.py#L256)`，Transport 相关逻辑见 `[transport_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern_neat.py#L200)`。

这在仿真中可用于早期测试，但不适用于实机：不仅会产生随机动作，还会污染 recurrent state。实机前应改为“观测不完整、非法或过期时发布零速度，且不调用网络”。RoundFlocking NEAT 当前已经采用这种停车行为。

`navigation_pattern_neat` 还有一个相关问题：网络未加载或推理抛出异常时只记录日志，不主动发布零速度。若底盘没有 watchdog，上一条命令可能继续生效。该分支也应统一为 fail-stop。

### 9.4 P0：没有观测新鲜度和跨传感器同步检查

各节点只缓存“最近一次” `/global_info` 和 `/scan`，不检查年龄；`/global_info` 内的 `timestamp` 也没有被控制节点使用。定位发布器死亡后，机器人可能继续基于旧状态动作。

应为每个输入保存 ROS 接收时间/源时间，并设置：

- `global_info_max_age`，建议先从 0.2--0.3 s 验证；
- `scan_max_age`，建议不大于 2 个扫描周期；
- 机器人状态、目标状态、包裹状态的 validity；
- 位置跳变、速度异常、非有限数检查；
- 任一条件失败时的零速度和 recurrent state 处理策略。



### 9.5 P0：三个节点的 Lidar 扇区方向可能在实机上错位

RoundFlocking PT、RoundFlocking NEAT、Navigation NEAT 只按数组索引取样，忽略 `angle_min/angle_increment`。换一款驱动后，网络看到的“前方”可能实际是后方，或者左右镜像。必须用单墙测试逐一验证第 6--17 维的方向。

### 9.6 P1：Navigation 的 `goal_threshold` 当前不负责停车

两个 Navigation 节点都声明并读取了 `goal_threshold`，但控制回调没有使用它判断到达并停车。它目前只是无效参数。实机任务管理器需要单独判断成功并发 STOP/零速度，或补上带迟滞和连续帧确认的到达逻辑。

### 9.7 P1：recurrent state 没有随任务 START/STOP 重置

所有 checkpoint 都有 recurrent state。当前网络只在节点加载时 reset；收到 `/swarm_command` STOP 再 START 时不会开始一个全新 episode。Transport 的 `_goal_reached` 还会永久锁存到节点重启。

应定义清楚：

- 新任务 START 是否重置网络；
- 短暂传感器丢失是保持 state、清零 state 还是重启 episode；
- STOP 后再次 START 是否清除 `_goal_reached`；
- 多机器人集中推理时，每个机器人必须有独立 recurrent state。



### 9.8 e-puck2 实机 launch

e-puck2 统一使用：

- `epuck2_physical.launch.py`：底盘、360 度 Lidar、策略适配和安全链；
- `epuck2_overhead_tracking.launch.py`：二维码检测和全局状态；
- `epuck2_mock_chain.launch.py`：无硬件闭环测试。

这些入口显式使用 `use_sim_time=false`，并能拆分为中央策略与 Pi-puck 本地驱动/安全。Webots 保持原有独立启动路径。

### 9.9 P1：运行依赖没有在包元数据中声明

`[setup.py](../src/ros2swarm/setup.py#L101)` 的 `install_requires` 只有 `setuptools`，`[package.xml](../src/ros2swarm/package.xml#L1)` 也缺少运行依赖。实机镜像必须显式安装并锁定至少：

- 与目标 ROS 2 发行版匹配的 `rclpy`、消息包、launch 依赖；
- `numpy`；
- PT 节点所需的 PyTorch；
- 原生 NEAT 节点所需的 `neat-python`；
- `ament_index_python` 和本仓库 `communication_interfaces`；
- 底盘、Lidar、TF 和定位系统驱动。

ARM 平台尤其要提前验证 PyTorch wheel、启动时间、内存和 CPU 占用。checkpoint 很小，不代表 PyTorch runtime 本身很小。

### 9.10 P1：模型固定维度与参数表面可配置之间存在冲突

`n_lidar_rays` 看似可配置，但当前 RoundFlocking/Navigation 模型文件固定只接受 18 维，因此实际必须保持 12。PT wrapper 会明确报维度不匹配；原生 NEAT 也无法接受任意改变后的长度。

`collisions=false` 对 PT Navigation 仍会补 12 个最大距离，维度保持 18；对 NEAT Navigation 则会只构造 6 维并失败。实机配置应将这些模型结构参数视为常量。

## 10. 推荐的实机系统架构

建议第一版采用“中央状态估计 + 每机器人本地推理和底盘安全”的架构：

```text
                         ┌──────────────────────────────┐
定位/顶视视觉/动捕 ──────>│ real_global_info_bridge      │
包裹跟踪 ────────────────>│ 统一坐标、速度滤波、有效性    │
静态目标配置 ────────────>│ -> /global_info              │
                         └──────────────┬───────────────┘
                                        │ DDS/Wi-Fi
           ┌────────────────────────────┼────────────────────────────┐
           │ robot_0                    │ robot_1                    │
           │                            │                            │
Lidar ──> scan adapter ──> /scan        │                            │
           │                            │                            │
           ├─ pattern + 独立网络状态     ├─ pattern + 独立网络状态     │
           │       -> /drive_command    │       -> /drive_command    │
           │                            │                            │
           ├─ safety supervisor/HPL     ├─ safety supervisor/HPL     │
急停/保险杠/定位健康 ─────>│       -> /cmd_vel             │                            │
           │                            │                            │
           └─ base driver + watchdog    └─ base driver + watchdog    │
```

Transport 也应经过 `safety supervisor`，只是该安全层需要识别允许接触的包裹，不能直接使用“任何近距离物体都避开”的普通 HPL。

为什么先保留 `/global_info`：它能最小化控制器改动，便于将同一包 Webots 数据和实机数据离线回放比较。长期应改为有 Header、类型明确的自定义消息，而不是无 schema 的 JSON `String`；迁移期间可同时发布 typed state 和兼容 `/global_info`。

## 11. 推荐部署步骤



### 阶段 0：冻结仿真基线

1. 为每种任务和 PT/NEAT 各录一包成功 Webots 数据；
2. 记录原始 `/global_info`、每台 `/scan`、`/drive_command`、`/cmd_vel`、`/swarm_command`；
3. 临时增加观测向量和网络二维输出的诊断话题或日志；
4. 统计每一维的 min/max/均值/标准差和控制延迟；
5. 保存模型 SHA-256、配置和代码 commit，避免实机测试中模型悄然变化。



### 阶段 1：只验证底盘，不运行网络

1. 设置实机 `use_sim_time=false`；
2. 验证 namespace、`/scan`、`/cmd_vel`、TF 和底盘 watchdog；
3. 用极低速手动 Twist 验证线速度/角速度正负号；
4. 测量实际轮距、轮径、最大稳定轮速、制动距离；
5. 验证物理急停和远程 STOP 在网络中断时仍有效。



### 阶段 2：验证 `/global_info`，网络只做 shadow inference

1. 机器人架空或使能电机断开；
2. 发布实机状态并构造模型观测，但不把命令送到底盘；
3. 对照实物真实方向检查所有相对向量的符号；
4. 原地旋转机器人，确认 yaw、body velocity 和 body relative vector 变化正确；
5. 在 12 个方向放置单一障碍，验证 Lidar 维度索引；
6. 比较实机观测分布和阶段 0 仿真分布，超出训练范围时先定位原因，不直接裁剪掩盖问题。



### 阶段 3：单机器人、低速、空旷场地

1. 将速度上限降至当前配置的 20%--30%；
2. 在软边界和物理隔离场地中运行；
3. 人员手持急停，先验证输入丢失和节点崩溃停车；
4. 分别测试目标在前后左右、机器人不同初始 yaw；
5. 检查 recurrent state 在 STOP/START 后是否符合预期。



### 阶段 4：按任务逐步增加复杂度

- Navigation：先单机器人静态障碍，再动态障碍，最后多机器人；
- RoundFlocking：先单机器人绕目标，再 2 台，逐步增加数量；
- Transport：先单机器人/轻质假包裹，再多机器人；最后才使用真实负载，并监控电机电流、打滑和包裹跟踪遮挡。



### 阶段 5：必要时做再训练或适配训练

如果真实观测分布明显不同，不应无限调整 `world_scale` 或速度增益。更可靠的做法是用实测数据标定仿真，并在训练时加入：

- 定位噪声、延迟、丢帧和速度估计噪声；
- Lidar 盲区、离群点、角度偏差和不同机器人外形；
- 轮胎打滑、电机死区、动作延迟和速度饱和；
- 包裹质量、摩擦、碰撞和视觉遮挡随机化；
- 不同场地尺寸、坐标原点和目标位置；
- 安全层对动作的实际修改。



## 12. 上车前验收清单



### 12.1 网络与观测

- [ ] Transport 每次严格 11 个有限数；RoundFlocking/Navigation 每次严格 18 个有限数；
- [ ] `n_lidar_rays` 固定为 12，NEAT Navigation 的 `collisions=true`；
- [ ] PT/NEAT 使用各自正确的世界/本体坐标表达，没有共用错误的 adapter；
- [ ] `p_b-p_t`、`p_b-p_r`、`p_r-p_t` 的符号经过实物位置测试；
- [ ] `world_scale=1.75` 对应明确的实物到模型坐标标定；
- [ ] 机器人、包裹、目标速度单位为 m/s，并经过带宽合适的滤波；
- [ ] 模型加载后实际报告的输入/输出维度和文档一致；
- [ ] 每个机器人拥有独立 recurrent state，新任务能按定义 reset。



### 12.2 Lidar

- [ ] `/robot_N/scan` 为 360 度有效扫描，frame 与 `base_link` 标定正确；
- [ ] 第一个网络 ray 对应机器人正前方，后续每 30 度方向和旋转方向正确；
- [ ] `0/NaN/Inf/range_min/range_max` 的含义按驱动实际行为处理；
- [ ] RoundFlocking 输入为 `[0,0.20]` m 原始距离；
- [ ] PT Navigation 输入为 `0.35-d`，NEAT Navigation 输入为 `0.20-d`；
- [ ] 雷达不会持续看到机器人自身外壳或支架；
- [ ] 实物 Lidar 频率、延迟和丢包在安全阈值内。



### 12.3 ROS 2、时钟和通信

- [ ] 所有实机节点 `use_sim_time=false`，没有等待不存在的 `/clock`；
- [ ] 所有机器人使用一致的 `ROS_DOMAIN_ID`，namespace 唯一；
- [ ] `/global_info` 发布频率和最大时延满足 10 Hz 控制；
- [ ] Wi-Fi 断开、中央状态桥退出、Lidar 退出时机器人均在规定时间内停车；
- [ ] QoS 经实网测试，没有把高丢包误当正常状态；
- [ ] CPU/内存/温度满足 PT 或 NEAT runtime 长时间运行。



### 12.4 底盘和安全

- [ ] `/cmd_vel` 类型和 topic 与底盘驱动完全匹配；
- [ ] 正负方向、轮距、轮径、轮速上限和加速度上限已经实测；
- [ ] 底盘控制器有独立命令 watchdog；
- [ ] 物理急停不依赖 ROS 2 或 Wi-Fi；
- [ ] Navigation/RoundFlocking 的 HPL 和任务节点同时正常工作；
- [ ] Transport 虽允许接触包裹，但人员、墙体、越界和定位失效仍能触发安全停车；
- [ ] Navigation 由任务管理器明确执行到达停车，因为当前节点不使用 `goal_threshold` 停车。



## 13. 建议的代码改造优先级



### 第一优先级：没有这些不要驱动实物

1. 将 Transport/Navigation NEAT 的随机缺失观测改为停车且不推进 recurrent state；
2. 修复 HPL 的 STOP、输入未就绪和输入超时行为，保证任何异常都能把零速度送到 `/cmd_vel`；
3. 给 `/global_info` 和 `/scan` 增加 age、validity、有限数和跳变检查；
4. 给所有底盘命令增加独立 watchdog、急停和最终轮速/加速度限制；
5. 为 Transport 实现“允许包裹接触但仍保护其他碰撞”的安全 supervisor；
6. 将三个索引式 Lidar sampler 改为统一的、基于角度和 TF 的采样器；
7. 修复实机 launch 的 `use_sim_time`、`sensor_type` 转发和驱动启动路径。



### 第二优先级：保证任务可以重复和诊断

1. 在任务 START/STOP 边界管理网络 reset 和 `_goal_reached`；
2. 为 Navigation 实现可靠的到达停车和迟滞；
3. 发布观测向量、网络原始输出、最终 Twist、输入 age 和安全状态诊断；
4. 将固定模型结构参数与普通调参分开，启动时断言 11/18 维；
5. 用 typed message 替代 JSON，同时保留兼容桥；
6. 补全 ROS/Python 运行依赖并建立可复现的 ARM/x86 镜像。



### 第三优先级：提高 sim-to-real 表现

1. 用实物录包回放测试所有 observation builder；
2. 标定坐标、速度滤波、Lidar 噪声和动作延迟；
3. 在 Webots/训练环境注入实测噪声与动力学；
4. 若仍有显著域差异，重新训练或微调模型，不用安全层替代策略能力。



## 14. 最终判断

这 6 个节点的网络维度本身很小，算力通常不是最难的问题。真正的部署难点是：当前成功依赖 Webots Supervisor 提供低延迟、无漂移、统一坐标的机器人/包裹/目标真值，而实机必须用定位、目标标定和包裹跟踪重建完全相同的观测语义。

推荐先实现兼容 `/global_info` 的实机状态桥和统一 Lidar adapter，以 shadow inference 验证每一维，再接入低速底盘。对于当前代码，随机缺失观测、无数据超时、Transport 绕过保护、Lidar 方向假设和实机 launch 参数问题都应在首次带电自主运行前解决。

## 15. 关键源码索引

- PT checkpoint 载入、维度校验和 recurrent runtime：`[vmas_actor_policy.py](../src/ros2swarm/ros2swarm/vmas_actor_policy.py#L24)`
- Transport PT 观测与动作适配：`[vmas_transport_adapter.py](../src/ros2swarm/ros2swarm/vmas_transport_adapter.py#L39)`
- Navigation PT 观测和角度感知 Lidar 采样：`[vmas_planar_adapter.py](../src/ros2swarm/ros2swarm/vmas_planar_adapter.py#L26)`
- Transport PT 节点：`[transport_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern.py#L27)`
- Transport NEAT 节点：`[transport_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern_neat.py#L37)`
- RoundFlocking PT 节点：`[roundflocking_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern.py#L42)`
- RoundFlocking NEAT 节点：`[roundflocking_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern_neat.py#L80)`
- Navigation PT 节点：`[navigation_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/navigation_pattern.py#L34)`
- Navigation NEAT 节点：`[navigation_pattern_neat.py](../src/ros2swarm/ros2swarm/movement_pattern/basic/navigation_pattern_neat.py#L38)`
- Webots `/global_info` 生成：`[global_info_supervisor.py](../src/launch_webots/launch_webots/global_info_supervisor.py#L412)`
- 行为输出 topic 选择：`[movement_pattern.py](../src/ros2swarm/ros2swarm/movement_pattern/movement_pattern.py#L18)`
- HardwareProtectionLayer：`[hardware_protection_layer.py](../src/ros2swarm/ros2swarm/hardware_protection_layer.py#L49)`
- e-puck2 实机 bringup：`[epuck2_physical.launch.py](../src/ros2swarm/launch/epuck2_physical.launch.py)`、`[epuck2_overhead_tracking.launch.py](../src/ros2swarm/launch/epuck2_overhead_tracking.launch.py)`
- 无硬件闭环：`[epuck2_mock_chain.launch.py](../src/ros2swarm/launch/epuck2_mock_chain.launch.py)`
