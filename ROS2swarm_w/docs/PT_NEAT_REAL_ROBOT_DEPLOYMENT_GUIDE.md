# PT/NEAT 策略运行契约

本文只记录 Navigation、RoundFlocking、Transport 的模型输入输出契约。实物安装和启动以 [EPUCK2_PI_PUCK_END_TO_END_DEPLOYMENT.md](EPUCK2_PI_PUCK_END_TO_END_DEPLOYMENT.md) 为准。

## 1. 模型总表

| 任务 | PT 节点 | NEAT 节点 | 输入维度 | 输出维度 | ROS 输入 |
| --- | --- | --- | --- | --- | --- |
| Transport | `transport_pattern` | `transport_pattern_neat` | 11 | 2 | `/global_info`、`/swarm_command` |
| RoundFlocking | `roundflocking_pattern` | `roundflocking_pattern_neat` | 18 | 2 | `/global_info`、`/robot_N/scan`、`/swarm_command` |
| Navigation | `navigation_pattern` | `navigation_pattern_neat` | 18 | 2 | `/global_info`、`/robot_N/scan`、`/swarm_command` |

实物优先使用 PT：

```text
transport_pattern
roundflocking_pattern
navigation_pattern
```

仓库中的 `.pt` checkpoint 是导出的 recurrent 策略，不是无状态前馈网络。START 时的状态重置、10 Hz控制周期和输入顺序都属于模型契约。

## 2. 统一运行链

Webots 与实物使用相同控制节点：

```text
/global_info + /robot_N/scan
  -> PT/NEAT pattern
  -> /robot_N/drive_command
  -> hardware_protection_layer
  -> /robot_N/cmd_vel
```

Transport 由共享 YAML 中的 `bypass_protection: true` 直接发布 `/robot_N/cmd_vel`。实物侧不再覆盖该参数，也不新增第二套策略或安全控制链。

## 3. `/global_info` 契约

实物顶视桥必须复现 Webots 的 JSON 结构：

```json
{
  "timestamp": 12.345,
  "frame_id": "world",
  "robots": {
    "robot_0": {
      "position": {"x": 0.2, "y": -0.4, "z": 0.0},
      "velocity": {"x": 0.05, "y": 0.01},
      "orientation": [
        0.866, -0.5, 0.0,
        0.5, 0.866, 0.0,
        0.0, 0.0, 1.0
      ]
    }
  },
  "objects": {}
}
```

约束：

- 机器人名称与 namespace 一致，例如 `robot_0`；
- 位置单位 m，速度单位 m/s；
- 所有状态在同一个稳定世界坐标系；
- 机器人朝向是行优先3×3旋转矩阵；
- `yaw = atan2(R10, R00)`；
- 无效或过期实体不应继续以旧状态发布。

## 4. Transport：11维

PT 输入由以下内容组成：

```text
机器人位置 2
机器人速度 2
箱体相对目标位置 2
箱体相对机器人位置 2
箱体速度 2
是否到达目标 1
合计 11
```

需要在顶视系统中持续跟踪：

```text
robot_N
transport_box
target_zone
```

Transport 网络本身不读取 Lidar。接触箱体是任务行为的一部分，因此保持仿真验证过的 `bypass_protection: true`。

## 5. RoundFlocking：18维

```text
机器人/目标运动状态 6
12路 Lidar 12
合计 18
```

顶视系统必须提供所有 `robot_N` 和共同目标 `flocking_target`。策略仍从完整 `/robot_N/scan` 按共享配置获得12路输入；真实雷达的12 m物理量程不会改变训练输入上限。

环绕半径、actor 权重及径向/切向增益都是既有策略行为，场地变化时必须先在相同尺寸 Webots 场景验证。

## 6. Navigation：18维

```text
机器人位置 2
机器人速度 2
机器人与目标相对位置 2
12路 Lidar 12
合计 18
```

每个 `robot_N` 必须对应 `navigation_target_N`。PT 节点会根据 `LaserScan.angle_min`、`angle_increment`、`n_lidar_rays` 和 `max_range` 构造模型输入。

## 7. 不允许改变的契约

未经重新仿真验证，不要修改：

- 输入维度、顺序、符号和坐标系；
- 米制单位和 `world_scale`；
- Lidar 射线数量、角度顺序和训练截断范围；
- 输出到差速速度的映射；
- 控制周期；
- recurrent state 的 START 生命周期；
- Transport 的保护层旁路行为。

PT 与 NEAT 即使输入维度相同，也不代表内部观测表达和输出映射可以互换。切换实现必须重新完成对应任务的 Webots 验证。

## 8. 实物适配边界

允许与硬件相关、但不能改变策略语义的适配只有：

- 顶视二维码检测和 `camera_to_world` 标定；
- 真实雷达驱动发布标准 `/robot_N/scan`；
- e-puck2 驱动消费标准 `/robot_N/cmd_vel`；
- ROS namespace、串口和 DDS 网络配置。

所有策略代码、YAML、checkpoint、`lidar_layer` 和 `hardware_protection_layer` 与 Webots 共用。
