# e-puck 实物部署代码实施方案

> 版本：Draft 1.0  
> 日期：2026-07-19  
> 适用范围：当前主项目 `/root/workspace/ROS2swarm_w` 的 `src/`、`tests/` 和 `docs/`。  
> 架构参考：[ROS2swarm/ROS2swarm](https://github.com/ROS2swarm/ROS2swarm/tree/master)。

> 2026-07-19 状态更新：用户已确认实物为 **e-puck2**，机器人端运行 ROS 2，扩展 360 度 Lidar，机器人和任务物体由顶部二维码定位。本文的硬件型号不确定性已据此收敛；最终实施和上机步骤见 `EPUCK2_REAL_DEPLOYMENT_GUIDE.md`。

## 1. 结论先行

### 1.1 当前是否已经有实物部署代码？

**有一部分通用实机骨架，但没有完整的 e-puck 实物部署闭环。** 当前主项目已经具备：

- `geometry_msgs/Twist` 到 `/robot_N/cmd_vel` 的行为输出接口；
- `/drive_command -> HardwareProtectionLayer -> /cmd_vel` 的通用安全层结构；
- Lidar `LaserScan -> RangeData` 的传统传感器层；
- 通用行为输出、传感层和安全层结构；
- e-puck 的 Webots 仿真模型、Webots ros2-control 配置和仿真传感器节点；
- Transport、RoundFlocking、Navigation 的 PT/NEAT 控制节点、checkpoint 和输入适配器；
- 当前模型需要的 `/global_info` JSON 数据契约。

但当前主项目**没有**下列真实 e-puck 所需的完整实现：

- 真实 e-puck2/Pi-puck 的 I2C、串口或无线底盘驱动集成；
- 真实激光雷达型号对应的启动、串口权限、TF 和诊断配置；
- 顶视相机读取二维码/ArUco/AprilTag 并输出机器人及物体世界位姿的节点；
- 从标记位姿计算世界速度、处理遮挡/跳变/过期的状态估计器；
- 将真实追踪状态转换为当前策略节点所需 `/global_info` 的生产级状态桥；
- 底盘驱动层的失联自动停车和独立命令 watchdog；
- 无硬件条件下可重复运行的“模拟 e-puck 驱动 + 模拟定位 + 策略 + 安全”的端到端测试 launch。

`src/launch_webots/launch_webots/epuck_node.py` 不能当作真实 e-puck 驱动。它订阅 Webots 产生的 `odom` 和距离传感器，再拼成仿真的 `LaserScan`；它不访问真实 I2C、USB、蓝牙或 e-puck 无线协议。

### 1.2 是否可以没有实物先完成一版高质量代码？

**可以，但必须明确“代码完成”和“硬件验证完成”不是同一个终点。** 没有实物时可以完成并验证：

- ROS 2 topic、消息、namespace、TF 和 launch 契约；
- 标记检测结果到世界坐标的几何转换；
- 机器人/物体状态桥和速度估计；
- 4 个网络输入适配路径的维度、顺序、坐标系、Lidar 数值语义；
- PT/NEAT 推理、recurrent state、START/STOP、失联停车；
- 模拟 e-puck 底盘的轮速限制和命令 watchdog；
- Webots/rosbag/合成图像作为测试数据源时的完整行为链；
- 多机器人 namespace、任务目标 ID、网络延迟和传感器故障注入。

没有实物时不能最终证明：

- Lidar 实际安装偏角、盲区、反射、扫描频率和串口稳定性；
- e-puck 电机死区、轮胎打滑、轮距/轮径实测值和制动距离；
- Pi-puck CPU、内存、温度、电池和无线链路在长时间运行下的裕量；
- 顶视相机在真实光照、遮挡、运动模糊和标记尺寸下的识别率；
- 真实负载下 Transport 的接触力和多机协同效果。

因此实施目标应定义为：

> 先完成一版“硬件后端可替换、无实物可运行、故障可测试、带安全 fail-stop 的 ROS 2 实物部署软件”，再用真实 e-puck 完成参数标定和硬件验收。

## 2. 当前主项目与上游架构参考

### 2.1 当前主项目

主项目是：

```text
/root/workspace/ROS2swarm_w
├── src/communication_interfaces
├── src/launch_webots
├── src/ros2swarm
├── tests
└── docs
```

顶层 workspace 当前 ROS 2 发行版为 Humble。主项目的策略节点位于：

```text
src/ros2swarm/ros2swarm/movement_pattern/basic/
```

### 2.2 上游 ROS2swarm

[ROS2swarm/ROS2swarm](https://github.com/ROS2swarm/ROS2swarm/tree/master) 提供了多平台实机系统的分层启动、传感器抽象和行为安全架构参考：

- `sensor_layer` 将不同传感器统一成 `RangeData`；
- `HardwareProtectionLayer` 对 `drive_command` 做传统避障；
- 底盘、传感器、安全层和行为层通过 launch 组合。

它没有真实 e-puck2 驱动，也没有顶视二维码定位链，因此只用于架构参考。

## 3. 目标硬件和数据链假设

用户提供的信息已经确认目标平台具有：

- e-puck2 机器人底盘；
- 上层 Raspberry Pi/Pi-puck 类扩展板或计算单元；
- 独立的顶视/外部视觉定位条件，机器人和物体顶部贴有编码标记；
- 独立激光雷达传感器。

当前仍未确认：

- Pi-puck 的具体版本和 I2C 设备号；
- 激光雷达品牌、型号、串口波特率和 ROS 驱动；
- 顶部标记是 QR Code、ArUco 还是 AprilTag。

因此第一版软件必须把这几项做成参数化后端，而不是把某个型号写死在策略节点中。

### 3.1 目标 ROS 2 数据链

```text
真实底盘驱动
  -> /robot_N/odom
  -> /robot_N/cmd_vel

真实 360 度 Lidar 驱动
  -> /robot_N/scan

顶视相机 + 标记检测
  -> /tracked_markers 或 /marker_detections

标记位姿融合/速度估计/目标配置
  -> /global_state (typed message)
  -> /global_info (兼容当前策略的 JSON String)

当前 Transport/RoundFlocking/Navigation 节点
  -> /robot_N/drive_command 或 policy command

安全 supervisor + watchdog
  -> /robot_N/cmd_vel
```

### 3.2 部署位置建议

首版推荐中央计算和本地安全分层：

| 组件 | 建议运行位置 | 原因 |
|---|---|---|
| 顶视相机、二维码检测、全局状态桥 | 实验室 x86 主机 | 图像处理、标定、追踪和多目标融合更容易调试 |
| PT/NEAT 策略推理 | 首版运行在 x86 主机 | PyTorch 更容易安装；模型、日志和 rosbag 集中管理 |
| e-puck 底盘驱动 | Pi-puck/机器人本机 | 直接访问 I2C/硬件，避免把底盘控制完全依赖 Wi-Fi |
| 本地安全 supervisor/watchdog | Pi-puck/机器人本机 | 网络或中央节点失联仍能停车 |
| Lidar 驱动 | 连接 Lidar 的主机；若接在 Pi-puck 则本机 | 由 USB/串口物理连接决定 |

NEAT 网络很小，未来可以下沉到 Pi-puck；PT 版先保持中央运行，待测量实际 CPU、内存和推理延迟后再决定。

## 4. 开源资料及采用策略

### 4.1 Cyberbotics `epuck_ros2`

仓库：<https://github.com/cyberbotics/epuck_ros2>  
当前审阅提交：`91ff8a1fcc31db716e0add0a02e830dd910de669`。

该项目明确支持“e-puck2 + Pi-puck extension”，不是裸 e-puck 的通用串口驱动。其 `epuck_ros2_driver`：

- 通过 `/dev/i2c-12` 访问 Pi-puck；
- 订阅相对命名空间的 `cmd_vel`；
- 发布 `scan`、`odom`、`imu`、`ps0..ps7`、`tof` 等 topic；
- 用 `wheel_distance`、`wheel_radius` 参数计算差速轮速度；
- 提供 `--type test` 的 I2C 文件模拟后端，现有 launch tests 可在无真实硬件时验证电机、编码器、IR 和 LaserScan 数据链。

它的 LaserScan 是由 e-puck 红外传感器和前向 ToF 组合而成的稀疏扫描，覆盖约 `-150°..150°`，不是当前策略训练所依赖的真正 360°高密度激光雷达。它适合作为底盘/里程计后端参考，不能自动满足 RoundFlocking/Navigation 的 12 路 360°输入。

采用方式：

1. 不复制其 I2C 代码到策略包；
2. 将其作为外部依赖或带固定 commit 的 vendor package；
3. 在主项目上增加安全 patch：命令超时归零、namespace/frame 参数、诊断和版本兼容；
4. 保留其 `--type test` 思路，为无实物测试提供 fake I2C 后端；
5. 真实部署前确认手头机器人确实是 Pi-puck 硬件，否则切换到无线/串口底盘后端。

参考：

- README：<https://github.com/cyberbotics/epuck_ros2/blob/master/README.md>
- Driver：<https://github.com/cyberbotics/epuck_ros2/tree/master/epuck_ros2_driver>
- Driver tests：<https://github.com/cyberbotics/epuck_ros2/blob/master/epuck_ros2_driver/test/test_driver.py>

### 4.2 GCtronic `epuck_ros2`

仓库：<https://github.com/gctronic/epuck_ros2>  
当前审阅提交：`c76eb7a47fba9cbda2fb81d3e2ebd051f0f56455`。

该仓库是在 Cyberbotics 驱动基础上继续维护的版本，额外包含 `epuck_ros2_uwb`。UWB 节点发布 `PoseStamped`，可作为另一种定位后端，但本项目已经具备顶视标记定位条件，不应把 UWB 作为必需依赖。

采用方式：

- 优先比较该仓库与 Cyberbotics 版本对 e-puck2/Pi-puck 硬件的兼容性；
- 在独立 overlay 中构建，不直接覆盖主项目的 `ros2swarm` 包；
- 只让它提供底盘和原始传感 topic，状态桥负责转换为本项目统一实体状态。

### 4.3 SLAMTEC `rplidar_ros`

仓库：<https://github.com/Slamtec/rplidar_ros>  
采用 ROS 2 分支：`ros2`；当前审阅提交：`24cc9b6dea97e045bda1408eaa867ce730fd3fc3`。

该分支 README 明确列出 A1/A2/A3/S1/S2/S2E/S3/T1/C1 等型号，要求：

- 型号匹配的 `serial_baudrate`；
- USB 串口 udev 权限；
- 正确的 `laser_frame`/TF；
- 根据物理方向确认 `LaserScan` 的 angle_min、angle_increment 和顺时针方向。

采用方式：

- 若实物是 RPLidar A2，优先使用该 ROS 2 分支；
- 不把物理 `range_max` 直接当成网络输入上限；
- 在它之后增加本项目的 `policy_scan_adapter`，把任意合法 `LaserScan` 采样为训练规定的 12 路；
- 保留完整扫描供 RViz、安全层和录包，不在驱动层提前裁掉 0.20/0.35 m 之外的数据。

参考：<https://github.com/Slamtec/rplidar_ros/tree/ros2>。

### 4.4 `ros2_aruco` / `aruco_ros`

主要参考：

- `ros2_aruco`：<https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/humble>
- `aruco_ros`：<https://github.com/pal-robotics/aruco_ros/tree/humble-devel>

`ros2_aruco` 的 ROS 2 API 订阅 `/camera/image_raw`、`/camera/camera_info`，发布 `/aruco_poses` 和带 ID 的 `/aruco_markers`，参数包括 marker size、dictionary、camera topic 和 camera frame。它适合作为“顶视相机 -> 标记相机坐标位姿”的前端。

但是，若当前顶部标签真的是普通 QR Code，而不是 ArUco/AprilTag，`ros2_aruco` 不能直接检测。应实现统一接口：

```text
MarkerDetector backend:
  aruco -> ros2_aruco
  apriltag -> apriltag_ros/其他 AprilTag 节点
  qr -> OpenCV QRCodeDetector + solvePnP

统一输出:
  marker_id, stamp, camera_frame, position, orientation, reprojection_error, valid
```

第一阶段只需确认标签类型并选择一个后端，策略层不应感知检测器差异。

## 5. 目标软件架构

### 5.1 新增包和模块建议

建议不把真实硬件代码散落进 `movement_pattern/basic`，而是增加两个边界清晰的层：

```text
src/
├── communication_interfaces/
│   └── msg/
│       ├── TrackedEntity.msg
│       ├── TrackedEntityArray.msg
│       ├── PolicyHealth.msg
│       └── SafetyState.msg
├── ros2swarm/
│   └── ros2swarm/
│       ├── movement_pattern/          # 保留当前策略
│       ├── real_robot/                # 与硬件无关的桥和安全节点
│       │   ├── global_state_bridge.py
│       │   ├── marker_state_fuser.py
│       │   ├── policy_scan_adapter.py
│       │   ├── safety_supervisor.py
│       │   └── observation_monitor.py
│       └── test_support/
│           ├── fake_epuck_driver.py
│           ├── fake_marker_tracker.py
│           └── synthetic_scan_publisher.py
└── epuck_bringup/                    # 建议独立 ament package
    ├── launch/
    ├── config/
    ├── urdf/
    └── test/
```

如果第一阶段不希望增加新的 ROS package，也可以先把 `real_robot` 放入现有 `ros2swarm`，但 e-puck vendor driver 应保持独立 package，避免主项目被 I2C/相机/雷达依赖污染。

### 5.2 Typed 状态消息

当前策略消费 `std_msgs/String` JSON，适合 Webots 快速开发，不适合作为实物长期接口。建议增加：

```text
TrackedEntity.msg
  string name
  uint8 entity_type              # ROBOT/OBJECT/TARGET
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
  builtin_interfaces/Time source_stamp
  bool pose_valid
  bool twist_valid
  float32 confidence

TrackedEntityArray.msg
  std_msgs/Header header          # frame_id = world
  TrackedEntity[] entities
```

过渡期保留：

```text
TrackedEntityArray -> global_info_compat_bridge -> std_msgs/String /global_info
```

这样当前六个策略节点无需第一步大改，同时新节点和测试可以使用强类型、带 Header、可表达 validity 的接口。

### 5.3 顶视标记状态链

```text
camera/image_raw + camera_info
        |
        v
marker_detector
        |
        v
camera-frame marker detections
        |
        | TF/camera calibration + planar world calibration
        v
marker_state_fuser
        |
        | marker_id -> robot_0 / transport_box / target_zone
        | marker offset -> base_link/object center
        | yaw unwrap + velocity filter + confidence
        v
/global_state (world frame)
        |
        +--> global_info_compat_bridge -> /global_info
```

#### 标记定位的具体步骤

1. 标定相机内参、畸变和相机到实验场地 `world` 平面的外参；
2. 在配置中维护 `marker_id -> entity_name/entity_type` 映射；
3. 使用 marker 四角和已知边长求相机坐标下的 3D pose；
4. 将 marker 中心投影到 `world` 平面；
5. 应用 `T_entity_marker`，把标记中心转换为机器人底盘中心或物体中心；
6. 从 marker 朝向提取机器人 yaw，并加上安装偏角 `marker_yaw_offset`；
7. 对每个实体做带时间戳的速度估计；
8. 遮挡、低置信度、重投影误差过大或时间超限时，将 validity 置 false；
9. 不对长时间丢失的实体继续“假装”发布旧位置。

#### QR、ArUco、AprilTag 的判断

不要仅凭中文习惯把所有黑白标签称为二维码。第一项实施工作应从现有标签图像确认：

- 四周有 ArUco 字典网格、具有明确 ID 方向：使用 ArUco；
- AprilTag 家族编码：使用 AprilTag detector；
- 带三个定位角方块的普通 QR：使用 QRCodeDetector，并自行做 pose/ID 映射。

这三种检测器的输出统一到上述 `marker_id + pose + confidence` 接口。

### 5.4 速度估计策略

当前策略输入需要世界速度和/或本体速度。推荐：

```text
视觉 pose + source timestamp
  -> alpha-beta/Kalman filter
  -> world vx/vy
  -> 用 yaw 旋转得到 body vx/vy
```

对于机器人可选地融合 `/odom`：

- 视觉负责全局位置和航向，抑制长期漂移；
- 编码器 odom 负责短时间速度连续性；
- 融合器在视觉短暂丢失时最多短时预测；
- 超过 max_age 后必须 fail-stop，而不是无限外推。

对于 Transport 包裹，优先使用包裹顶部标记的视觉轨迹；不能用机器人 odom 推断包裹速度，除非包裹与机器人已经物理锁定且任务逻辑明确允许这一假设。

## 6. Lidar 适配方案

### 6.1 物理扫描保持原始，策略输入另行适配

节点链应分为：

```text
/robot_N/scan_raw       # 驱动原始 LaserScan
        |
        +--> /robot_N/scan_policy  # 12 路方向、固定清洗和裁剪
        |
        +--> safety_supervisor     # 保留完整或更保守的局部扫描
```

策略适配必须显式处理：

- `angle_min`、`angle_increment`、顺/逆时针方向；
- `base_link -> laser_frame` 安装旋转；
- NaN、Inf、0、低于 `range_min` 和超出 `range_max` 的语义；
- 雷达自反射、机器人顶部结构和安装遮挡；
- 目标采样角度 `0, 30, ..., 330` 度；
- 网络训练上限与物理雷达上限的区别。

### 6.2 四个控制器的固定输入配置

| 控制器 | 采样后维度 | 网络值 | 实机约束 |
|---|---:|---|---|
| RoundFlocking PT | 12 | 原始距离，裁剪 `[0,0.20]` m | 必须能解释 360° 12 个方向 |
| RoundFlocking NEAT | 12 | 原始距离，裁剪 `[0,0.20]` m | 同上 |
| Navigation PT | 12 | `0.35 - distance` | 距离上限 0.35 m |
| Navigation NEAT | 12 | `0.20 - distance` | 距离上限 0.20 m |

模型固定 18 输入，因此 `n_lidar_rays` 不能在实机配置中改成其他值，除非重新训练/导出模型。

### 6.3 传感器不足时的策略

如果当前独立 Lidar 不是 360°二维扫描，而是单线、前向或小范围传感器，有三种选择：

1. **推荐**：增加真正 360° Lidar，保持原模型输入语义；
2. **可测试但需重新训练**：用已知地图和标记目标合成 12 路射线；
3. **不推荐**：将 8 个 IR/ToF 直接复制/插值成 12 路，维度虽然匹配，但训练分布和角度覆盖不匹配。

无实物阶段可以实现第 2 种作为仿真/回放 backend，但必须在文档和 topic 中标记 `synthetic_scan=true`，不能误认为真实避障感知。

## 7. 安全控制设计

### 7.1 不再让策略直接拥有最终 `/cmd_vel`

当前 Transport 配置会绕过 HardwareProtectionLayer。实物版本应建立明确的安全管线：

```text
policy_cmd_vel
      |
      v
task-aware safety supervisor
  - global STOP
  - marker/global-state freshness
  - Lidar emergency stop
  - wheel velocity/acceleration limit
  - communication timeout
  - Transport contact exception
      |
      v
/robot_N/cmd_vel_safe
      |
      v
e-puck driver
```

Transport 的“允许接触包裹”只能放宽包裹方向/接触距离规则，不能关闭：

- 人员/墙体碰撞保护；
- 实验场地边界；
- 定位丢失停车；
- Lidar/相机状态失效停车；
- ROS 2/Wi-Fi 失联停车；
- 底盘硬件 watchdog。

### 7.2 Watchdog 层级

至少需要三层：

1. 策略节点：输入过期或推理异常时立即发布零速度；
2. 本机安全节点：策略命令超过 `cmd_timeout` 未更新时发布零速度；
3. e-puck 驱动/固件：ROS 2 进程、Wi-Fi 或 I2C 异常时，电机最终归零。

第一版建议 `cmd_timeout` 从 0.25 s 起步，实际以控制周期、Wi-Fi 抖动和底盘响应测试调整；不能只依赖一个固定值，必须记录超时事件。

### 7.3 当前代码需要的策略侧修改

在物理部署实现中应处理之前分析出的当前问题：

- 原生 NEAT 节点缺失输入时不能生成随机观测；
- Navigation/Transport 的输入必须检查 freshness、finite 和维度；
- recurrent actor 必须在新 episode START 时 reset；
- HPL/Safety supervisor 在 STOP、传感器未就绪和传感器过期时必须 fail-stop；
- Navigation 的 `goal_threshold` 应接入任务完成停车逻辑；
- Lidar 采样统一使用角度元数据；
- Transport 实物配置不能只用 `bypass_protection=true` 表达接触需求。

这些修改应先在模拟驱动和单元测试中完成，再接真实硬件。

## 8. 无实物实施与测试计划

### Phase 0：工程基线与接口冻结

目标：让主项目可重复构建并冻结实机接口。

工作项：

- 固定 ROS 2 包选择、依赖安装和构建命令；
- 锁定 ROS 2 Humble、Python、PyTorch、`neat-python` 和模型 SHA-256；
- 建立 `real_robot` 参数命名空间和版本化 YAML；
- 写出 typed `TrackedEntity`/`TrackedEntityArray` 消息；
- 设计 `/global_state`、`/global_info`、`/scan_raw`、`/scan_policy`、`policy_cmd_vel` 和 `cmd_vel_safe` 的 topic 契约。

验收：

- 干净环境可以只构建主项目；
- `ros2 topic list` 中没有仿真节点时，接口名称仍稳定；
- 所有模型加载时能报告预期维度。

### Phase 1：底盘和 Lidar mock backend

目标：不接真实 e-puck，在当前机器模拟 `/odom`、`/scan_raw`、`cmd_vel` 和驱动状态。

实现：

- `fake_epuck_driver`：接受 `cmd_vel_safe`，按差速运动学积分，发布 `/odom`；
- `synthetic_scan_publisher`：发布含 angle metadata、障碍物、NaN/Inf/丢帧的 LaserScan；
- `epuck_driver_adapter`：将真实/模拟驱动 topic 统一到 `/robot_N`；
- 安全 supervisor 的纯函数：速度限幅、轮速限幅、加速度限幅、命令过期停车。

测试：

- 正向/反向/原地旋转的轮速符号；
- wheel separation/wheel radius 对运动学结果的影响；
- 250 ms 无命令后 `cmd_vel_safe=0`；
- Lidar 未就绪、NaN、Inf、全 0 和过期时的 fail-stop；
- 多 namespace 并行。

### Phase 2：顶视标记定位 mock backend

目标：在没有相机和真实二维码时，仍完整测试全局状态链。

实现：

- `fake_marker_tracker` 发布固定/运动的 robot、box、target marker pose；
- `marker_state_fuser` 完成 marker ID 映射、平面变换、安装偏移、yaw 和速度估计；
- 支持 ground-truth/合成 marker detections 两种输入；
- `global_info_compat_bridge` 输出当前策略所需 JSON。

测试：

- 世界原点、x/y 正方向和 yaw 正方向；
- marker 在相机左/右/前/后时的相对向量符号；
- 机器人旋转时 body velocity 转换；
- 包裹运动速度和目标静态速度；
- marker 遮挡、短暂丢失、乱序时间戳、跳变和恢复；
- `robot_0 -> navigation_target_0` 的 ID 映射。

### Phase 3：标记检测图像测试

目标：不接真实相机，使用合成或录制图像验证 detector。

实现：

- 用 OpenCV 生成 ArUco 测试图像；
- 使用真实标签照片/录制视频作为回归样本；
- 若实际标签是 QR，增加 QRCodeDetector backend；
- 固定 camera_info、marker_size、平面外参和畸变模型；
- 输出检测重投影误差和 pose covariance。

测试：

- 不同距离、角度、光照、模糊、部分遮挡；
- 多机器人 ID 同帧识别；
- marker orientation 0/90/180/270 度；
- 相机图像时间戳与状态发布频率；
- 不允许低置信度检测直接驱动策略。

### Phase 4：策略端到端测试

目标：真实实物接口已被 mock 后，验证六个 PT/NEAT 节点。

工作项：

- 用 synthetic global state 驱动 Transport、RoundFlocking、Navigation；
- 用 synthetic scan 驱动 12 路适配器；
- 记录 observation、raw action、safe command 和输入 age；
- 运行 START/STOP、网络重启、状态桥退出、Lidar 退出等故障注入；
- 对比 Webots golden rosbag 和 mock physical rosbag 的观测统计。

验收：

- Transport 始终 11 维；其他模式始终 18 维；
- 没有输入时不调用网络，不推进 recurrent state；
- 所有错误路径最终输出零速度；
- 安全节点命令优先级稳定，无两个 publisher 竞争最终 `/cmd_vel`；
- 多机器人各自使用独立 recurrent state。

### Phase 5：Webots 作为软件在环平台

目标：不使用真实 e-puck，但使用当前 Webots 作为运动和碰撞植物，验证“物理 launch 结构”。

做法：

- Webots driver 只提供模拟 `/odom`、`/scan_raw` 和底盘执行；
- 运行与实物相同的 `marker_state_fuser`，由 Webots ground-truth adapter 模拟顶视标记输出；
- 运行与实物相同的 `global_info_compat_bridge`、policy scan adapter、safety supervisor 和策略节点；
- 禁止策略直接订阅 Webots Supervisor 真值作为最终物理部署路径；真值只能经过与相机输出相同的 adapter。

这样可以发现真实 launch 中最容易出现的 namespace、topic、TF、QoS 和安全链问题。

### Phase 6：硬件后端接入前的离线门禁

在真实 e-puck 到来前，以下条件全部满足才允许上车：

- 主项目可在干净 Humble workspace 构建；
- mock driver、mock tracker、策略和 safety 全链通过 launch tests；
- Webots software-in-loop 通过；
- 真实 driver package 可以在普通 x86 主机以 `--type test` 构建/启动；
- marker detector 能对实际标签照片输出正确 ID 和位姿；
- Lidar driver 的 `LaserScan` 录包可以离线回放；
- 断开所有输入的故障测试均在阈值内停车；
- 所有最终 cmd_vel 只有一个安全发布者。

## 9. 真实 e-puck 接入阶段计划

### 9.1 第一阶段：只接底盘，不接策略

1. 确认 e-puck/e-puck2、Pi-puck 版本和 `/dev/i2c-*`；
2. 在机器人本机安装/构建 `epuck_ros2_driver`；
3. 运行硬件 driver 自带 `--type test` 和真实模式对比；
4. 通过 `ros2 topic echo` 检查 `odom`、原始传感器、TF；
5. 低电压/抬轮状态验证 `cmd_vel`，确认左右轮方向；
6. 加入命令 watchdog 后才允许落地运行。

### 9.2 第二阶段：只接 Lidar

1. 确认雷达型号、供电、电流、USB/串口设备和波特率；
2. 安装对应 ROS 2 driver 和 udev 规则；
3. 检查 `/scan_raw` 的角度、频率、frame、range_min/max；
4. 建立 `base_link -> laser_frame` 静态 TF；
5. RViz 中放置单墙/单物体，确认正前、左、右和反向索引；
6. 录制至少 10 分钟原始 scan，检查丢帧和 NaN/Inf 比例。

### 9.3 第三阶段：只接顶视定位

1. 相机固定、标定内参和地面平面；
2. 确认标签类型、ID、边长和安装位置；
3. 校准 marker 中心到机器人底盘中心偏移；
4. 校准 marker yaw 到 `base_link` yaw 的固定偏角；
5. 机器人手动移动，验证 `/global_state` 的位置/速度；
6. 机器人静止时速度应回到接近 0，遮挡后应正确 invalid/停车。

### 9.4 第四阶段：Shadow policy

策略只计算和记录，不发布底盘命令：

- 比较真实 observation 与 Webots golden observation；
- 检查动作是否饱和、抖动或因 yaw 错误反向；
- 检查 Transport 包裹相对位置和速度；
- 检查 RoundFlocking 12 路 Lidar 是否看到目标、邻居和墙；
- 检查 Navigation 目标 ID 是否与机器人匹配。

### 9.5 第五阶段：单台、低速、软边界

- 先 Navigation，再 RoundFlocking，最后 Transport；
- 速度限制先使用配置值的 20%--30%；
- 现场人员持物理急停；
- 安全节点和底盘 watchdog 分别拔网线/杀进程测试；
- 先空载，Transport 最后使用真实物体；
- 每一步保留 rosbag、参数、git commit 和诊断日志。

## 10. 代码实现的具体交付顺序

### Milestone A：接口和测试基座

交付：

- typed tracking messages；
- `real_robot` Python package skeleton；
- fake driver/tracker/scan；
- safety pure functions；
- launch test fixture；
- 主项目可重复构建。

### Milestone B：状态和 Lidar adapter

交付：

- marker detection adapter interface；
- ArUco backend；
- QR backend placeholder/实现（按标签类型决定）；
- planar world calibration；
- velocity filter；
- `global_info` compatibility bridge；
- angle-aware 12-ray policy scan adapter。

### Milestone C：策略安全改造

交付：

- 删除随机观测 fallback；
- freshness/finite/dimension checks；
- recurrent state reset；
- safe command topic；
- Navigation arrival state；
- Transport contact-aware safety mode；
- policy/observation diagnostics。

### Milestone D：Webots software-in-loop

交付：

- Webots ground-truth-to-marker mock adapter；
- physical-style launch；
- all six controller integration tests；
- fault injection tests；
- golden rosbag comparison tool。

### Milestone E：e-puck/Pi-puck backend

交付：

- pinned `epuck_ros2_driver` dependency；
- watchdog patch；
- wheel parameter config；
- e-puck namespace/frame launch；
- Lidar vendor launch/remapping；
- local safety supervisor launch；
- hardware diagnostic checklist。

### Milestone F：真实标定和验收

交付：

- camera/marker calibration YAML；
- robot-specific wheel calibration；
- Lidar TF/calibration YAML；
- field-test rosbag；
- sim-to-real discrepancy report；
- final deployment image/workspace。

## 11. 推荐目录和 launch 设计

### 11.1 运行配置

```text
src/epuck_bringup/config/
├── common/
│   ├── safety.yaml
│   ├── marker_tracker.yaml
│   └── scan_adapter.yaml
├── epuck2_pipuck/
│   ├── driver.yaml
│   ├── lidar.yaml
│   └── frames.yaml
└── marker_maps/
    ├── lab_world.yaml
    └── transport_objects.yaml
```

### 11.2 Launch 入口

```text
epuck_hardware.launch.py
  - vendor driver
  - lidar driver
  - static TF
  - local safety/watchdog

epuck_tracking.launch.py
  - camera driver or image input
  - marker detector
  - marker state fuser
  - global state publisher

epuck_policy.launch.py
  - PT or NEAT policy
  - policy_scan_adapter
  - task config
  - diagnostics

epuck_robot.launch.py
  - above three groups
  - use_sim_time=false
  - namespace=robot_N
  - emergency-stop policy
```

物理 launch 不应复用当前 Webots 的 `global_info_supervisor`，也不应启动 WebotsController、空壳 URDF 或仿真时间。

## 12. 必须定义的验收指标

### 12.1 定位

- marker 检测频率：目标不低于策略频率的 2 倍；
- 单帧检测延迟和时间戳可追踪；
- 静止位置抖动、yaw 抖动、速度估计噪声有统计报告；
- 目标遮挡超过 `max_age` 后策略停车；
- marker ID 不会错误映射到其他机器人。

### 12.2 Lidar

- 实际 `/scan_raw` 频率、丢帧和非法值比例；
- 正前/正后/左右方向映射正确；
- 适配后的 12 维输入范围严格正确；
- 网络输入和安全输入分开，策略裁剪不影响安全层原始数据。

### 12.3 控制和安全

- 速度、角速度、轮速和加速度均不超限；
- 策略节点退出、DDS 断开、状态桥退出、Lidar 退出后在 watchdog 阈值内停车；
- STOP 命令无论当前任务和 HPL 状态都能到达底盘；
- Transport 接触包裹时不会因为安全层误判而完全失去推动能力，但人员/墙体仍受保护；
- 单一最终 `/cmd_vel` 发布者，不存在行为节点和安全节点竞争。

### 12.4 策略一致性

- six-node 输入维度和字段顺序与 checkpoint 一致；
- PT/NEAT 的坐标系差异有明确配置和测试，不混用；
- `world_scale`、速度缩放、Lidar 上限和动作 mapping 由 YAML 固定；
- recurrent state episode 生命周期可重复；
- 在 Webots golden bag 和 mock physical bag 上动作统计可解释。

## 13. 主要风险和决策门

### 决策门 1：真实标签类型

如果标签是 ArUco/AprilTag，可以直接采用对应 ROS 2 detector；如果是普通 QR，必须实现独立 QR pose backend。没有这一步，不能承诺“二维码定位代码已经完成”。

### 决策门 2：真实 Lidar 模型

如果是 RPLidar A2，使用 `rplidar_ros` ROS 2 分支；如果是其他型号，仅复用 `LaserScan` 接口和 adapter，不复用型号参数。物理量程、串口波特率、扫描方向必须以实测/说明书为准。

### 决策门 3：Pi-puck 硬件

如果是 e-puck2 + Pi-puck，优先采用/修补 `gctronic/epuck_ros2`；如果没有 Pi-puck，不能直接依赖其 I2C driver，需要另行实现蓝牙/Wi-Fi/串口底盘 bridge。

### 决策门 4：策略是否需要全局定位

当前模型不是只靠 Lidar 的局部策略。Transport 和 Navigation/RoundFlocking 都使用 `/global_info` 中的绝对或相对状态。顶视二维码定位链必须先于策略实物运行完成，不能用只有 odom 的底盘 bringup 替代。

### 决策门 5：是否重新训练

如果真实 Lidar、标记速度、相机延迟、机器人动力学与 Webots 分布差异较大，第一版软件可以完成，但策略效果不一定达到仿真水平。需要先录包比较 observation 分布，再决定是标定/适配还是重新训练，而不是用安全层长期掩盖策略域差异。

## 14. 本方案后的第一批实现任务

在开始写物理代码时，建议按以下顺序执行：

1. 为主项目增加可重复构建基线和版本锁定；
2. 定义 typed tracking/safety messages；
3. 写 `fake_epuck_driver`、`fake_marker_tracker`、`synthetic_scan_publisher`；
4. 写并测试 `policy_scan_adapter`；
5. 写 marker state fuser 和 `/global_info` compatibility bridge；
6. 修复策略节点的随机输入、freshness、reset 和 fail-stop；
7. 写统一 safety supervisor 和底盘 watchdog mock；
8. 用 Webots 做 software-in-loop；
9. 集成真实标签图像 detector；
10. 集成真实 Lidar ROS 2 driver；
11. 最后接入/修补 e-puck/Pi-puck driver；
12. 完成 shadow policy、低速单机和多机验收。

这个顺序能在没有真实机器人时先完成绝大多数代码和测试，同时把真正依赖硬件的内容限制在底盘驱动、雷达物理参数、相机标定和最终动力学验证四个边界内。

## 15. 参考代码索引

### 当前主项目

- 策略节点：`src/ros2swarm/ros2swarm/movement_pattern/basic/`
- PT runtime：`src/ros2swarm/ros2swarm/vmas_actor_policy.py`
- PT Transport adapter：`src/ros2swarm/ros2swarm/vmas_transport_adapter.py`
- PT planar adapter：`src/ros2swarm/ros2swarm/vmas_planar_adapter.py`
- Webots e-puck 仿真节点：`src/launch_webots/launch_webots/epuck_node.py`
- Webots Supervisor：`src/launch_webots/launch_webots/global_info_supervisor.py`
- 通用 HPL：`src/ros2swarm/ros2swarm/hardware_protection_layer.py`

### 外部开源资料

- Cyberbotics e-puck ROS 2：<https://github.com/cyberbotics/epuck_ros2>
- GCtronic e-puck ROS 2：<https://github.com/gctronic/epuck_ros2>
- SLAMTEC RPLidar ROS 2：<https://github.com/Slamtec/rplidar_ros/tree/ros2>
- ROS 2 ArUco：<https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/humble>
- PAL ArUco ROS 2 分支：<https://github.com/pal-robotics/aruco_ros/tree/humble-devel>
- ROS 2 robot_localization：<https://github.com/cra-ros-pkg/robot_localization>
