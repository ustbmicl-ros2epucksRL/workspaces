# e-puck2 实物场地、坐标与网络输入适配指南

> 文档日期：2026-07-23
>
> 适用项目：`ROS2swarm_w`
>
> 适用硬件：已加装激光雷达的智能佳 e-puck2 / e-puck2.3、顶视相机或高频投影定位系统
>
> 目的：说明当前 3.5 m × 3.5 m 仿真假设，以及真实场地尺寸、中心、坐标方向、定位设备变化后应该修改什么、不能直接修改什么。

## 1. 先给出结论

当前模型和代码不能只靠“相机能看到机器人”就直接迁移到任意场地。实物部署至少要同时处理三层坐标：

```text
相机或厂家定位坐标
  -> 真实物理 world 坐标（米、场地中心为原点）
  -> 策略/网络输入坐标（按训练约定归一化）
```

推荐遵守以下原则：

1. `/global_state` 始终表示真实物理量：位置单位为 m，速度单位为 m/s，角度单位为 rad。
2. 顶视相机主要直接测量机器人、任务物体和目标标记的平面位置、朝向及时间戳。
3. 当前自建相机链中的 `vx、vy` 不是相机直接输出，而是相邻位置差分后滤波得到。
4. `world_scale=1.75` 来自当前 3.5 m × 3.5 m 场景的半边长，不是 e-puck2 的固定参数。
5. `velocity_alpha=0.65` 和互补权重 `0.35` 是可配置滤波参数，不是固定物理常数，也不由场地尺寸直接决定。
6. 实际场地若是标准 2 m × 2 m，可先以场地中心为 `(0,0)`，得到约 `[-1,1] m` 的物理坐标，并把六个策略配置的 `world_scale` 候选值改为 `1.0`。
7. 安全距离、机器人直径、箱体半径、轮距、Lidar 截断距离不能因为场地从 3.5 m 缩到 2 m 就机械地乘以 `2/3.5`。
8. 仅调整归一化可以恢复网络输入的坐标范围，但不能保证真实动力学、障碍物分布和任务难度仍与训练一致；必须进行离线观测对比和低速实测。

长期最稳妥的实现是让“物理安全坐标”和“模型等效坐标”分开：安全层只看真实米制 `/global_state`，策略桥在生成 `/global_info` 时再进行物理场地到模型场地的变换。当前 `global_info_bridge.py` 还只是直接复制坐标，没有实现独立的策略坐标变换。

## 2. 智能佳场地和定位设备的公开信息

智能佳公开页面给出的标准群体智能场地可用区域为 **2 m × 2 m**，也支持其他尺寸定制；场地包含边框、挡板、桌面、相机和补光灯安装支架。购买设备的实际内尺寸仍应现场测量，不能只使用商品标称值。

智能佳 e-puck2.3 方案还列出了高频投影定位系统、R&B 红外测距通信板和 360° 全景相机：

- 高频投影定位系统可以提供平面坐标输出；厂家介绍中还提到位置、方向和速度信息。
- R&B 板的 12 组红外收发器约每 30° 分布，主要用于带有对应设备的机器人之间测距、测向和通信。
- e-puck2.3 的 360° 全景相机是反射镜视觉系统。

后两项虽然都有“360°”或“12 方向”的表述，但都不能直接假定为当前策略所需的 `sensor_msgs/LaserScan`：

- R&B 未必能检测墙、普通箱体或所有障碍物；
- 全景相机输出图像，不直接输出以米为单位的 2D 激光距离；
- 当前实机启动链需要激光雷达驱动或转换节点发布 `/robot_N/scan_raw`。

本项目的 e-puck2 已经升级并加装激光雷达，因此实际方案应直接使用这台激光雷达提供距离观测，而不是用 R&B 或环视相机替代。仍需根据雷达铭牌和实测确认其型号、串口/网络接口、波特率、扫描方向、频率、量程、盲区和 ROS 2 驱动，不能仅根据 e-puck2 的网页型号判断。

## 3. 当前项目中的数据流

### 3.1 顶视相机链

当前实现的数据流是：

```text
/overhead_camera/image_raw
  -> QR/ArUco 检测
  -> /marker_detections
  -> camera_to_world + marker_offsets
  -> 相邻位置差分和速度滤波
  -> /global_state
  -> /global_info
  -> PT/NEAT 网络观测
```

相关文件：

| 文件 | 当前作用 |
| --- | --- |
| `src/ros2swarm/config/e_puck/real_robot/marker_tracking.yaml` | 相机话题、标记映射、`camera_to_world`、速度滤波 |
| `src/ros2swarm/ros2swarm/real_robot/marker_state_fuser.py` | 把检测坐标变换到 `world`，发布 `/global_state` |
| `src/ros2swarm/ros2swarm/real_robot/state.py` | 用位置差分计算并滤波 `vx、vy` |
| `src/ros2swarm/ros2swarm/real_robot/global_info_bridge.py` | 把 `/global_state` 转成旧策略使用的 `/global_info` JSON |
| `scripts/calibrate_camera_to_world.py` | 用对应点拟合相机平面到物理世界的二维仿射变换 |

当前 `marker_tracking.yaml` 中：

```yaml
camera_to_world: [1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
velocity_alpha: 0.65
max_velocity_dt: 0.50
publish_period: 0.05
```

`camera_to_world` 仍是需要用实际场地标定结果替换的占位值，不能直接用于正式部署。

### 3.2 相机直接获得什么，计算得到什么

对当前 QR/ArUco 方案，可以把信息来源理解为：

| 状态量 | 当前来源 |
| --- | --- |
| `x、y` | 顶视相机检测标记，再经 `camera_to_world` 和标记中心偏移换算 |
| `yaw` | 标记朝向，再经坐标轴方向和标记安装偏角换算 |
| `vx、vy` | 连续两帧世界位置差分，再进行一阶滤波 |
| 目标位置 | 顶视标记，或以后改为配置中的固定物理坐标 |
| 箱体位置/速度 | 箱体标记的位置检测及同样的位置差分 |
| Lidar 12 路距离 | 机器人本体的 `/scan_raw` 经角度感知采样和量程截断 |
| 网络动作 | PT/NEAT 根据上述观测推理得到，再换算为差速底盘命令 |

因此，“顶部相机主要就是获取坐标位置，其他速度等输入通过计算换算”对**当前自建定位链**基本正确，但要加两个限定：

1. 相机还负责提供标记朝向和采样时间，不只是位置。
2. 如果改用智能佳高频定位系统，而且厂家接口已经可靠地提供 `vx、vy`，可以直接使用厂家速度，不必重复对位置差分；但必须验证其坐标系、单位、延迟和滤波特性。

### 3.3 已加装激光雷达的接入链

当前实物已经加装激光雷达，推荐数据流为：

```text
加装激光雷达
  -> 雷达 ROS 2 驱动
  -> /robot_N/scan_raw（真实完整 LaserScan）
       ├─> safety_supervisor（真实米制扫描）
       └─> policy_scan_adapter
             -> /robot_N/scan（12 方向、按 checkpoint 量程截断）
                  -> Navigation / RoundFlocking 网络
```

这里必须保留两份语义：

- `/scan_raw` 是完整的真实物理扫描，供安全层判断实际障碍距离；
- `/scan` 是网络输入适配结果，只保留 12 个方向，并按模型截断到 `0.20 m` 或 `0.35 m`。

当前 `epuck2_physical.launch.py` 默认：

```text
start_lidar=false
lidar_package=rplidar_ros
lidar_executable=rplidar_composition
lidar_serial_port=/dev/ttyUSB0
lidar_baudrate=115200
lidar_sensor_yaw=0.0
```

如果加装的是代码已支持的 SLAMTEC RPLidar，可按实际串口和型号参数启动，例如：

```bash
ros2 launch ros2swarm epuck2_physical.launch.py \
  robot_id:=0 \
  pattern:=navigation_pattern \
  start_driver:=true \
  start_lidar:=true \
  lidar_serial_port:=/dev/serial/by-id/实际设备名 \
  lidar_baudrate:=实际波特率 \
  lidar_sensor_yaw:=实际安装偏角
```

如果加装的是其他品牌或驱动参数不兼容，不要强行使用上述默认驱动。应单独启动正确驱动，把话题放入机器人命名空间并最终发布：

```text
/robot_0/scan_raw
```

此时运行物理 launch 时设置 `start_lidar:=false`，但保持 `start_scan_adapter:=true` 和 `start_safety:=true`。

在运行网络前检查：

```bash
ros2 topic hz /robot_0/scan_raw
ros2 topic echo /robot_0/scan_raw --once
ros2 topic hz /robot_0/scan
ros2 topic echo /robot_0/scan --once
```

必须确认：

- `scan_raw` 的 `ranges` 是以 m 为单位的距离；
- `angle_min`、`angle_increment` 和扫描正方向正确；
- 前方障碍确实落到机器人本体 `+x` 对应方向；
- `range_min/range_max` 与实际型号一致；
- `NaN/Inf/无回波` 的行为已知；
- 雷达中心相对 `base_link` 的安装位姿已配置；
- 电机、Wi-Fi、补光灯工作时雷达供电和扫描频率稳定。

场地从 3.5 m 改为 2 m 时，Lidar 驱动不需要因为场地尺寸而改变单位或角度。策略侧 `0.20/0.35 m` 的截断范围也建议首版保持不变，因为机器人、墙和局部避障几何并没有按比例缩小。

## 4. 当前 3.5 m × 3.5 m 场景假设

当前仿真/模型适配按以下约定工作：

```text
场地尺寸：3.5 m × 3.5 m
场地中心：(0, 0)
x 范围：[-1.75, 1.75] m
y 范围：[-1.75, 1.75] m
归一化：x_net = x_world / 1.75
          y_net = y_world / 1.75
```

所以 `1.75` 的含义是：

```text
1.75 m = 3.5 m / 2
```

它是训练场景尺度/网络输入归一化参数，不是相机参数，也不是 e-puck2 的机械参数。

项目中六个 e-puck 策略配置目前都把 `world_scale` 设置为 `1.75`：

- `transport_pattern.yaml`
- `transport_pattern_neat.yaml`
- `roundflocking_pattern.yaml`
- `roundflocking_pattern_neat.yaml`
- `navigation_pattern.yaml`
- `navigation_pattern_neat.yaml`

网络不仅使用相对坐标，部分观测还显式包含机器人绝对 `x、y`。因此场地中心或坐标轴不一致时，网络输入会整体偏移或翻转，不能只依赖网络自己适应。

## 5. 三层坐标应该如何定义

建议为每个实物场地记录一份明确的坐标约定。

### 5.1 相机/厂家原始坐标

记为：

```text
p_sensor = [x_sensor, y_sensor]
```

它可能是：

- 图像像素；
- PnP 算出的相机平面米制坐标；
- 智能佳定位系统自己的平面坐标；
- 以场地左上角、左下角或相机光心为原点的坐标。

这层坐标不能直接送入网络。

### 5.2 真实物理 world 坐标

建议定义：

```text
单位：m
原点：实测可用区域的几何中心
+x：沿场地的一条长边
+y：按右手系确定
+yaw：从 +x 向 +y 为正
```

若场地物理坐标原来以某点为原点，实测中心为 `(c_x,c_y)`，且轴方向还需要旋转 `theta`，则：

```text
p_world = R(theta) * (p_physical - [c_x, c_y])
```

如果定位系统的 y 轴向下，还需要反射。最终必须用实物沿 `+x` 行驶、沿 `+y` 移动和原地正向旋转三个测试验证符号，不能只看图像方向猜测。

### 5.3 模型输入坐标

方形场地边长为 `L`，希望把边界约归一化到 `[-1,1]` 时：

```text
world_scale = L / 2
x_net = x_world / world_scale
y_net = y_world / world_scale
```

对于 3.5 m 场地，`world_scale=1.75`；对于 2 m 场地，候选值为 `1.0`。

这一步是策略适配，不应改变安全层使用的真实米制距离。

## 6. 不同场地情况分别如何修改

### 6.1 尺寸不变，只是中心位置不同

例如实际场地仍为 3.5 m × 3.5 m，但厂家定位原点在左下角 `(0,0)`。

应做：

1. 标定或平移到场地中心；
2. 确认中心输出约为 `(0,0)`；
3. 保持 `world_scale=1.75`；
4. 重新验证 yaw 和速度方向。

理论上左下角为 `(0,0)`、右上角为 `(3.5,3.5)` 时：

```text
x_world = x_sensor - 1.75
y_world = y_sensor - 1.75
```

实际仍建议用多点拟合，因为相机通常还存在旋转、透视和尺度误差。

### 6.2 尺寸不变，但相机旋转、镜像或重新安装

应重新标定 `camera_to_world`，而不是修改 `world_scale`。

`camera_to_world` 的六个值表示：

```text
x_world = a * x_camera + b * y_camera + tx
y_world = c * x_camera + d * y_camera + ty
```

它可以同时修正：

- 平移；
- 平面旋转；
- x/y 镜像；
- 相机平面到真实米制坐标的尺度；
- 小范围的二维仿射误差。

相机高度、分辨率、镜头、安装角度、图像裁剪或定位软件输出定义变化后，都应重新标定。

### 6.3 方形场地整体缩小或放大

若真实场地为边长 `L_new` 的方形，首版可采用：

```text
物理 world 坐标：仍使用真实米
world_scale = L_new / 2
```

例如智能佳标准 2 m × 2 m 场地：

```text
x_world, y_world ≈ [-1.0, 1.0] m
world_scale = 1.0
```

与 3.5 m 场地相比，几何比例为：

```text
k = 2.0 / 3.5 ≈ 0.5714
```

只有“随场地布局一起按比例定义”的参数才可以把 `k` 当作候选比例。例如：

- 初始点和目标点与中心的距离；
- 障碍物布局间距；
- 如果任务定义也要完全等比，`target_radius=0.6 m` 可先评估约 `0.343 m`；
- 如果希望保持相似的归一化穿越时间，`0.12 m/s × k ≈ 0.0686 m/s` 可作为后期速度候选上限。

不能按 `k` 直接缩放的量包括：

- e-puck2 直径和轮距；
- 实物箱体半径；
- 机器人与箱体的接触几何；
- 由制动性能决定的停车距离；
- 定位噪声和标记尺寸；
- 只感知机器人近邻/障碍物局部几何的 Lidar 输入范围。

在 2 m 场地第一次落地时，不建议立即使用 `0.0686 m/s`，应先把最大线速度限制为约 `0.03 m/s`，完成方向、失联停车和边界验收后逐步提高。

### 6.4 长宽不同的矩形场地

若场地为 `W × H`，可以数学上定义：

```text
x_net = x_world / (W/2)
y_net = y_world / (H/2)
```

但当前代码只提供单个 `world_scale`，没有独立的 `x_scale/y_scale`。更重要的是，`W != H` 的非等比例变换会改变：

- 距离和角度；
- 圆形环绕轨迹；
- 相对向量方向；
- 机器人群体的空间密度；
- 网络看到的碰撞和目标分布。

因此不建议为了“填满 `[-1,1]²`”而把长方形强行拉成正方形。优先选择：

1. 在矩形中定义一个居中的方形有效任务区，按其半边长设置 `world_scale`；
2. 保持真实等比例坐标，用较大的半边长作统一 `world_scale`，短边的网络范围自然小于 `[-1,1]`；
3. 对新长宽比进行重新训练或至少使用场地尺寸随机化微调。

对于 RoundFlocking，强行非等比缩放尤其危险，因为圆形会在模型坐标中变成椭圆。

### 6.5 场地尺寸相同，但障碍物或任务布局变化

此时 `world_scale` 可能无需修改，但仍可能发生明显的 sim-to-real 分布偏移：

- 墙面距离和可通行宽度变化；
- 机器人密度变化；
- 目标位置分布变化；
- 箱体质量、尺寸和摩擦变化；
- Lidar 在训练中能看到的实体与实物不同。

应该比较网络实际 11/18 维观测与训练/仿真观测的分布，而不是继续调整一个全局尺度去掩盖所有差异。偏差较大时应回到仿真重新标定环境，并加入场地大小、布局、噪声、延迟和动力学随机化。

## 7. 2 m × 2 m 智能佳场地的首版参数建议

下表是假设“实际可用内尺寸确实为 2.00 m × 2.00 m”的起点。正式值必须按现场测量和实验确定。

| 项目 | 当前值/假设 | 2 m 场地首版建议 | 原因 |
| --- | ---: | ---: | --- |
| world 原点 | 场地中心 | 场地实测中心 | 保持绝对坐标语义 |
| world 范围 | `[-1.75,1.75] m` | 约 `[-1,1] m` | 使用真实物理米 |
| `world_scale` | `1.75` | `1.0` 候选 | 半边长归一化 |
| `velocity_alpha` | `0.65` | 先保持 `0.65` | 与定位噪声/频率有关，不与场地尺寸直接相关 |
| `target_radius` | `0.6 m` | 任务等比时约 `0.343 m` | 只缩放任务布局，不缩放机器人 |
| 策略最大线速度 | `0.12 m/s` | 首测约 `0.03 m/s`，验收后再评估 `0.05~0.07 m/s` | 先验证安全与制动 |
| PT Navigation Lidar 输入上限 | `0.35 m` | 初期保持 | 局部物理感知语义 |
| NEAT Navigation / Flocking Lidar 上限 | `0.20 m` | 初期保持 | checkpoint 的固定输入语义 |
| 安全停车距离 | `0.10 m` | 不按场地缩放，重新测制动后确定 | 真实物理安全量 |
| Transport 箱体半径 | `0.06 m` | 按实物测量 | 真实物体几何 |
| 配置轮距 | `0.05685 m` | 实测所购底盘后填写 | 不能由场地尺寸推导 |

需要修改 `world_scale` 的文件：

```text
src/ros2swarm/config/e_puck/movement_pattern/basic/transport_pattern.yaml
src/ros2swarm/config/e_puck/movement_pattern/basic/transport_pattern_neat.yaml
src/ros2swarm/config/e_puck/movement_pattern/basic/roundflocking_pattern.yaml
src/ros2swarm/config/e_puck/movement_pattern/basic/roundflocking_pattern_neat.yaml
src/ros2swarm/config/e_puck/movement_pattern/basic/navigation_pattern.yaml
src/ros2swarm/config/e_puck/movement_pattern/basic/navigation_pattern_neat.yaml
```

不建议直接覆盖现有 3.5 m 配置。应复制成明确命名的 `arena_2m` 配置或增加 launch 参数，保留仿真基线以便对比和回退。

### 7.1 当前 launch 对多场地配置的限制

当前两个实机 launch 的配置选择能力不同：

- `epuck2_overhead_tracking.launch.py` 已经提供 `config` 参数，可以直接选择一份新的 `marker_tracking_2m.yaml`；
- `epuck2_physical.launch.py` 会根据 `pattern` 固定加载 `basic/<pattern>.yaml`，并固定加载 `real_robot/safety.yaml`，目前没有 `pattern_config` 或 `safety_config` 参数。

所以仅复制一个 `navigation_pattern_arena_2m.yaml`，当前物理 launch **不会自动加载它**。有两种做法：

1. 临时验收时，先保存当前参数副本，再修改实际被加载的策略 YAML 和 `safety.yaml`；
2. 正式维护时，给 `epuck2_physical.launch.py` 增加显式的 `pattern_config`、`safety_config` 或 `arena_profile` 参数。

第二种更适合同时保留 3.5 m 仿真、2 m 实物以及其他定制场地，避免操作人员每次手工覆盖文件。本文只增加说明文档，尚未修改该 launch 的配置选择逻辑。

2 m 相机配置可以先按现有接口单独启动：

```bash
ros2 launch ros2swarm epuck2_overhead_tracking.launch.py \
  config:=/绝对路径/marker_tracking_2m.yaml
```

第一次低速实验若把策略 `max_linear_velocity` 降为 `0.03`，也建议同时把当前实验安全配置的 `max_linear_velocity` 降为 `0.03`，形成第二层限速。这是试验安全措施，不是按场地比例推导出的永久机械参数。

## 8. 速度输入和 `0.65/0.35` 的含义

当前位置跟踪器按以下方式计算速度：

```text
raw_vx = (x_now - x_previous) / dt
raw_vy = (y_now - y_previous) / dt

vx = alpha * raw_vx + (1 - alpha) * previous_vx
vy = alpha * raw_vy + (1 - alpha) * previous_vy
```

默认 `alpha=0.65`，所以代码中出现：

```text
vx = 0.65 * raw_vx + 0.35 * previous_vx
vy = 0.65 * raw_vy + 0.35 * previous_vy
```

两者之和必须为 1；当前 `0.35` 是 `1-0.65`，不是另一个独立参数。

调节规律：

- `alpha` 较大：更跟随当前测量，响应快，但位置抖动会被放大成速度噪声；
- `alpha` 较小：速度更平滑，但启动、转向和停止会有更大延迟；
- 场地只是变大或变小，不构成修改 `alpha` 的理由；
- 相机帧率、曝光、运动模糊、坐标抖动或定位系统更换，才需要重新评估。

建议标定方法：

1. 静止采集 30~60 s，检查 `vx、vy` 的均值、标准差和峰值；
2. 分别以已知低速直行 `+x/-x/+y/-y`，比较速度方向和幅值；
3. 急停后记录速度估计衰减到接近 0 所需时间；
4. 检查偶发丢帧后是否产生异常速度尖峰；
5. 同时记录厂家速度、轮式里程计和视觉差分速度，比较延迟与漂移；
6. 最后根据网络输入分布和安全响应选择 `alpha`。

如果定位帧率不稳定，固定 `alpha` 的等效滤波时间会随帧率变化。后续可考虑改成基于时间常数 `tau` 的形式：

```text
alpha(dt) = 1 - exp(-dt / tau)
```

当前代码还没有速度离群值剔除、最大合理速度检查或视觉/里程计融合。真实部署时至少应记录异常跳变，避免单帧误检产生极大速度输入。

### 8.1 场地缩放后，速度是否也要缩放

这取决于具体模型：

- PT Transport 和 PT Navigation 当前 `velocity_scale<=0`，会让速度与位置共用 `world_scale`。
- PT RoundFlocking 当前 `velocity_scale=1.0`，本体速度不会跟 `world_scale` 同比例变化。
- 原生 NEAT 的若干本体速度维度直接使用 m/s，也不会自动按 `world_scale` 缩放。

因此把 `world_scale` 从 `1.75` 改为 `1.0` 后，不能简单声称所有网络的速度语义都已经同步适配。每个 checkpoint 都必须分别进行观测回放测试。若希望统一控制速度尺度，最好新增显式的 `velocity_scale` 并与训练代码对齐，而不是隐式复用位置尺度。

## 9. 为什么不应在 `camera_to_world` 中直接伪造 3.5 m 场地

一种看似方便的做法是把真实 2 m 坐标放大 `1.75` 倍，让网络继续看到 `[-1.75,1.75]`，并保持 `world_scale=1.75`。

当前代码中不推荐这样做，因为：

1. `marker_state_fuser` 先应用 `camera_to_world`，再对变换后的位置差分，所以速度也会被放大；
2. `/global_state` 会失去“真实米”的含义；
3. `safety_supervisor` 同时用 `/global_state` 与真实 `/scan_raw` 判断 Transport 接触几何；
4. 箱体半径、接触距离、制动距离和 Lidar 仍是真实米，两套尺度会相互冲突。

推荐的长期数据流是：

```text
定位设备
  -> /global_state_physical（真实 m、m/s，供安全层）
       ├─> safety_supervisor
       └─> policy coordinate adapter
             -> /global_info（训练/模型等效坐标）
```

也就是说：

- `camera_to_world` 只负责把相机输出标成真实物理 world；
- 独立策略适配器负责平移、等比例缩放和网络归一化；
- 物理安全阈值永远不进入模型等效坐标。

当前 `global_info_bridge.py` 没有上述可配置变换。若 2 m 首版仅把物理坐标中心化并修改各策略 `world_scale`，可以不改桥；如果要保留多个场地配置、把 2 m 场地映射成训练场景或支持不同 x/y 尺度，则应先扩展桥或增加独立适配节点。

## 10. 顶视相机的实际标定流程

### 10.1 测量有效区域

首先测量机器人中心实际可到达的区域，而不只是桌面外框：

```text
W_usable = 左右障碍内侧距离
H_usable = 上下障碍内侧距离
```

同时记录：

- 场地四角和几何中心；
- 墙体厚度、高度和反光情况；
- 机器人中心到墙的最小可达距离；
- 相机高度、分辨率和固定方式；
- 标记实际尺寸、标记中心到机器人几何中心的偏移；
- 目标区和运输箱体的真实尺寸。

### 10.2 定义标定点

以实测场地中心为 `(0,0)`。对于 `W × H` 场地，四个理论角点可写为：

```text
(-W/2, -H/2)
( W/2, -H/2)
( W/2,  H/2)
(-W/2,  H/2)
```

实际标定建议使用四角、四边中点和中心等至少 9 个点，并避开挡板遮挡。点数越多，越容易发现镜头畸变或局部误差。

CSV 格式为：

```text
camera_x,camera_y,world_x,world_y
```

运行：

```bash
cd /root/workspace/ROS2swarm_w
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/calibrate_camera_to_world.py arena_points.csv
```

把输出的：

```yaml
camera_to_world: [a, b, tx, c, d, ty]
```

写入单独的实现场地配置文件。`rmse_m` 应按定位精度要求评估；不仅要看整体 RMSE，还要逐点检查场地边缘的最大误差。

### 10.3 无电机验收

在允许车轮落地前完成：

1. 把标记放在中心，检查坐标接近 `(0,0)`；
2. 放到各角点和边中点，检查位置误差；
3. 沿 `+x` 移动，检查 `x` 增大且 `vx>0`；
4. 沿 `+y` 移动，检查 `y` 增大且 `vy>0`；
5. 机器人朝向 `+x`，检查 `yaw≈0`；
6. 正向旋转，检查 yaw 符号；
7. 保持静止，检查速度噪声；
8. 遮挡标记，检查状态超时后策略和底盘停车。

## 11. 使用智能佳高频定位系统时如何接入

先确认厂家输出是否包含：

```text
robot_id
x, y
yaw
vx, vy（如果有）
source timestamp
confidence/valid（如果有）
```

还要确认：

- 坐标和速度单位；
- 原点位置和 x/y 方向；
- yaw 零方向、正方向和范围；
- 位置是机器人中心还是标记/投影点；
- 数据频率、延迟、丢帧行为；
- 多机器人 ID 是否稳定；
- 是否有 ROS 2 驱动、UDP/TCP/串口或 SDK 接口。

有两种接法：

### 11.1 复用当前状态融合接口

编写厂家数据适配节点，发布 `/marker_detections`，继续使用：

```text
marker_state_fuser -> /global_state -> global_info_bridge
```

优点是改动小；缺点是如果厂家已经提供速度，当前 `MarkerDetectionArray`/fuser 路径仍会重新差分位置，不能直接利用厂家速度。

### 11.2 直接发布 `/global_state`

编写适配节点直接发布 `TrackedEntityArray`：

- 厂家位置先变换为中心化的物理米制 world；
- 厂家速度可靠时直接发布，并正确设置 `twist_valid`；
- 没有速度时再使用当前差分滤波逻辑；
- 厂家时间戳应保留为 `source_stamp`。

这种方式更适合完整定位系统，但启动文件必须避免再启动另一个 `marker_state_fuser` 同时发布 `/global_state`。当前 overhead launch 的组件开关需要根据最终接口检查，不能只关闭图像 detector 后保留冲突的 fuser。

## 12. 网络输入和安全参数的修改边界

### 12.1 可以随场地配置修改

- 相机到物理 world 的标定矩阵；
- 场地中心和坐标轴约定；
- `world_scale`；
- 初始点、目标点和障碍物布局；
- 任务定义中的环绕半径；
- 经过实测确定的最大任务速度；
- 与定位频率/噪声有关的滤波参数。

### 12.2 必须按实物测量，不能由场地比例推导

- 轮距、轮径和轮速上限；
- 箱体半径、机器人半径和标记偏移；
- 制动距离、接触距离和碰撞容差；
- 电机死区、加速度、角速度和推箱速度；
- 定位误差、延迟和超时；
- Lidar 安装偏角、盲区和有效量程。

### 12.3 checkpoint 固定语义，修改前必须验证或重训

- 11/18 维输入顺序；
- 世界坐标与本体坐标的区别；
- PT/NEAT 各自的速度缩放；
- 12 路 Lidar 的方向和数值含义；
- Navigation 的 `0.35-d` 或 `0.20-d`；
- Flocking 的 `[0,0.20] m` 原始距离；
- 输出动作到差速底盘的映射。

## 13. 推荐的 2 m 场地部署顺序

### 阶段 A：只做测量和坐标

1. 实测 `W_usable、H_usable`；
2. 明确场地中心、`+x、+y、+yaw`；
3. 获取厂家定位接口说明和一段原始数据；
4. 确认加装激光雷达的型号、驱动和 `/scan_raw` 距离语义；
5. 测量轮距、箱体和标记偏移。

### 阶段 B：完成定位但不运行网络

1. 重新标定 `camera_to_world` 或厂家坐标变换；
2. 发布 `/global_state`；
3. 验证中心、四角、朝向和速度；
4. 验证遮挡、丢帧和超时；
5. 录制 rosbag 作为定位基线。

### 阶段 C：离线影子推理

1. 将 2 m 场地策略副本的 `world_scale` 设为 `1.0`；
2. 网络读取真实观测，但禁止输出到电机；
3. 记录完整 11/18 维向量、网络原始输出和输入有效性；
4. 与 3.5 m 仿真基线比较每一维的范围、均值、方差和极值；
5. 分别测试 PT 与 NEAT，不能只测其中一个后推断另一个。

### 阶段 D：抬轮与低速落地

1. 抬轮确认前进、后退和旋转方向；
2. 验证 STOP、定位丢失、Lidar 丢失、Wi-Fi 中断和节点退出都能停车；
3. 初始最大线速度约设为 `0.03 m/s`；
4. 单机器人、空场开始；
5. 再增加墙边、目标、其他机器人和 Transport 箱体；
6. 每次只改变一个变量，并保存 rosbag 和参数版本。

### 阶段 E：决定继续适配还是重训

以下情况优先考虑重新训练/微调，而不是继续堆参数：

- 实际场地长宽比明显不同；
- 机器人/障碍物密度与训练差异大；
- 加装激光雷达的回波、盲区或可见实体与仿真不同；
- 真实观测经归一化后仍明显超出训练分布；
- 箱体摩擦、动力学和多机器人推力与仿真差异大；
- 需要在多个尺寸场地通用运行。

训练时建议加入：

- 场地尺寸和中心随机化；
- 初始点、目标点和障碍物布局随机化；
- 定位噪声、丢帧、延迟和速度估计噪声；
- Lidar 噪声、遮挡、盲区和安装偏角；
- 轮胎打滑、电机死区、动作延迟和速度饱和；
- 机器人、箱体尺寸和摩擦范围。

## 14. 最终验收清单

- [ ] 已测量实际可用内尺寸，不仅使用商品标称尺寸；
- [ ] 所有实体共用唯一、中心化、米制的 `world` 坐标；
- [ ] 场地中心、四角、x/y 方向和 yaw 符号均已实测；
- [ ] `camera_to_world` 是本次相机安装的标定值，不是占位值；
- [ ] `world_scale` 与本场地/本 checkpoint 的归一化约定一致；
- [ ] 物理安全层未使用被人为放大的模型等效坐标；
- [ ] `vx、vy` 来源、单位、滤波、时间戳和有效性明确；
- [ ] `velocity_alpha` 用静止和运动数据验证，而不是因为换场地机械修改；
- [ ] 12 路输入来自已加装激光雷达的 `/scan_raw`，且方向、单位和截断语义正确；
- [ ] PT/NEAT 的 11/18 维输入分别做过离线回放；
- [ ] 停车距离、轮距、箱体尺寸和接触窗口均按实物测量；
- [ ] 定位/Lidar/命令/DDS 任一失效都会使底盘停车；
- [ ] 先经过影子推理、抬轮测试和 `0.03 m/s` 低速测试；
- [ ] 参数和 rosbag 已按场地版本保存，可恢复到 3.5 m 仿真基线。

## 15. 资料来源

以下资料用于确认厂家设备和标准场地的公开规格，实际交付版本以采购合同、铭牌和厂家接口文档为准：

- 智能佳：[E-PUCK2 教育机器人及扩展介绍](https://www.cnblogs.com/bjrobot/articles/17356711.html)
- 智能佳：[2 m × 2 m 群体智能测试场地介绍](https://www.cnblogs.com/bjrobot/articles/13671599.html)
- 智能佳：[e-puck2 高频投影定位系统介绍](https://www.cnblogs.com/bjrobot/p/13963549.html)
- GCtronic：[e-puck2 官方规格页 PDF](https://projects.gctronic.com/epuck2/e-puck2-flyer.pdf)

官方 e-puck2 资料列出的基础底盘直径约 70 mm、高约 45 mm、轮距约 53 mm、最大速度约 15.4 cm/s。项目当前安全配置的 `wheel_separation=0.05685 m` 与该公开基础规格不完全相同，所以必须以手头所购机器人、轮胎和扩展结构的实测值为准。

更完整的代码启动、硬件驱动和安全验收流程见：

- [EPUCK2_REAL_DEPLOYMENT_GUIDE.md](EPUCK2_REAL_DEPLOYMENT_GUIDE.md)
- [PT_NEAT_REAL_ROBOT_DEPLOYMENT_GUIDE.md](PT_NEAT_REAL_ROBOT_DEPLOYMENT_GUIDE.md)
