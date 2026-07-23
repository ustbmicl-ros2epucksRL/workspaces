# e-puck2 ROS 2 实机部署、标定与验收指南

> 文档日期：2026-07-19  
> 目标平台：e-puck2，机器人端运行 ROS 2，扩展 360 度 Lidar，机器人及任务物体通过顶视二维码定位。  
> 主项目：`/root/workspace/ROS2swarm_w`。  
> 架构参考：[ROS2swarm/ROS2swarm](https://github.com/ROS2swarm/ROS2swarm/tree/master)。  

## 1. 当前实现达到的状态

当前项目已经具备一套与 Webots 并行、可在无实物条件下运行的 e-puck2 实机软件链：

```text
顶视相机
  -> QR/ArUco 检测 /marker_detections
  -> 状态融合 /global_state
  -> 兼容桥 /global_info

360 度 Lidar
  -> /robot_N/scan_raw
  -> 12 方向策略适配 /robot_N/scan

PT/NEAT 策略
  -> /robot_N/drive_command
  -> 实机安全 supervisor
  -> /robot_N/cmd_vel
  -> e-puck2/Pi-puck 驱动
```

已经在当前机器验证的内容：

- 新消息生成、Python 包安装和 ROS 2 launch 解析；
- 普通 QR 与 ArUco 检测分支、PnP 位姿估计及退化解回退；
- 相机平面到世界坐标的仿射变换和标定拟合；
- 状态速度估计、遮挡过期、typed state 到旧 `/global_info` JSON 的转换；
- 任意合法 360 度 `LaserScan` 到网络所需 12 路输入的角度感知采样；
- PT/NEAT 实机模式的数据超时停车、目标到达停车和 recurrent state 重置；
- 速度、角速度、轮速、加速度、障碍物、STOP、急停和数据 watchdog；
- Transport 的定位约束窄角度接触窗口，其余障碍仍触发停车；
- 六个模式的完整无硬件 ROS 图：Transport、RoundFlocking、Navigation 的 PT 和 NEAT；
- GCtronic e-puck2 驱动固定提交的主机编译和 11 项 `--type test`；
- SLAMTEC RPLidar ROS 2 固定提交的主机编译；
- 原 Webots launch 仍可解析，实机参数默认关闭，不改变仿真策略行为。

没有实物时无法完成最终确认的内容：

- e-puck2 轮径、轮距、电机死区、制动距离和负载能力；
- Lidar 型号、波特率、安装偏角、盲区和供电稳定性；
- 真实二维码尺寸、相机曝光、运动模糊、遮挡和端到端定位延迟；
- Pi-puck 上的 CPU、内存、温度、Wi-Fi 和长时间运行稳定性；
- Transport 推箱时的接触几何、摩擦、多机器人受力和安全距离。

因此当前版本可以称为“无硬件闭环验证完成的实机候选版本”，不能称为“实物验收完成”。

## 2. 代码入口和文件说明



### 2.1 实机启动入口


| 文件                                                        | 用途                                   |
| --------------------------------------------------------- | ------------------------------------ |
| `src/ros2swarm/launch/epuck2_physical.launch.py`          | 单台 e-puck2 的驱动、Lidar、策略和安全链，可按机器拆分组件 |
| `src/ros2swarm/launch/epuck2_overhead_tracking.launch.py` | 顶视二维码检测、状态融合和 `/global_info` 兼容桥     |
| `src/ros2swarm/launch/epuck2_mock_chain.launch.py`        | 无硬件完整链路                              |
| `scripts/validate_epuck2_mock_chain.py`                   | 六模式自动链路和故障注入验收                       |
| `scripts/calibrate_camera_to_world.py`                    | 相机平面到世界平面的仿射标定                       |




### 2.2 配置


| 文件                                              | 用途                                |
| ----------------------------------------------- | --------------------------------- |
| `config/e_puck/real_robot/marker_tracking.yaml` | QR 内容、实体名称、标记偏移、相机到世界变换           |
| `config/e_puck/real_robot/safety.yaml`          | watchdog、限速、轮距、避障和 Transport 接触窗口 |
| `config/e_puck/real_robot/mock_tracking.yaml`   | 无硬件测试状态映射                         |




### 2.3 外部硬件依赖


| 文件                                                      | 用途                                                 |
| ------------------------------------------------------- | -------------------------------------------------- |
| `dependencies/epuck2_hardware.repos`                    | 固定版本的 GCtronic e-puck2 和 RPLidar ROS 2 仓库          |
| `dependencies/patches/epuck_ros2_driver_watchdog.patch` | e-puck2 驱动本地 watchdog、namespace/TF 和 Humble 测试兼容补丁 |




## 3. 策略输入契约

详细逐维说明见 [PT_NEAT_REAL_ROBOT_DEPLOYMENT_GUIDE.md](PT_NEAT_REAL_ROBOT_DEPLOYMENT_GUIDE.md)。部署前必须记住以下不可变契约：


| 模式                    | 输入维度 | 必需信息                       |
| --------------------- | ---- | -------------------------- |
| Transport PT/NEAT     | 11   | 机器人位姿/速度、运输物体位姿/速度、目标区位置   |
| RoundFlocking PT/NEAT | 18   | 机器人位姿/速度、环绕目标位置、12 路 Lidar |
| Navigation PT/NEAT    | 18   | 机器人位姿/速度、专属目标位置、12 路 Lidar |


不得只保证维度相同。还必须保持输入顺序、坐标方向、世界/本体坐标系、量纲、归一化和 Lidar 数值语义。PT 与 NEAT 在部分坐标表达和输出映射上不同，不能互换观测构造器。

## 4. 推荐部署拓扑

首版推荐拆成中央计算机和机器人本机两层：

```text
x86 中央机
  顶视相机 + QR 检测 + 状态融合
  PT/NEAT 策略
               Wi-Fi / ROS 2 DDS
Pi-puck/e-puck2
  e-puck2 I2C 驱动
  360 Lidar 驱动
  12 路扫描适配器
  safety_supervisor
```

安全层必须靠近底盘运行。中央机、策略或 Wi-Fi 失联时，Pi-puck 上的 supervisor 会停止发布非零速度；打过补丁的 e-puck2 驱动在最终 `cmd_vel` 消失 0.25 秒后还会再次独立归零。

NEAT 网络较小，将来可以下沉到 Pi-puck。PT 首版建议留在 x86，只有在实测推理延迟、内存和温度合格后再下沉。

## 5. 系统和网络准备

下列命令以 ROS 2 Humble、Ubuntu 22.04 为例。机器人已有 ROS 2 时仍需确认发行版和架构一致。

```bash
printenv ROS_DISTRO
uname -m
timedatectl status
```

中央机和 Pi-puck 必须满足：

- 相同 `ROS_DOMAIN_ID`；
- 同一可互通网络，关闭会阻断 DDS multicast/unicast 的防火墙规则；
- 使用 NTP/chrony 同步时间；
- 主机名和 IP 稳定；
- STOP 和急停不依赖图形界面。

示例：

```bash
export ROS_DOMAIN_ID=42
ros2 multicast receive
ros2 multicast send
```

多机发现不稳定时，应为所用 RMW 配置显式 peer，不要通过增大 watchdog 来掩盖网络问题。

## 6. 构建主项目

```bash
cd /root/workspace/ROS2swarm_w
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build \
  --packages-select communication_interfaces ros2swarm \
  --symlink-install
source install/setup.bash
```

在 Pi-puck 上也需要部署并构建这两个包，因为扫描适配器和安全 supervisor 在机器人本机运行。PT/NEAT checkpoint 可以随包安装，但当 `start_policy:=false` 时不会加载网络。

## 7. 构建 e-puck2 和 Lidar 驱动

创建独立硬件 overlay，不要把外部驱动复制进策略包：

```bash
mkdir -p ~/epuck2_hw_ws/src
cd ~/epuck2_hw_ws
vcs import src < /root/workspace/ROS2swarm_w/dependencies/epuck2_hardware.repos
git -C src/epuck_ros2 submodule update --init --recursive
```

应用固定提交补丁：

```bash
git -C src/epuck_ros2 apply --check \
  /root/workspace/ROS2swarm_w/dependencies/patches/epuck_ros2_driver_watchdog.patch
git -C src/epuck_ros2 apply \
  /root/workspace/ROS2swarm_w/dependencies/patches/epuck_ros2_driver_watchdog.patch
```

补丁提供：

- `command_timeout=0.25` 的驱动内电机归零；
- `odom_frame`、`base_frame`、`proximity_laser_frame` 和 `frame_prefix`；
- LED、地面传感器话题进入机器人 namespace；
- ROS 2 Humble launch test API 兼容；
- watchdog 回归测试。

安装 Pi-puck 镜像要求的 `wiringPi` 和 I2C 依赖后构建：

```bash
cd ~/epuck2_hw_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

在 Pi-puck 上先执行上游测试后端：

```bash
colcon test --packages-select epuck_ros2_driver
colcon test-result --verbose
```

当前 x86 已用 host stub 验证补丁可编译且 11 项 `--type test` 全部通过。host stub 只用于无硬件 CI，不能复制到机器人代替真实 `wiringPi`。

## 8. 确认 e-puck2 驱动

GCtronic 驱动面向 e-puck2 + Pi-puck，通过 `/dev/i2c-12` 访问硬件。先检查：

```bash
ls -l /dev/i2c-12
i2cdetect -y 12
ros2 pkg executables epuck_ros2_driver
```

抬起车轮进行第一次测试：

```bash
ros2 run epuck_ros2_driver driver \
  --ros-args \
  -r __ns:=/robot_0 \
  -p command_timeout:=0.25 \
  -p odom_frame:=robot_0/odom \
  -p base_frame:=robot_0/base_link \
  -p proximity_laser_frame:=robot_0/proximity_laser \
  -p frame_prefix:=robot_0/
```

另一个终端只发一次低速命令：

```bash
ros2 topic pub --once /robot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.0}}"
```

车轮应短暂转动并在 0.25 秒左右自动停止。若持续转动，禁止落地测试，说明补丁未生效或启动了错误的驱动 overlay。

## 9. 配置新增 360 度 Lidar

e-puck2 驱动自身发布的 `scan` 是红外 + ToF 稀疏扫描，不是新增的 360 度 Lidar。实机 launch 会将其重映射为：

```text
/robot_0/proximity_scan
```

新增 Lidar 必须发布：

```text
/robot_0/scan_raw  sensor_msgs/msg/LaserScan
```

若是 SLAMTEC RPLidar，可由实机入口直接启动。先根据铭牌确定型号和波特率，不能盲用默认值。常见值仅供核对：A1/A2M8 常见 115200，部分 A2/A3 为 256000，S2 可为 1000000；最终以对应型号官方 launch 为准。

检查串口：

```bash
ls -l /dev/serial/by-id/
udevadm info --query=all --name=/dev/ttyUSB0
```

生产环境应使用 `/dev/serial/by-id/...` 或自定义 udev 名称，不要依赖可能变化的 `/dev/ttyUSB0`。

启动前确认以下参数：

- `lidar_serial_port`：稳定串口路径；
- `lidar_baudrate`：与型号一致；
- `lidar_scan_mode`：空字符串表示驱动典型模式，也可填型号支持的模式；
- `lidar_x/y/z`：Lidar 原点相对 `base_link` 的米制安装位置；
- `lidar_sensor_yaw`：Lidar 0 度方向相对机器人前方的弧度偏角。

用正前方墙面校验角度：

```bash
ros2 topic hz /robot_0/scan_raw
ros2 topic echo --once /robot_0/scan_raw
ros2 run tf2_ros tf2_echo robot_0/base_link robot_0/laser_frame
```

策略适配器输出固定 12 路：

```bash
ros2 topic echo --once /robot_0/scan --field ranges
```

第 0 路必须对应机器人正前方，后续依次为 30、60、...、330 度。物理量程不能替代网络训练上限：RoundFlocking/NEAT Navigation 使用 0.20 m，PT Navigation 使用 0.35 m。

如果不是 RPLidar：

1. 令 `start_lidar:=false`；
2. 使用厂商 ROS 2 驱动；
3. 将输出 remap 到 `/robot_N/scan_raw`；
4. 保留 `angle_min`、`angle_increment`、正负方向和无效值语义；
5. 继续使用本项目 scan adapter 和 safety supervisor。



## 10. 顶视 QR 定位配置

默认生产配置使用 OpenCV 普通 QR 后端。建议二维码正文直接使用以下语义键：

```text
robot_0
navigation_target_0
flocking_target
transport_box
target_zone
```

对应配置位于 `marker_tracking.yaml`：

```yaml
backend: 'qr'
marker_keys: ['robot_0', 'navigation_target_0', 'flocking_target', 'transport_box', 'target_zone']
entity_names: ['robot_0', 'navigation_target_0', 'flocking_target', 'transport_box', 'target_zone']
entity_types: [0, 2, 2, 1, 2]
```

若已有二维码正文是数字或其他文本，只改 `marker_keys`，不要改策略需要的 `entity_names`。若实际贴的是 ArUco，则改为：

```yaml
backend: 'aruco'
aruco_dictionary: 'DICT_5X5_250'
marker_keys: ['0', '100', '101', '102', '103']
```



### 10.1 相机内参

相机必须发布 `Image` 和匹配的 `CameraInfo`：

```text
/overhead_camera/image_raw
/overhead_camera/camera_info
```

使用 ROS camera calibration 完成焦距、主点和畸变标定。改变分辨率、变焦、焦距或裁剪后要重新标定。

`marker_size` 是二维码外边长，单位米。尺寸错误会按比例污染 PnP 平移结果。

### 10.2 世界平面标定

在场地放置至少 4 个不共线标定点。记录 `/marker_detections` 中的相机平面 `x,y`，并实测对应世界 `x,y`，建立 CSV：

```csv
camera_x,camera_y,world_x,world_y
-0.82,-0.51,0.00,0.00
0.79,-0.50,1.60,0.00
-0.81,0.52,0.00,1.00
0.80,0.51,1.60,1.00
```

拟合：

```bash
python3 scripts/calibrate_camera_to_world.py camera_world_pairs.csv
```

输出示例：

```text
camera_to_world: [a, b, tx, c, d, ty]
rmse_m: 0.0042
linear_determinant: -0.98
```

将六个数写入 `marker_tracking.yaml`。`determinant < 0` 可以是顶视光学坐标 y 轴反向的正常镜像；接近 0 表示点退化。建议定位 RMSE 小于 10 mm，具体阈值需小于任务可接受误差。

### 10.3 标记中心偏移

`marker_offsets` 每个实体三个数：

```text
[marker_to_entity_x, marker_to_entity_y, marker_to_entity_yaw]
```

偏移在二维码自身坐标系中定义。如果二维码中心不等于机器人旋转中心，必须实测填写，否则转弯时会产生虚假平移和速度尖峰。

### 10.4 定位验收

```bash
ros2 launch ros2swarm epuck2_overhead_tracking.launch.py
ros2 topic hz /marker_detections
ros2 topic hz /global_state
ros2 topic echo --once /global_info
```

检查：

- 静止实体位置抖动；
- 机器人朝向正负号；
- 沿世界 +x/+y 移动时坐标方向；
- 遮住二维码后 0.30 秒内 `pose_valid=false`；
- 恢复后没有米级跳变或异常速度；
- 所有机器时间同步，消息 Header 不为零。



## 11. 无硬件回归

每次修改实机配置或策略后先执行：

```bash
cd /root/workspace/ROS2swarm_w
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q \
  src/ros2swarm/test/test_real_robot_geometry.py \
  src/ros2swarm/test/test_real_robot_safety.py \
  src/ros2swarm/test/test_real_robot_policy_mode.py \
  src/ros2swarm/test/test_real_robot_state.py \
  src/ros2swarm/test/test_global_info_bridge.py \
  src/ros2swarm/test/test_fiducial_detector.py \
  src/ros2swarm/test/test_roundflocking_neat_safety.py
python3 scripts/validate_epuck2_mock_chain.py --all-patterns
```

验证器会对六个模式逐一检查：

- 所有关键 topic 存在；
- START 前零速；
- 策略命令到安全输出再到 mock odom；
- command timeout；
- scan timeout；
- state timeout；
- 急停；
- STOP；
- 最终电机命令为零。



## 12. 启动实机



### 12.1 推荐的分布式启动

终端 A，中央机启动顶视定位：

```bash
source /opt/ros/humble/setup.bash
source /root/workspace/ROS2swarm_w/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ros2swarm epuck2_overhead_tracking.launch.py
```

终端 B，Pi-puck 启动底盘、Lidar、扫描适配和本地安全，不启动策略：

```bash
source /opt/ros/humble/setup.bash
source ~/epuck2_hw_ws/install/setup.bash
source ~/ROS2swarm_w/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ros2swarm epuck2_physical.launch.py \
  robot_id:=0 \
  pattern:=navigation_pattern \
  start_driver:=true \
  start_lidar:=true \
  start_scan_adapter:=true \
  start_policy:=false \
  start_safety:=true \
  lidar_serial_port:=/dev/serial/by-id/YOUR_LIDAR \
  lidar_baudrate:=115200 \
  lidar_sensor_yaw:=0.0
```

终端 C，中央机只启动策略：

```bash
source /opt/ros/humble/setup.bash
source /root/workspace/ROS2swarm_w/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ros2swarm epuck2_physical.launch.py \
  robot_id:=0 \
  pattern:=navigation_pattern \
  start_driver:=false \
  start_lidar:=false \
  start_scan_adapter:=false \
  start_policy:=true \
  start_safety:=false
```

将 `pattern` 换成以下任意值：

```text
transport_pattern
transport_pattern_neat
roundflocking_pattern
roundflocking_pattern_neat
navigation_pattern
navigation_pattern_neat
```

终端 D，确认状态后发送 START：

```bash
ros2 topic echo --once /robot_0/safety_state
ros2 topic pub --once /swarm_command \
  communication_interfaces/msg/Int8Message "{data: 1}"
```

STOP：

```bash
ros2 topic pub --once /swarm_command \
  communication_interfaces/msg/Int8Message "{data: 0}"
```

急停：

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

解除软件急停不会自动 START：

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"
```



### 12.2 单机启动

若 PT 推理、定位、驱动和 Lidar 全部在同一台机器，可先启动顶视定位，再运行：

```bash
ros2 launch ros2swarm epuck2_physical.launch.py \
  robot_id:=0 \
  pattern:=navigation_pattern \
  start_driver:=true \
  start_lidar:=true
```

默认会同时启动策略、扫描适配和安全层。

## 13. 上电后的分阶段验收



### 阶段 1：车轮悬空

1. 不启动策略，只启动 driver 和 safety；
2. 未收到 START 时 `/robot_0/cmd_vel` 必须持续为零；
3. 手工发一次 0.02 m/s，确认驱动 watchdog 自动停车；
4. 杀死 supervisor，确认驱动仍在 0.25 秒内停车；
5. 验证实体急停按钮或断电手段可触达。



### 阶段 2：Lidar 静态场景

1. 正前、左、后、右依次放置平面障碍；
2. 验证 raw scan 的角度和距离；
3. 验证 12 路输出索引；
4. 验证拔掉 Lidar 或停止节点后 `scan_timeout`；
5. 检查 Lidar 电机电流和 e-puck2 供电是否有压降。



### 阶段 3：二维码定位

1. 手推机器人走已知直线和方形；
2. 检查 x、y、yaw 和速度方向；
3. 遮挡二维码，确认 `state_timeout`；
4. 记录 rosbag，计算更新率、延迟、抖动和丢帧；
5. 未达到定位指标前不启动闭环运动。



### 阶段 4：低速落地

临时把 `safety.yaml` 中最大线速度降到 0.03 m/s、角速度降到 0.3 rad/s。在无遮挡开阔场地逐项验证：

- 直行方向；
- 原地左右转；
- 轮速符号；
- STOP、急停、Wi-Fi 断开、相机遮挡、Lidar 断开；
- `cmd_vel`、编码器里程计和顶视位移是否一致。



### 阶段 5：Navigation

1. 目标先放在机器人正前 0.3 m；
2. 确认到达阈值后持续零速；
3. 再测试侧后方目标和静态障碍；
4. 逐步扩大场地，不要先提高速度；
5. 对 PT 和 NEAT 分别验收，不以一个版本代替另一个。



### 阶段 6：RoundFlocking

1. 单机器人先验证目标方位和环半径；
2. 再加入第二台机器人；
3. 检查其他机器人在 Lidar 中的回波与训练分布差异；
4. 观察目标定位误差是否引起高频转向；
5. 多机器人 TF frame 和 namespace 必须唯一。



### 阶段 7：Transport

Transport 最后验收，因为它要求主动接触物体。当前安全层只在以下条件同时满足时屏蔽窄角度回波：

- `task_mode=transport`；
- 机器人和 `transport_box` 二维码状态有效；
- 物体在机器人前方指定角度内；
- 物体中心距离小于 `transport_contact_distance`；
- Lidar 回波距离与“物体中心距离减物体半径”一致；
- 命令为向前运动。

进入接触区后线速度还会限制到 `max_transport_contact_linear_velocity`。必须实测调整：

```yaml
transport_contact_distance: 0.22
transport_box_radius: 0.06
transport_contact_bearing_deg: 25.0
transport_contact_mask_deg: 12.0
transport_contact_range_tolerance: 0.07
max_transport_contact_linear_velocity: 0.05
```

不要通过扩大 mask 角度来解决定位不准；这会把真实障碍错误当成运输物体。物体二维码丢失时接触豁免会立即失效并停车，这是预期行为。

## 14. 安全参数说明

默认值位于 `safety.yaml`：


| 参数                          | 默认值        | 含义                 |
| --------------------------- | ---------- | ------------------ |
| `cmd_timeout`               | 0.25 s     | 策略命令过期停车           |
| `state_timeout`             | 0.30 s     | 顶视状态过期停车           |
| `scan_timeout`              | 0.30 s     | Lidar 过期停车         |
| `obstacle_stop_distance`    | 0.10 m     | 前向/运动走廊停车距离        |
| `max_linear_velocity`       | 0.12 m/s   | 最终线速度上限            |
| `max_angular_velocity`      | 1.0 rad/s  | 最终角速度上限            |
| `wheel_separation`          | 0.05685 m  | e-puck2 初始轮距参数，需实测 |
| `max_wheel_linear_velocity` | 0.1507 m/s | 单轮线速度上限            |
| `max_linear_acceleration`   | 0.40 m/s²  | 线加速度限制             |
| `max_angular_acceleration`  | 2.0 rad/s² | 角加速度限制             |


不要先放宽 timeout。若 10 Hz 策略在 0.30 秒内经常超时，应修复网络、时间同步或节点负载。

## 15. 运行时诊断

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /global_state
ros2 topic hz /global_info
ros2 topic hz /robot_0/scan_raw
ros2 topic hz /robot_0/scan
ros2 topic hz /robot_0/drive_command
ros2 topic hz /robot_0/cmd_vel
ros2 topic echo /robot_0/safety_state
ros2 topic echo --once /robot_0/odom
ros2 run tf2_tools view_frames
```

`SafetyState.reason` 常见值：


| reason            | 含义               |
| ----------------- | ---------------- |
| `swarm_stop`      | 未 START 或收到 STOP |
| `emergency_stop`  | 软件急停有效           |
| `command_timeout` | 策略节点或网络停止更新      |
| `state_timeout`   | 顶视状态过期或机器人位姿无效   |
| `scan_timeout`    | Lidar 原始扫描过期     |
| `obstacle`        | 命令运动走廊内存在过近障碍    |


多个原因用逗号组合。调试时以第一个根因修复，不要关闭 `require_state` 或 `require_scan` 规避。

## 16. rosbag 记录

每次实机验收建议至少记录：

```bash
ros2 bag record \
  /swarm_command \
  /emergency_stop \
  /marker_detections \
  /global_state \
  /global_info \
  /robot_0/scan_raw \
  /robot_0/scan \
  /robot_0/drive_command \
  /robot_0/safety_state \
  /robot_0/cmd_vel \
  /robot_0/odom \
  /tf /tf_static
```

rosbag 回放可再次验证策略，但回放时禁止连接真实电机驱动，或确保输出进入 mock namespace。

## 17. 多机器人扩展

每台机器人使用唯一 `robot_id`：

```text
/robot_0/...
/robot_1/...
```

每台 Pi-puck 分别启动本地 driver/Lidar/adapter/safety；中央机分别启动对应策略。Navigation 目标必须存在：

```text
navigation_target_0
navigation_target_1
```

在 `marker_tracking.yaml` 增加每台机器人和目标的 `marker_keys`、`entity_names`、`entity_types` 与三项 `marker_offsets`。补丁会把 e-puck2 TF frame 参数化为 `robot_N/...`，但仍应通过 `view_frames` 检查是否有重复 frame。

## 18. 与 Webots 仿真的隔离

e-puck2 实机功能与原 Webots launch/config 相互独立。隔离原则为：

- 所有实机节点放在 `ros2swarm.real_robot`；
- 所有实机 launch 以 `epuck2_` 开头；
- 所有实机配置位于 `config/e_puck/real_robot`；
- 六个策略的 `real_robot_mode` 默认是 `false`；
- Webots 不启动新的 safety supervisor，不改变原 topic 和模型输入；
- 实机入口显式把 `bypass_protection` 改为 false，并由新 supervisor 独占最终 `cmd_vel`。

不要在 Webots 启动命令中加入 `real_robot_mode:=true`，否则仿真中的随机缺失数据回退和 recurrent 生命周期会按实机 fail-stop 规则改变。

## 19. 最终验收清单

软件验收：

- [ ] 主项目构建成功；
- [ ] 25 项实机相关单元测试通过；
- [ ] 六模式 mock 故障注入通过；
- [ ] e-puck2 驱动补丁应用到正确固定提交；
- [ ] 驱动 `--type test` 包含 watchdog 并通过；
- [ ] Lidar 驱动构建成功；
- [ ] Webots launch 仍可解析和运行。

硬件验收：

- [ ] 单次 `cmd_vel` 在 0.25 秒后由驱动自动归零；
- [ ] safety supervisor 被杀死后机器人停车；
- [ ] Lidar 拔出后机器人停车；
- [ ] 二维码遮挡后机器人停车；
- [ ] Wi-Fi 断开后机器人停车；
- [ ] 急停和 STOP 均停车；
- [ ] 轮径、轮距、死区、速度和制动距离已实测；
- [ ] 相机内参和世界平面标定达到误差指标；
- [ ] Lidar 0 度、方向、安装位姿和 12 路索引正确；
- [ ] Navigation、RoundFlocking、Transport 按顺序低速验收；
- [ ] 至少一次电池全周期长时间运行测试完成。



## 20. 外部资料和固定版本

- GCtronic e-puck2 ROS 2：[https://github.com/gctronic/epuck_ros2](https://github.com/gctronic/epuck_ros2)，固定提交 `c76eb7a47fba9cbda2fb81d3e2ebd051f0f56455`；
- Cyberbotics e-puck2 ROS 2：[https://github.com/cyberbotics/epuck_ros2](https://github.com/cyberbotics/epuck_ros2)；
- SLAMTEC RPLidar ROS 2：[https://github.com/Slamtec/rplidar_ros/tree/ros2](https://github.com/Slamtec/rplidar_ros/tree/ros2)，固定提交 `24cc9b6dea97e045bda1408eaa867ce730fd3fc3`；
- JMU `ros2_aruco` Humble：[https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/humble](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/humble)；
- PAL Robotics `aruco_ros` Humble：[https://github.com/pal-robotics/aruco_ros/tree/humble-devel](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)。

真正上机前还必须确认 Lidar 的品牌和具体型号。若不是 RPLidar，保留本项目统一 `/scan_raw` 契约，替换硬件驱动即可，不要改策略节点。