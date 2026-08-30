# e-puck2 实物部署现场流程

适用配置：2 m x 2 m 场地、3 台 e-puck2 + Pi-puck、每台一个 360 度雷达、中央顶视相机、ROS 2 Humble。策略优先使用 PT 版本，并在 Pi-puck 本地推理。

本文是现场执行清单。完整参数说明和故障处理见 [端到端部署指南](EPUCK2_PI_PUCK_END_TO_END_DEPLOYMENT.md)。

## 1. 项目边界

当前项目提供：

- `fiducial_detector`：从顶视图像识别二维码/ArUco；
- `marker_state_fuser`：将检测结果转换为世界坐标并估计速度；
- `global_info_bridge`：发布策略使用的 `/global_info`；
- `epuck2_physical.launch.py`：启动单台机器人的驱动、雷达层、策略和保护层；
- Navigation、RoundFlocking、Transport 的 PT/NEAT 策略和配置。

现场另需提供：

- 顶视相机驱动；
- e-puck2 底盘驱动和雷达驱动；
- 相机内参、场地坐标标定和二维码实体配置；
- Pi-puck 的 ROS 2、PyTorch、wiringPi、串口/I2C 权限；
- 中央机与三台 Pi-puck 的同一局域网通信。

中央机和 Pi-puck 可部署同一工作区，但只运行各自需要的节点。

## 2. 运行拓扑

```text
中央机：相机驱动
  /overhead_camera/image_raw       sensor_msgs/msg/Image
  /overhead_camera/camera_info     sensor_msgs/msg/CameraInfo
        |
        v
  fiducial_detector
        -> /marker_detections
  marker_state_fuser
        -> /global_state
  global_info_bridge
        -> /global_info (std_msgs/msg/String, JSON)
                 |
       ---------- ROS 2 DDS ----------
       |              |              |
   robot_0 Pi      robot_1 Pi      robot_2 Pi
   /robot_N/scan   /robot_N/scan   /robot_N/scan
        |              |              |
     本机 PT 策略 -> hardware_protection_layer -> /robot_N/cmd_vel
        |              |              |
     本机 e-puck2 驱动执行电机命令
```

策略需要的接口与 Webots 相同：`/global_info`、`/robot_N/scan` 和 `/robot_N/cmd_vel`。实物 `/global_info` 可以多出 `frame_id` 字段，但 `robots`、`objects`、位置、速度和朝向字段必须保持相同语义。

## 3. 现场配置

### 3.1 坐标

- 场地中心是 `(0,0)`；边界是 `x,y ∈ [-1,1] m`；
- 从上往下看，`+x` 向右、`+y` 向上；
- 逆时针 yaw 为正；机器人朝 `+x` 时 yaw 为 0；
- 顶视相机固定在场地上方，光轴尽量垂直向下，画面覆盖整个场地。

相机平面到场地世界坐标的变换写在 `marker_tracking.yaml`：

```text
world_x = a * camera_x + b * camera_y + tx
world_y = c * camera_x + d * camera_y + ty
```

`camera_to_world` 的六个值必须用现场至少 3 个不共线已知点拟合，不能直接把模板值当作最终值：

```bash
python3 scripts/calibrate_camera_to_world.py calibration_points.csv
```

相机移动、改变分辨率或重新对焦后，重新标定。

### 3.2 二维码

二维码内部是普通文本，二维码本身仍是黑白方块。例如机器人 0 的二维码内容为 `robot_0`。项目不生成或打印二维码。

`marker_tracking.yaml` 中三个数组按相同下标绑定：

```yaml
marker_keys:   ['robot_0', 'transport_box', 'target_zone']
entity_names:  ['robot_0', 'transport_box', 'target_zone']
entity_types:  [0, 1, 2]  # 0=机器人，1=物体，2=目标
```

识别到 `marker_keys[i]` 后，程序使用同一行的名称、类型和 `marker_offsets`，再发布到：

```text
类型 0 -> /global_info["robots"][entity_names[i]]
类型 1/2 -> /global_info["objects"][entity_names[i]]
```

机器人二维码尽量安装在旋转中心正上方，并使二维码方向与机器人前进方向一致；否则测量并填写对应的 `[x,y,yaw]` 偏移。

复制现场配置并修改实际值：

```bash
mkdir -p ~/epuck_deploy/config
cp ~/ROS2swarm_w/src/ros2swarm/config/e_puck/real_robot/marker_tracking.yaml \
  ~/epuck_deploy/config/marker_tracking_2m.yaml
```

至少确认：`image_topic`、`camera_info_topic`、`marker_size`、三个二维码数组、`marker_offsets`、`camera_to_world`。

## 4. 构建和同步

### 4.1 中央机

```bash
cd ~/ROS2swarm_w                         # 进入项目
source /opt/ros/humble/setup.bash        # 加载 ROS 2 Humble
rosdep install --from-paths src --ignore-src -r -y  # 首次部署或依赖变化时执行
colcon build --packages-select communication_interfaces ros2swarm \
  --symlink-install                      # 构建消息、顶视链和策略
source install/setup.bash                 # 加载构建结果
```

依赖已安装且源码未变化时可跳过 `rosdep` 和 `colcon build`；每个新终端仍需执行 `source`。

### 4.2 Pi-puck

将同一版本工作区同步到每台 Pi-puck，然后在每台 Pi-puck 执行：

```bash
cd ~/ROS2swarm_w
source /opt/ros/humble/setup.bash
vcs import --recursive src < dependencies/epuck2_hardware.repos  # 首次获取驱动时执行
rosdep install --from-paths src --ignore-src -r -y                # 首次部署或依赖变化时执行
colcon build --packages-select communication_interfaces ros2swarm \
  epuck_ros2_driver rplidar_ros --symlink-install                # 构建项目和硬件驱动
source install/setup.bash
```

驱动源码、依赖和补丁已经正确安装且未变化时，可以跳过对应获取、安装和构建命令。e-puck2 驱动 watchdog 补丁只应用一次。

确认雷达设备路径：

```bash
ls -l /dev/serial/by-id/  # 复制稳定设备路径到启动命令
```

## 5. 正式启动

四台主机统一设置：

```bash
export ROS_DOMAIN_ID=42      # 四台主机必须一致
export ROS_LOCALHOST_ONLY=0  # 允许局域网 DDS 发现
```

### 5.1 中央机

先按相机型号启动外部相机驱动，然后启动项目顶视链：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2swarm_w/install/setup.bash
ros2 launch ros2swarm epuck2_overhead_tracking.launch.py \
  config:=~/epuck_deploy/config/marker_tracking_2m.yaml
```

该命令启动 `fiducial_detector`、`marker_state_fuser` 和 `global_info_bridge`。

检查：

```bash
ros2 topic hz /overhead_camera/image_raw
ros2 topic hz /global_info
ros2 topic echo --once /global_info
```

### 5.2 每台 Pi-puck

三台分别使用 `robot_id:=0/1/2`，并使用同一个任务策略。以 robot 0 的 Navigation PT 为例：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2swarm_w/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

ros2 launch ros2swarm epuck2_physical.launch.py \
  robot_id:=0 \
  pattern:=navigation_pattern \
  start_driver:=true \
  start_lidar:=true \
  lidar_serial_port:=/dev/serial/by-id/你的雷达设备
```

另外两台只需将 `robot_id` 改为 `1`、`2`。现场已有底盘驱动节点时使用 `start_driver:=false`；已有雷达节点发布 `/robot_N/scan` 时使用 `start_lidar:=false`，避免重复启动和重复发布。

每台 Pi-puck 的项目节点是：

```text
e-puck2 驱动、雷达驱动、lidar_layer、本机 PT 策略、hardware_protection_layer
```

Transport 按当前配置直接发布 `/robot_N/cmd_vel`，不再额外启动第二个策略或安全链。

## 6. START、STOP 和验收

START 前逐台检查：

```bash
ros2 topic hz /robot_0/scan
ros2 topic hz /robot_0/range_data
ros2 topic echo --once /robot_0/odom
ros2 topic echo --once /robot_0/cmd_vel  # START 前应为零
```

先抬起机器人测试方向，再发送 START：

```bash
ros2 topic pub --once /swarm_command \
  communication_interfaces/msg/Int8Message "{data: 1}"
```

停止时发送：

```bash
ros2 topic pub --once /swarm_command \
  communication_interfaces/msg/Int8Message "{data: 0}"
```

真实硬件最终验收必须覆盖：

1. 相机图像、内参和二维码识别稳定；
2. `/global_info` 中 3 台机器人和当前任务实体的坐标、yaw 正确；
3. 三台 Pi-puck 都能收到 `/global_info`，各自雷达发布正确的 `/robot_N/scan`；
4. START、STOP、策略进程中断和驱动 watchdog 都能停车；
5. 单台任务闭环通过后，再运行三台 Navigation、RoundFlocking、Transport。

在没有真实硬件时，项目已完成 Webots 和三机器人 mock 链路验证，但这不等价于真实硬件验收。
