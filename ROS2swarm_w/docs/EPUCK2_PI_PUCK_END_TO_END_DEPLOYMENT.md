# e-puck2 + Pi-puck 实物部署流程

适用环境：2 m x 2 m 场地、3 台 e-puck2 + Pi-puck、每台配备激光雷达、中央顶视相机、ROS 2 Humble、PT 策略在 Pi-puck 本地推理。

### 当前实现需要注意的一点

当前代码虽然调用了 `solvePnP` 获取二维码位置，但 yaw 使用的是图像边 `corner_0 -> corner_1` 的方向，而没有直接使用 `solvePnP` 返回的旋转矩阵。

考虑：先把相机安装成严格顶视，继续使用现有方法完成方向标定；如果实测 yaw 误差随位置或相机倾角明显变化，再升级为带姿态解筛选的 PnP 方法。

## 1. 项目边界

当前项目已经提供：

- 顶视标记检测、世界坐标转换和 `/global_info` 发布；
- 三台机器人各自运行 PT 策略的物理 launch；
- e-puck2、RPLidar 固定版本依赖和驱动补丁；
- Navigation、RoundFlocking、Transport 的 PT 策略、配置和 checkpoint；

部署现场仍需提供：

- 顶视相机及其 ROS 驱动；
- 相机内参和畸变标定；
- 相机到 2 m 场地坐标的现场标定；
- 实际 QR/ArUco 标记及安装偏移；
- 三台 Pi-puck、e-puck2、雷达和串口权限；
- ROS 局域网、DDS 发现和系统时间同步；
- Pi-puck 上的 ROS 2、wiringPi 和 PyTorch。

代码部署位置：


| 设备         | 运行内容                                                                            |
| ---------- | ------------------------------------------------------------------------------- |
| 中央机        | 外部相机驱动、`fiducial_detector`、`marker_state_fuser`、`global_info_bridge`、START/STOP |
| 每台 Pi-puck | e-puck2 驱动、雷达驱动、`lidar_layer`、本机 PT pattern、对应控制层                               |


中央机和三台 Pi-puck 都部署同一版本的完整工作区，但只启动各自负责的节点。

## 2. 数据流和接口

```text
外部顶视相机驱动
  -> /overhead_camera/image_raw
  -> /overhead_camera/camera_info
  -> fiducial_detector
  -> /marker_detections
  -> marker_state_fuser
  -> /global_state
  -> global_info_bridge
  -> /global_info
          |
          +------------ ROS 2 DDS ------------+
          |                 |                  |
       robot_0 Pi        robot_1 Pi         robot_2 Pi
       本机 /scan        本机 /scan         本机 /scan
       本机 PT 推理      本机 PT 推理       本机 PT 推理
       本机 cmd_vel      本机 cmd_vel       本机 cmd_vel
```

主要话题：


| 话题                             | 类型                                                  | 发布者                      |
| ------------------------------ | --------------------------------------------------- | ------------------------ |
| `/overhead_camera/image_raw`   | `sensor_msgs/msg/Image`                             | 外部相机驱动                   |
| `/overhead_camera/camera_info` | `sensor_msgs/msg/CameraInfo`                        | 外部相机驱动                   |
| `/marker_detections`           | `communication_interfaces/msg/MarkerDetectionArray` | `fiducial_detector`      |
| `/global_state`                | `communication_interfaces/msg/TrackedEntityArray`   | `marker_state_fuser`     |
| `/global_info`                 | `std_msgs/msg/String`                               | `global_info_bridge`     |
| `/robot_N/scan`                | `sensor_msgs/msg/LaserScan`                         | 本机雷达驱动                   |
| `/robot_N/range_data`          | `communication_interfaces/msg/RangeData`            | `lidar_layer`            |
| `/robot_N/drive_command`       | `geometry_msgs/msg/Twist`                           | Navigation/RoundFlocking |
| `/robot_N/cmd_vel`             | `geometry_msgs/msg/Twist`                           | 控制层或 Transport           |
| `/swarm_command`               | `communication_interfaces/msg/Int8Message`          | 中央操作端                    |


相机驱动不在当前项目中。它必须发布：

```text
/overhead_camera/image_raw    sensor_msgs/msg/Image
/overhead_camera/camera_info  sensor_msgs/msg/CameraInfo
```

其中：

- 图像必须能由 `cv_bridge` 转换为 `bgr8`；
- `CameraInfo.width/height` 必须与图像一致；
- `CameraInfo.k` 和 `CameraInfo.d` 必须是当前镜头和分辨率的标定结果；
- 图像时间戳必须使用当前 ROS 时间；
- 建议图像频率不低于 10 Hz。

如果相机驱动使用其他话题名，只需修改 `marker_tracking_2m.yaml` 中的：

```yaml
image_topic: '/实际图像话题'
camera_info_topic: '/实际CameraInfo话题'
```



## 3. 场地、相机和配置



### 3.1 现场坐标

先在地面固定坐标定义：

- 场地中心为 `(0,0)`；
- 边界为 `x,y in [-1.0,1.0] m`；
- 在地面标出 `+x`、`+y`；
- `+z` 向上；
- 从上向下看，逆时针 yaw 为正；
- 机器人朝 `+x` 时 yaw 为 0。

俯视示意图：

```text
                  场地世界坐标（俯视）

       (-1,+1)                         (+1,+1)
          +-------------------------------+
          |               +y              |
          |                ^              |
          |                |              |
          |  -x <------ (0,0) ------> +x  |
          |                |              |
          |                v              |
          |               -y              |
          +-------------------------------+
       (-1,-1)                         (+1,-1)

机器人朝 +x：yaw = 0
机器人朝 +y：yaw = +pi/2
从 +x 转向 +y 是逆时针旋转，yaw 增大。
```



### 3.2 相机安装

- 相机安装在场地中心上方；
- 光轴尽量垂直向下；
- 画面覆盖完整场地并保留边缘余量；
- 最远角落的标记仍能稳定识别；
- 固定支架、焦距、对焦和分辨率后再标定；
- 相机移动或改变分辨率后必须重新标定。

当前项目不使用 `world -> camera` TF 计算顶视位置。实际生效的是：

```yaml
camera_to_world: [a, b, tx, c, d, ty]
```

计算关系：

```text
world_x = a * camera_x + b * camera_y + tx
world_y = c * camera_x + d * camera_y + ty
```

也可以写成矩阵：

```text
[world_x]   [a  b] [camera_x]   [tx]
[world_y] = [c  d] [camera_y] + [ty]
```

其中 `camera_x/y` 是检测器通过二维码实际尺寸、相机内参和图像角点计算出的米制相机平面坐标，不是图像像素 `u/v`。


| 参数   | 含义                         |
| ---- | -------------------------- |
| `a`  | `camera_x` 对 `world_x` 的贡献 |
| `b`  | `camera_y` 对 `world_x` 的贡献 |
| `c`  | `camera_x` 对 `world_y` 的贡献 |
| `d`  | `camera_y` 对 `world_y` 的贡献 |
| `tx` | 世界 x 方向平移；相机原点映射后的 world x |
| `ty` | 世界 y 方向平移；相机原点映射后的 world y |


`[a,b;c,d]` 一起描述相机平面到世界平面的旋转、轴方向翻转和比例；`tx/ty` 描述原点平移。例如：

```yaml
camera_to_world: [1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
```

表示：

```text
world_x = camera_x
world_y = -camera_y
```

这只是便于理解的示例。实际六个值不要手算或直接使用模板值，应由第 3.5 节的现场对应点拟合得到。

### 3.3 标记配置

二维码看到的是黑白方块，但方块内部编码的是普通文本。例如，为
`robot_0` 打印的二维码用手机扫描后应显示英文字符串 `robot_0`。
当前项目负责解码和绑定，不负责生成、打印二维码。

检测器解码文本后，用三个等长数组的相同下标完成映射：

```text
marker_keys[i]    检测到的 QR 文本
entity_names[i]   发布到 /global_info 时使用的实体名称
entity_types[i]   实体类型：0=机器人，1=物体，2=目标
marker_offsets    每个实体连续使用 3 个值 [x,y,yaw]
```

当前默认对应关系：


| QR 中写入的文本             | 实体名称      | 类型  | 最终位置                                       |
| --------------------- | --------- | --- | ------------------------------------------ |
| `robot_0`             | `robot_0` | 0   | `/global_info.robots.robot_0`              |
| `robot_1`             | `robot_1` | 0   | `/global_info.robots.robot_1`              |
| `robot_2`             | `robot_2` | 0   | `/global_info.robots.robot_2`              |
| `navigation_target_0` | 同名        | 2   | `/global_info.objects.navigation_target_0` |
| `navigation_target_1` | 同名        | 2   | `/global_info.objects.navigation_target_1` |
| `navigation_target_2` | 同名        | 2   | `/global_info.objects.navigation_target_2` |
| `flocking_target`     | 同名        | 2   | `/global_info.objects.flocking_target`     |
| `transport_box`       | 同名        | 1   | `/global_info.objects.transport_box`       |
| `target_zone`         | 同名        | 2   | `/global_info.objects.target_zone`         |


例如只看三个实体时：

```yaml
marker_keys:  ['robot_0', 'transport_box', 'target_zone']
entity_names: ['robot_0', 'transport_box', 'target_zone']
entity_types: [0, 1, 2]
marker_offsets: [
  0.0, 0.0, 0.0,  # robot_0
  0.0, 0.0, 0.0,  # transport_box
  0.0, 0.0, 0.0   # target_zone
]
```

因此，打印给 `robot_0` 的 QR 码内容必须是文本 `robot_0`。如果二维码内容拼写不同，且没有同步修改 `marker_keys`，检测结果会被状态融合节点忽略。

若使用 ArUco，`marker_keys` 使用字符串形式的数字 ID，例如 `['0','1','2']`，而 `entity_names` 仍可写成 `['robot_0','robot_1','robot_2']`。

所有标记应使用相同的实际黑色外框边长，并将该米制尺寸写入 `marker_size`。

机器人标记尽量安装在旋转中心正上方，标记方向与机器人前进方向一致。无法居中时，在 `marker_offsets` 中填写标记中心到实体中心的 `[x,y,yaw]` 偏移。

### 3.4 创建现场配置

```bash
mkdir -p ~/epuck_deploy/config
cp ~/ROS2swarm_w/src/ros2swarm/config/e_puck/real_robot/marker_tracking.yaml \
  ~/epuck_deploy/config/marker_tracking_2m.yaml
```

现场需要确认和填写：


| 参数                                 | 获取方式                       |
| ---------------------------------- | -------------------------- |
| `image_topic`、`camera_info_topic`  | `ros2 topic list`          |
| `marker_size`                      | 测量打印后的标记外框                 |
| `marker_keys`                      | 核对实际 QR 文本                 |
| `marker_offsets`                   | 测量安装偏移并原地旋转验证              |
| `camera_to_world`                  | 下一节的多点标定                   |
| `robot_id`                         | Pi-puck 标签与 `robot_N` 对应   |
| `lidar_serial_port`                | `ls -l /dev/serial/by-id/` |
| `lidar_baudrate`、`lidar_scan_mode` | 雷达型号和驱动说明                  |
| `lidar_x/y/z/yaw`                  | 测量雷达相对机器人中心的安装位姿           |
| `ROS_DOMAIN_ID`                    | 四台主机统一约定                   |




### 3.5 标定 camera_to_world

1. 固定相机并完成相机内参标定；
2. 启动相机驱动；
3. 单独启动标记检测：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2swarm_w/install/setup.bash

ros2 run ros2swarm fiducial_detector --ros-args \
  --params-file ~/epuck_deploy/config/marker_tracking_2m.yaml
```

1. 在场地四角、四个边中点和中心依次放置标记；
2. 从 `/marker_detections` 记录 `pose.pose.position.x/y`；
3. 保存：

```csv
camera_x,camera_y,world_x,world_y
...
```

1. 计算变换：

```bash
cd ~/ROS2swarm_w
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/calibrate_camera_to_world.py calibration_points.csv
```

1. 把输出写入 `marker_tracking_2m.yaml`；
2. 检查中心、`+x`、`+y`、yaw 和原地旋转时的位置。



## 4. 中央机和 Pi-puck 构建

所有主机统一：

```bash
export ROS_DOMAIN_ID=42       # 四台主机必须使用同一个 ROS 2 域
export ROS_LOCALHOST_ONLY=0   # 允许局域网内的其他主机发现 ROS 节点
```



### 4.1 中央机

```bash
cd ~/ROS2swarm_w                         # 进入项目根目录
source /opt/ros/humble/setup.bash        # 加载 ROS 2 Humble 环境

# 首次部署或依赖变化时执行；依赖已安装且未变化时可跳过
rosdep install --from-paths src --ignore-src -r -y
# 源码变化后执行；已有有效 install 且源码未变化时可跳过
colcon build --packages-select communication_interfaces ros2swarm \
  --symlink-install                      # 构建接口和主项目

source install/setup.bash                 # 加载本次构建结果
```

相机驱动按实际相机型号另外安装。

注意：每个新终端仍需执行两个 `source` 命令；修改源码、接口或依赖后，重新执行相应安装/构建命令。

### 4.2 同步代码到三台 Pi-puck

分别执行：

```bash
rsync -az \
  --exclude build --exclude install --exclude log \
  ~/ROS2swarm_w/ pi@ROBOT_IP:~/ROS2swarm_w/  # ROBOT_IP 替换为目标 Pi-puck 地址
```

该命令在中央机执行；代码未修改且三台 Pi-puck 已是当前版本时可跳过。

### 4.3 每台 Pi-puck

```bash
cd ~/ROS2swarm_w                         # 进入 Pi-puck 上的项目目录
source /opt/ros/humble/setup.bash        # 加载 ROS 2 环境

# 首次获取驱动源码；src/epuck_ros2 和 src/rplidar_ros 已存在且版本正确时可跳过
vcs import --recursive src < dependencies/epuck2_hardware.repos

git -C src/epuck_ros2 apply --check \
  ../../dependencies/patches/epuck_ros2_driver_watchdog.patch    # 只检查补丁能否应用
git -C src/epuck_ros2 apply \
  ../../dependencies/patches/epuck_ros2_driver_watchdog.patch    # 应用驱动看门狗补丁

# 首次部署或依赖变化时执行；依赖已安装且未变化时可跳过
rosdep install --from-paths src --ignore-src -r -y
# 源码变化后执行；已有有效 install 且源码未变化时可跳过
colcon build --packages-select \
  communication_interfaces ros2swarm epuck_ros2_driver rplidar_ros \
  --symlink-install                              # 构建策略、驱动和雷达节点

source install/setup.bash                         # 加载 Pi-puck 端构建结果
```

补丁只应用一次；如果 `--check` 失败，先用 `git -C src/epuck_ros2 status` 确认是否已经应用。

不要重复执行已经成功应用过的 `git apply`；每个新终端仍需执行 `source install/setup.bash`。

Pi-puck 还需确认（首次部署或更换硬件时执行；已确认且硬件未变化时可跳过）：

```bash
ldconfig -p | grep -i wiringPi       # 检查 Pi-puck 电机/底层库是否已安装
ls -l /dev/i2c-12                    # 检查 Pi-puck I2C 设备是否存在
ls -l /dev/serial/by-id/             # 获取雷达稳定串口路径，填入 LIDAR_PORT

python3 - <<'PY'
import torch
print(torch.__version__)              # 检查 PyTorch 是否可导入及版本
print(torch.rand(1))                   # 检查张量计算是否正常
PY
```



## 5. 正式启动流程



### 5.1 中央机启动相机驱动

按实际相机型号启动外部驱动，然后确认：

```bash
# 检查图像是否持续发布；命令持续运行，用 Ctrl-C 结束
ros2 topic hz /overhead_camera/image_raw
# 检查相机内参 K/D 和 frame_id 是否存在
ros2 topic echo --once /overhead_camera/camera_info
```

相机驱动通常需要每次开机启动；只有配置为系统服务或已经在运行时才可跳过启动命令，
但仍建议执行上面的两个检查命令。

### 5.2 中央机启动顶视状态链

```bash
# 加载 ROS 2 和项目环境
source /opt/ros/humble/setup.bash
source ~/ROS2swarm_w/install/setup.bash
# 设置局域网 ROS 2 通信参数
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 启动二维码检测、状态融合和 /global_info 桥接
ros2 launch ros2swarm epuck2_overhead_tracking.launch.py \
  config:=${HOME}/epuck_deploy/config/marker_tracking_2m.yaml
```

该命令启动：

```text
fiducial_detector
marker_state_fuser
global_info_bridge
```

确认：

```bash
# 检查二维码检测频率；持续运行，用 Ctrl-C 结束
ros2 topic hz /marker_detections
# 检查融合后的 typed 状态频率
ros2 topic hz /global_state
# 检查策略实际接收的 JSON 频率
ros2 topic hz /global_info
# 查看一帧完整全局状态，确认机器人和目标名称/坐标
ros2 topic echo --once /global_info
```



### 5.3 三台 Pi-puck 分别启动本机链

在三台 Pi-puck 上分别设置 `ROBOT_ID=0/1/2`，三台使用同一个 `PATTERN`：

```bash
# 加载 ROS 2 和项目环境
source /opt/ros/humble/setup.bash
source ~/ROS2swarm_w/install/setup.bash
# 设置与中央机相同的 ROS 2 域并允许局域网发现
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 本机编号必须与二维码 robot_N 和 namespace 对应
ROBOT_ID=0
# 三台机器人使用相同任务策略
PATTERN=navigation_pattern
# 使用 /dev/serial/by-id 下查询到的本机雷达路径
LIDAR_PORT=/dev/serial/by-id/本机雷达

# 启动本机驱动、雷达、RangeData 层和 PT 策略
# 若驱动由其他服务启动，将 start_driver 或 start_lidar 改为 false
# lidar_x/y/z/yaw 是雷达相对机器人 base_link 的安装位姿，按实测修改
ros2 launch ros2swarm epuck2_physical.launch.py \
  robot_id:=${ROBOT_ID} \
  pattern:=${PATTERN} \
  start_driver:=true \
  start_lidar:=true \
  lidar_serial_port:=${LIDAR_PORT} \
  lidar_baudrate:=115200 \
  lidar_x:=0.0 \
  lidar_y:=0.0 \
  lidar_z:=0.08 \
  lidar_yaw:=0.0
```

如果底盘驱动已经由其他 launch 或系统服务启动，将 `start_driver:=false`；如果雷达已经由外部
launch 发布 `/robot_N/scan`，将 `start_lidar:=false`，否则会重复发布。这里的“已设置”只表示驱动已安装，
不等于节点已经运行。

每台 Pi-puck 启动：

```text
epuck_ros2_driver
rplidar_ros
lidar_layer
本机 PT pattern
hardware_protection_layer（Navigation/RoundFlocking）
```

Transport 按当前 launch 直接由 `transport_pattern` 发布 `/robot_N/cmd_vel`。

可选 pattern：


| 任务            | `PATTERN`               | 必需实体                                       |
| ------------- | ----------------------- | ------------------------------------------ |
| Navigation    | `navigation_pattern`    | `robot_N`、`navigation_target_N`            |
| RoundFlocking | `roundflocking_pattern` | 三台 `robot_N`、`flocking_target`             |
| Transport     | `transport_pattern`     | 三台 `robot_N`、`transport_box`、`target_zone` |


如果雷达已经由外部 launch 发布 `/robot_N/scan`，设置 `start_lidar:=false`，避免重复发布。

### 5.4 START 前检查

中央机：

```bash
# 确认中央机已收到全部实体及其世界坐标
ros2 topic echo --once /global_info
```

每台 Pi-puck：

```bash
# 检查雷达 LaserScan 频率；将 robot_0 改为本机编号
ros2 topic hz /robot_0/scan
# 检查项目内部 RangeData 频率
ros2 topic hz /robot_0/range_data
# 检查底盘里程计和坐标方向
ros2 topic echo --once /robot_0/odom
# 检查最终速度命令；START 前应为零
ros2 topic echo --once /robot_0/cmd_vel
```

把 `robot_0` 替换为本机编号。未发送 START 时，`cmd_vel` 应为零。

### 5.5 START 和 STOP

START：

```bash
# 向所有策略广播 START（data=1）
ros2 topic pub --once /swarm_command \
  communication_interfaces/msg/Int8Message '{data: 1}'
```

STOP：

```bash
# 向所有策略广播 STOP（data=0）
ros2 topic pub --once /swarm_command \
  communication_interfaces/msg/Int8Message '{data: 0}'
```

首次启动先抬起车轮，确认前进方向、旋转方向和 STOP，再落地运行。

## 6. 推荐联调顺序

按以下顺序逐段调通，不要第一次同时启动全部设备：

1. 相机驱动：确认 `Image` 和 `CameraInfo`；
2. 标记检测：确认 `/marker_detections`；
3. 世界坐标：完成 `camera_to_world` 和 `marker_offsets`；
4. 全局状态：确认 `/global_info` 中实体、坐标和方向；
5. DDS 网络：三台 Pi-puck 都能看到 `/global_info`；
6. 单台硬件：逐台确认 e-puck2、雷达、odom 和 cmd_vel；
7. 抬轮控制：逐台验证 START、STOP 和驱动超时停车；
8. 三台本地推理：确认每台只发布本机 namespace；
9. 落地闭环：单台任务通过后再运行三台。

部署完成的最低判定：

- 相机两个外部话题稳定有效；
- `/global_info` 中包含当前任务全部实体；
- x/y/yaw 与现场坐标一致；
- 三台 Pi-puck 都能接收 `/global_info`；
- 三个 `/robot_N/scan` 和底盘驱动正常；
- `robot_id`、QR 文本和 namespace 一一对应；
- 三台使用相同代码、任务配置和 checkpoint；
- START、STOP 和驱动超时停车正常；
- 已完成一次单台和三台现场闭环；

