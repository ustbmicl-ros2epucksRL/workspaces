# MASAR 仿真平台使用与运维手册

本文是当前平台唯一的启动和使用入口。设计契约见 `SPEC.md` 与
`PLATFORM_DESIGN.md`，阶段验收证据见 `ACCEPTANCE.md`。

## 1. 最短启动流程

目标设备满足第 2 节前提后，只需要：

```bash
git clone https://github.com/ustbmicl-ros2epucksRL/workspaces.git
cd workspaces/masar-simulation-platform
make docker-start
```

启动完成后，本机浏览器直接访问：

```text
http://127.0.0.1:18090
```

`make docker-start` 是幂等命令。服务已经健康时不会重建或重启 8 个常驻服务；
前端源码发生变化时，它只使用固定 Node 容器更新 `frontend/dist`，Caddy 会直接
读取新的挂载文件。

## 2. 环境前提

平台当前支持：

- `linux/amd64`。
- NVIDIA GPU、可用的宿主机 NVIDIA 驱动和 NVIDIA Container Toolkit。
- Docker Engine 与 Docker Compose v2。
- `/dev/dri`、X11 socket 和可读的 `.Xauthority`。
- 同一 workspace 下的 `masar-simulation-platform`、`masar_ws` 和 `ROS2swarm_w`。

宿主机不要求安装 ROS、Python 依赖、Node.js 或 npm。默认目录结构是：

```text
workspaces/
├── masar-simulation-platform/
├── masar_ws/
└── ROS2swarm_w/
```

路径或显示环境不同可覆盖：

```bash
export MASAR_WORKSPACE_ROOT=/absolute/path/to/workspaces
export MASAR_HOST_DISPLAY=:0
export MASAR_XAUTHORITY=/home/your-user/.Xauthority
```

先做只读检查：

```bash
make docker-doctor
```

检查失败时脚本只报告问题，不会结束宿主机进程，也不会抢占无关端口。

## 3. Compose 服务

单一 `deploy/compose.yaml` 管理 8 个常驻服务：

| 服务 | 职责 |
|---|---|
| `ros1-runtime` | ROS1 Noetic 业务环境与 ROS1 Runtime Agent |
| `ros2-runtime` | ROS2 Humble/Webots 业务环境与 ROS2 Runtime Agent |
| `ros1-web` | ROS1 rosbridge、TF 和视频数据面 |
| `ros2-web` | ROS2 rosbridge、TF 和视频数据面 |
| `api` | 场景目录、Run 编排、状态、日志和归档 API |
| `display` | Gazebo、RViz、Webots 的 Xvfb/noVNC 显示管理 |
| `webots-guard` | Webots 兼容端口的 loopback 访问保护 |
| `caddy` | 前端与全部浏览器请求的同源入口 |

两个公开业务镜像固定为：

```text
talent18/masar_ros1_srv-g:v2@sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59
talent18/masar_ros2_bridge-w:v2@sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c
```

本机存在完整 digest 时不会访问 registry；缺少哪个镜像才拉取哪个。两个镜像均
挂载整个 workspace 到 `/root/workspace`，镜像只提供运行环境和系统依赖。

## 4. 初始化与首次构建

可显式初始化：

```bash
make docker-init
```

也可以直接执行 `make docker-start`，它会包含初始化。首次启动依次完成：

1. 检查架构、Docker、GPU、X11、目录和固定端口。
2. 复用或拉取两个固定 digest 的业务镜像。
3. 创建 32 字节、`0600` 的共享 Agent secret。
4. 在固定 Node 容器中构建前端静态资源。
5. 缺少 `masar_ws/devel/setup.bash` 时执行 `catkin_make`。
6. 缺少 `ROS2swarm_w/install/setup.bash` 时执行 `colcon build`。
7. 只在缺少平台依赖镜像时构建 API、sidecar、display 和 guard 镜像。
8. 启动 8 个服务并等待全部健康。

首次 ROS 编译可能较慢，完整日志保存在：

```text
masar-simulation-platform/.runtime/logs/bootstrap-ros1.log
masar-simulation-platform/.runtime/logs/bootstrap-ros2.log
masar-simulation-platform/.runtime/logs/frontend.log
```

后续检测到 overlay 已存在时直接跳过。脚本只修复 `build/devel/install/log` 等生成
目录的 UID/GID，不改源码目录所有权。

若设备上存在同名的旧外部业务容器，启动脚本只迁移以下两个精确名称：

```text
masar_ros1_srv-g
masar_ros2_bridge-w
```

迁移前必须没有活动 Run。脚本不会删除业务镜像、workspace、归档或其他容器。

## 5. 开发阶段目录挂载

当前不区分生产和开发 Compose。镜像保存依赖，可变内容从仓库只读挂载：

- `backend/`、`scenarios/` 挂载到 `api`。
- ROS Web 启动脚本和 supervisor 配置挂载到两个 sidecar。
- display manager、xstartup 和 supervisor 配置挂载到 `display`。
- Webots guard 脚本和 Caddyfile 分别挂载到对应服务。
- `frontend/dist` 挂载给 Caddy。
- `masar_ws`、`ROS2swarm_w` 和 Runtime Agent 代码通过 workspace 挂载进入业务容器。

API 的源码与场景仍是只读挂载。由于不同设备上的 checkout 可能包含 `0600` 源码，
Compose 明确让 API 容器以 root 用户读取这些挂载文件；API 不挂载 Docker socket，
也不获得宿主机特权或写入源码目录的能力。

常见修改后的生效方式：

```bash
# 前端源码：容器内自动 npm ci + Vite build，不重建 Caddy
make docker-start

# API 或场景 YAML：只重启 API，不重建镜像
docker compose -f deploy/compose.yaml restart api

# ROS1/ROS2 Web 脚本
docker compose -f deploy/compose.yaml restart ros1-web ros2-web

# 显示或 Webots guard 脚本
docker compose -f deploy/compose.yaml restart display webots-guard
```

只有 Dockerfile、系统包或前端 `package-lock.json` 的依赖集合发生变化时才需要重建
相应依赖镜像。

## 6. 页面与模块

平台统一三个模块：

| 模块 | 已提供场景 | 主要面板 |
|---|---|---|
| 群行为与控制验证 | ROS2swarm Webots；DEFORM/控制器 Gazebo | 仿真、世界坐标轨迹、按机器人传感器、机器人/话题健康、日志 |
| 探索组独立运行 | Gazebo 或 Webots Tepper | 仿真、RViz、任务分配与建图、传感器、探索态势、日志 |
| 完整搜救 | Gazebo 探索组 + 响应组 | 探索全部面板、应急对象、响应组核心状态、日志 |

群行为 Webots 世界固定为 `command_world.wbt`，机器人类型可选 `e_puck` 或
`burger`。群行为不启动 RViz。探索 Webots 固定先启动 ROS2
`start_robosar_webots.sh`，就绪后再启动 ROS1 `launch_robosar_w.sh`。

传感器选择器按机器人同步切换相机、LaserScan、红外、IMU 和 Odometry，只显示
实际发现且类型匹配的数据。总体轨迹使用 Webots `/global_info` 或 Gazebo
`/gazebo/model_states` 的统一世界坐标，不拼接各机器人的局部 odom。

## 7. Run 与平台停止

一次“启动环境”创建一个 Run。该模块的仿真、RViz、录包和业务进程都属于同一
Run。页面“停止运行”或 Run stop API 会逆序停止该 Run 的全部进程，但 8 个常驻
Compose 服务保持健康，便于开始下一次实验。

整个平台关闭使用：

```bash
make docker-stop
```

该命令会先请求活动 Run 停止并等待 `current=null`，随后关闭全部 8 个服务，包括
两个业务 runtime。API 不可用时会给出警告后执行整个平台关闭。它不会向宿主机
无关进程发送信号。

不要用关闭浏览器代替“停止运行”。

## 8. 状态与日志

```bash
make docker-status
make docker-logs
make docker-logs SERVICE=api
make docker-logs SERVICE=ros1-runtime
```

查看当前 Run 与能力目录：

```bash
curl --noproxy '*' -fsS http://127.0.0.1:18090/api/v1/runs/current
curl --noproxy '*' -fsS http://127.0.0.1:18090/api/v1/capabilities
```

业务 runtime 是 Compose 服务，执行以下命令后 Agent 会随容器主进程自动恢复，
不需要注入或修复命令：

```bash
docker restart masar_ros1_srv-g
docker restart masar_ros2_bridge-w
```

## 9. 远程服务器访问

直接在本机部署时不需要 SSH 转发。平台位于远程服务器时，只转发统一入口：

```sshconfig
Host masar-platform
  HostName <服务器地址>
  User <用户名>
  LocalForward 18090 127.0.0.1:18090
  ServerAliveInterval 30
  ServerAliveCountMax 3
```

转发建立后，本地浏览器仍访问 `http://127.0.0.1:18090`。本机部署和远程服务器部署
使用同一个浏览器地址，区别仅在于远程服务器需要上述 SSH 配置。不需要转发 Agent、
rosbridge、noVNC、Webots 或 Docker 端口。若本地 `18090` 已被其他程序占用，应先
确认并处理本地端口归属，不维护第二套平台 URL。

## 10. 常见故障

### doctor 报端口冲突

脚本允许当前 Compose 服务和两个精确旧业务容器占用登记端口。其他监听者会被
拒绝并显示端口信息，不会被自动终止。确认归属后由操作者单独处理。

### 首次启动停在 ROS 编译

查看 `.runtime/logs/bootstrap-ros1.log` 或 `bootstrap-ros2.log`。修复源码或依赖后
再次执行 `make docker-start`；已成功生成的 overlay 会被复用。

### 修改场景后页面仍是旧目录

场景目录已挂载，但 API 在启动时加载 catalog。执行：

```bash
docker compose -f deploy/compose.yaml restart api
```

### Webots 或 Gazebo 卡顿

先确认 `MASAR_HOST_DISPLAY` 指向 NVIDIA X server，`make docker-doctor` 通过，且
Webots Run 使用 VirtualGL/noVNC。不要切回 W3D 主显示路径，也不要对宿主机执行
全局 `pkill`。

### 已知上游限制

- Burger 的 `aggregation_pattern` 异常可在 ROS2swarm 原生运行中复现，暂不作为
  平台闭环缺陷处理。
- ROS2swarm 手工选择 `sensor_type=ir` 仍可能触发现有 `ir_layer` 话题格式问题；
  平台默认验收使用 `lidar`，同时展示 e-puck 的物理红外数据。
- Gazebo/Tepper 长时间运行曾出现原生 `free(): invalid pointer`；平台会归档并清理
  Run，但不会通过重启业务容器掩盖上游崩溃。
