# MASAR 统一仿真验证与可视化平台

本项目把 `ROS2swarm_w` 与 `masar_ws` 的群行为验证、探索组搜救和完整搜救入口统一到
一个 Web 工作台中。平台通过 Docker Compose 管理 ROS1、ROS2、Web 数据面、仿真显示、
API 和统一网关，操作者可以从浏览器选择场景、启动或停止一次完整运行，并同步查看仿真、
RViz、任务分配、地图、传感器、机器人状态和日志。

当前不接入 `multi-agent-neat`；ROS2swarm 的算法训练仍保持独立。

## 项目布局

三个项目需要位于同一个 workspace 下，但 workspace 可以位于宿主机任意路径：

```text
<workspace>/
├── masar-simulation-platform/
├── masar_ws/
└── ROS2swarm_w/
```

启动脚本会从当前 checkout 自动推导 `<workspace>`。只有目录布局不同于上述结构时，
才需要设置 `MASAR_WORKSPACE_ROOT`。

## 快速开始

目标设备需要 `linux/amd64`、Docker Engine、Docker Compose v2、NVIDIA 驱动、
NVIDIA Container Toolkit、X11、`/dev/dri` 和可读的 `.Xauthority`。

```bash
cd <workspace>/masar-simulation-platform
make docker-start
```

首次启动会复用或拉取公开的 ROS1/ROS2 运行环境镜像，并在缺少构建结果时自动执行
ROS1 `catkin_make` 和 ROS2 `colcon build`。完成后访问：

```text
http://127.0.0.1:18090
```

常用命令：

```bash
make docker-doctor   # 只读环境检查
make docker-start    # 初始化并启动完整平台
make docker-status   # 查看 8 个服务和当前 Run
make docker-logs     # 查看聚合日志
make docker-stop     # 先停止当前 Run，再关闭完整平台
```

## 远程服务器访问

平台运行在远程服务器时，在本地 SSH 配置中增加：

```sshconfig
LocalForward 18090 127.0.0.1:18090
```

连接服务器后，本地浏览器仍访问 `http://127.0.0.1:18090`。不需要单独转发 Agent、
rosbridge、noVNC、Webots 或 Docker 端口。

## 平台模块

- 群行为与控制验证：ROS2swarm Webots，以及 ROS1 DEFORM/控制器演示。
- 探索组独立运行：Gazebo 或 Webots Tepper 搜救探索。
- 完整搜救系统：Gazebo/Tepper 探索组与响应组协同运行。

## 文档

- [完整使用与运维手册](docs/OPERATIONS.md)
- [需求与接口规范](docs/SPEC.md)
- [平台架构设计](docs/PLATFORM_DESIGN.md)
- [实施计划与阶段状态](docs/IMPLEMENTATION_PLAN.md)
- [自动化和实测验收记录](docs/ACCEPTANCE.md)
- [开发过程设计与计划归档](docs/superpowers/)
