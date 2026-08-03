# Compose 管理业务运行时设计

日期：2026-08-03

状态：已获用户确认，等待实施计划

## 1. 背景

当前平台的 API、ROS Web sidecar、显示服务、Webots 端口保护和 Caddy 由
Docker Compose 管理，但 `masar_ros1_srv-g` 与 `masar_ros2_bridge-w` 是预先
存在的外部容器。`deploy/start-platform.sh` 通过 `docker exec -d` 向这两个容器
注入 Runtime Agent。

这一部署方式有两个确定缺陷：

1. 业务容器重启会终止附加的 Runtime Agent，平台随后无法启动 Run，必须人工
   再次执行 Agent bootstrap。
2. 新设备即使取得 workspace 源码，也没有两个固定名称的本地业务容器，无法通过
   平台启动脚本完成部署。

业务运行环境现已发布为公开 Docker Hub 镜像。平台应直接使用这些镜像创建业务
容器，并继续把 workspace 源码挂载到 `/root/workspace`。本次不引入两套部署模式，
不把业务源码烘焙进运行环境镜像。

## 2. 目标

1. 使用一个 Compose 项目管理平台全部容器，包括 ROS1 和 ROS2 业务运行时。
2. 删除对预先存在的本地业务容器和 `docker exec -d` Agent 注入的依赖。
3. 业务容器重启后由容器入口自动恢复 Runtime Agent。
4. 新设备取得完整 workspace 后，可通过一个命令完成镜像准备、缺失 ROS overlay
   编译、平台镜像构建和服务启动。
5. 保留当前源码挂载开发方式和业务脚本的 `/root/workspace` 路径约定。
6. 普通本地设备启动后可直接通过浏览器访问统一入口；SSH 端口转发仅作为远程
   服务器访问说明。
7. 每个实施阶段先完成自动化与真实运行验收，再进入下一阶段。
8. 实施期间同步更新现有设计、计划、运维和验收文档，最终不存在两套启动方式。

## 3. 非目标

1. 不为源码挂载和源码内置分别维护两套 Compose。
2. 不在本阶段重新制作 ROS1/ROS2 基础运行环境 Dockerfile。
3. 不把 ROS Web sidecar 合并进业务运行时容器。
4. 不改变六个场景、Run 状态机、进程所有权、停止顺序或 ROS2 Domain `42`。
5. 不接入 `multi-agent-neat`。
6. 不向 API 或前端挂载 Docker socket。
7. 不承诺 ARM64、无 NVIDIA GPU 或非 Linux 设备能够运行 Webots/Gazebo 图形仿真。

## 4. 固定镜像

Compose 使用公开、仅支持 `linux/amd64` 的镜像，并同时固定标签和不可变 digest：

| 服务 | 镜像 |
|---|---|
| ROS1 runtime | `talent18/masar_ros1_srv-g:v2@sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59` |
| ROS2 runtime | `talent18/masar_ros2_bridge-w:v2@sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c` |

当前设备已经拥有 digest 匹配的本地镜像，初始化必须跳过重复拉取。新设备或本地
digest 不匹配时才从 Docker Hub 拉取。两个镜像压缩体积约为 7.53 GB 和 9.13 GB，
初始化日志需要明确提示新设备首次下载耗时，但不得在每次启动时强制访问 registry。

## 5. 容器拓扑

单一 `deploy/compose.yaml` 管理以下八个服务：

| Compose 服务 | 容器职责 |
|---|---|
| `ros1-runtime` | Noetic、Gazebo、`masar_ws` 业务进程、ROS1 Runtime Agent |
| `ros2-runtime` | Humble、Webots、`ROS2swarm_w` 业务进程、ROS2 Runtime Agent |
| `ros1-web` | ROS1 rosbridge、TF republisher、视频服务 |
| `ros2-web` | ROS2 rosbridge、TF republisher、视频服务 |
| `api` | 场景目录、Run 编排、状态和归档 API |
| `display` | Gazebo、Webots、RViz 的 Xvfb、x11vnc 和 noVNC |
| `webots-guard` | Webots WebSocket 流端口限制 |
| `caddy` | 前端静态文件和统一同源网关 |

`api` 不再使用 `dev-api` profile。它是平台必需服务，始终随 Compose 启动。

为兼容场景定义、诊断工具和原有脚本，两个业务容器继续使用
`masar_ros1_srv-g` 与 `masar_ros2_bridge-w` 作为明确容器名，但其所有权改为
Compose。平台不支持同时运行第二套同名业务容器。

## 6. 业务容器配置

两个 runtime 服务把启动业务容器时已经验证的参数转换为 Compose 配置：

- `network_mode: host`
- `ipc: host`
- `gpus: all`
- `NVIDIA_DRIVER_CAPABILITIES=all`
- `/dev/dri:/dev/dri`
- `group_add: [video]`
- `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- 宿主机 Xauthority 到 `/tmp/.docker.xauth:ro`
- workspace 根目录到 `/root/workspace:rw`
- `init: true`
- `restart: unless-stopped`

部署脚本根据自身位置计算 workspace 根目录并导出绝对路径，Compose 文件不包含
`/home/zjc/workspace`。默认仓库布局保持为：

```text
<workspace>/masar-simulation-platform
<workspace>/masar_ws
<workspace>/ROS2swarm_w
```

ROS2 runtime、ROS2 Web sidecar 和所有 ROS2 场景继续统一使用
`ROS_DOMAIN_ID=42`。所有平台服务继续设置 loopback 的 `NO_PROXY/no_proxy`。

## 7. Runtime Agent 生命周期

业务容器入口直接 `exec` 对应的 `deploy/agent-bootstrap.sh`，Runtime Agent 成为
容器常驻主进程，而不是通过 `docker exec -d` 附加。Compose 健康检查访问：

- ROS1 Agent：`http://127.0.0.1:18101/healthz`
- ROS2 Agent：`http://127.0.0.1:18102/healthz`

ROS2 健康检查还必须验证 `webots_preflight` capability。API 和对应 Web sidecar
只有在 runtime 健康后才进入可用状态。

容器重启会终止该容器中的所有 Run 子进程并重新执行 Agent 入口。Agent 自动恢复
为空闲健康状态，不要求 `--agents-only` 修复。平台不会把中断的 Run 伪装为继续
运行；API 应按现有断连和失败契约记录终态。

## 8. 首次 overlay 编译

Git 不跟踪以下生成目录：

- `masar_ws/build`、`masar_ws/devel`
- `ROS2swarm_w/build`、`ROS2swarm_w/install`、`ROS2swarm_w/log`

`docker-start` 在启动长期服务前检查：

1. `/root/workspace/masar_ws/devel/setup.bash`
2. `/root/workspace/ROS2swarm_w/install/setup.bash`

ROS1 overlay 缺失时，使用一次性 `ros1-runtime` 容器在挂载的 `masar_ws` 中执行
项目既有的 `catkin_make` 构建流程。ROS2 overlay 缺失时，使用一次性
`ros2-runtime` 容器执行项目既有的 `colcon build` 流程。ROS1 完成后才开始 ROS2，
任一失败都阻止长期服务启动。

控制台只显示阶段、开始、完成、耗时和失败摘要。完整构建输出分别保存为：

```text
masar-simulation-platform/.runtime/logs/bootstrap-ros1.log
masar-simulation-platform/.runtime/logs/bootstrap-ros2.log
```

构建完成后，只对生成的 build/devel/install/log 目录恢复宿主机 UID/GID，不能递归
修改 workspace 源码所有权。overlay 已存在时直接跳过构建并输出一行说明。

本阶段只保证缺失 overlay 的首次构建。源码变更后的增量重编译继续使用项目原有
构建命令，不在启动脚本中引入源文件哈希、自动清理或并发构建系统。

## 9. 统一命令

仓库新增顶层 `Makefile`，只作为 `deploy/platform.sh` 的薄封装。部署逻辑只存在于
一个脚本中，避免 Makefile 与 shell 各维护一套行为。

```bash
make docker-init
make docker-start
make docker-stop
make docker-status
make docker-logs
make docker-doctor
```

### 9.1 `make docker-init`

1. 检查 Docker daemon、Docker Compose、Linux AMD64、NVIDIA runtime、GPU、
   `/dev/dri`、X11/Xauthority、workspace 布局和固定端口。
2. 创建或校验 32 字节、权限为 `0600` 的 Agent secret。
3. 检查本地两个业务镜像的 RepoDigest。
4. digest 匹配时跳过拉取；缺失或不匹配时拉取固定镜像。
5. 不启动长期服务，不编译 ROS workspace。

### 9.2 `make docker-start`

1. 执行与 `docker-init` 相同的幂等前置保证，因此用户可以直接运行这一条命令。
2. 处理一次性的旧容器迁移。
3. 按第 8 节准备缺失 overlay。
4. 使用 Docker 构建 API、ROS Web、display、guard 和包含前端静态产物的 Caddy
   镜像，不要求宿主机安装 Node.js/npm。
5. `docker compose up -d --build --remove-orphans` 启动全部服务。
6. 等待 runtime、sidecar、API、display、guard 和 Caddy 健康。
7. 输出 `http://127.0.0.1:18090` 和状态、日志、停止命令。

重复执行 `docker-start` 必须幂等。平台已健康时不能重启正在运行的 Run 或重复编译
已有 overlay。

### 9.3 停止与诊断

`docker-stop` 先查询当前 Run。存在活动 Run 时，通过 API 请求停止并等待其完成归档，
随后执行 Compose down。API 不可用时给出警告，再停止 Compose 管理的容器；容器边界
保证其中残留业务进程同时结束，不向宿主机其他进程发送信号。

`docker-status` 显示 Compose 服务、健康状态、当前 Run 和统一入口状态。
`docker-logs` 使用 Compose 聚合日志并允许透传服务名。
`docker-doctor` 只执行检查，不拉镜像、不编译、不启动或停止服务。

## 10. 旧部署迁移与删除

新 Compose 第一次启动前，部署脚本检查两个固定容器名是否已被非 Compose 容器占用。
如存在活动 Run，迁移失败并要求先停止 Run。不存在活动 Run 时，只停止并删除精确匹配
的旧容器，保留镜像、workspace、运行归档和其他宿主机容器。

doctor/init 的端口检查必须识别当前 Compose 项目和两个固定旧容器。由它们占用的端口
标记为“可迁移”，`docker-init` 只提示而不失败，`docker-start` 在第 9.2 节的迁移步骤
中处理。无法证明属于当前平台的端口占用仍然阻止启动，脚本不得结束对应进程。

实现中删除：

- `deploy/start-platform.sh` 及 `--agents-only` 行为；
- 启动脚本中的外部容器存在性检查；
- `docker exec -d` Agent 注入和精确 pkill 修复逻辑；
- 只验证旧注入方式的单元测试；
- 运维文档中的手工启动、修复和结束 Agent 命令；
- 当前架构文档中“业务容器不由 Compose 管理”的有效描述。

历史验收记录不伪造或重写。相关段落增加“已由 Compose runtime 部署取代”的说明。

## 11. 浏览器访问

统一入口继续只绑定 loopback。普通本地 Linux 设备启动完成后直接访问：

```text
http://127.0.0.1:18090
```

浏览器不需要直接访问 Agent、rosbridge、VNC 或 Webots 内部端口。

远程服务器是附加场景。运维文档在本地访问说明之后提供 SSH 示例：

```sshconfig
LocalForward 18090 127.0.0.1:18090
```

本地端口冲突时允许把本地端改为 18089，但服务器目标仍为 18090。SSH 转发不属于
平台启动前提，也不应出现在默认本地快速开始步骤中。

## 12. 错误处理

| 失败 | 行为 |
|---|---|
| Docker/Compose 不可用 | doctor/start 立即失败并给出安装前提 |
| 非 AMD64 | 在下载大镜像前失败并说明镜像架构限制 |
| GPU、NVIDIA runtime、X11 或 `/dev/dri` 缺失 | 在创建业务容器前失败 |
| Docker Hub 暂时不可用但本地 digest 匹配 | 继续使用本地镜像，不访问 registry |
| 镜像缺失且拉取失败 | 停止，不创建部分平台服务 |
| ROS overlay 编译失败 | 保存完整日志，停止，不执行 Compose up |
| 固定端口被无关进程占用 | 失败并列出端口和进程，不清理无关进程 |
| 旧容器中仍有活动 Run | 拒绝迁移，不结束 Run |
| runtime 健康检查失败 | 依赖服务不进入 ready，启动命令返回失败 |
| Compose 管理的 runtime 重启 | Agent 自动恢复，旧 Run 按失败契约收口 |

## 13. 安全与所有权

1. API 和前端不接触 Docker socket。
2. Agent secret 只读挂载到 API 与两个 runtime，宿主机权限保持 `0600`。
3. 业务进程仍由 Run ID、PID/PGID 和 `/proc` start ticks 证明所有权。
4. 单个 Run 的停止不能停止 runtime 容器；整个平台 `docker-stop` 才停止 Compose
   管理的 runtime。
5. 旧容器迁移只匹配两个固定容器名，不使用模糊名称、glob 或全局 Docker 清理。
6. 构建脚本只修复生成目录的所有权，不修改源码和宿主机其他目录。

## 14. 分阶段实施与验收

### 阶段 1：Compose 与命令闭环

实现 Makefile、部署脚本、两个 runtime 服务、前端容器内构建和旧逻辑删除。验收包括：

- `docker compose config`；
- 镜像 digest 匹配和缺失拉取的隔离测试；
- doctor/init/start/stop/status/logs 命令测试；
- 缺失 overlay、编译失败和已存在 overlay 分支测试；
- 确认运行时代码不再包含 `docker exec -d` 或 `--agents-only`；
- 全量 Python、前端和部署契约回归。

阶段 1 全部通过后才进入阶段 2。

### 阶段 2：真实容器生命周期

在确认无活动 Run 后迁移当前两个旧容器，启动完整 Compose。验收包括：

- 八个服务全部健康；
- 当前本地镜像 digest 匹配时没有重复拉取；
- Agent capability 与场景目录可用；
- 分别重启 ROS1 和 ROS2 runtime 后 Agent 自动恢复；
- 重启不需要运行任何修复命令；
- `docker-stop` 不影响宿主机无关容器或进程。

阶段 2 全部通过后才进入阶段 3。

### 阶段 3：三个模块真实回归

按顺序验收：

1. 群行为 Webots Run；
2. Webots 探索 Run，确认 ROS2 脚本先于 ROS1 脚本；
3. Gazebo 完整搜救 Run。

每个 Run 必须达到 `RUNNING`，页面获得真实数据，停止为受控终态，归档 checksum
通过，无 Run 子进程和 Run-only 端口残留。任一失败阻止下一项。

### 阶段 4：文档和新设备流程收口

同步更新 `SPEC.md`、`PLATFORM_DESIGN.md`、`IMPLEMENTATION_PLAN.md`、
`OPERATIONS.md` 和 `ACCEPTANCE.md`。使用无 overlay 的隔离 workspace 验证首次自动
编译流程，最后执行一次从 Compose down 到统一入口健康的干净启动验收。

## 15. 完成标准

1. 两个业务容器和六个平台容器全部由同一 Compose 项目管理。
2. 当前设备不重复拉取 digest 已匹配的本地业务镜像。
3. 新设备仅需完整 workspace、Docker Compose、兼容 NVIDIA 环境和 X11，即可运行
   `make docker-start` 完成缺失镜像拉取、首次 overlay 编译和平台启动。
4. 业务容器重启后 Runtime Agent 自动恢复，平台不需要人工修复命令。
5. `make docker-stop` 关闭活动 Run 及全部 Compose 服务，不影响宿主机其他进程。
6. 本地浏览器可直接访问 `http://127.0.0.1:18090`；远程服务器可选用 SSH 转发。
7. 三个模块的真实 Run 回归全部通过。
8. 有效代码和使用文档中只保留新的 Make/Compose 启动方式。
