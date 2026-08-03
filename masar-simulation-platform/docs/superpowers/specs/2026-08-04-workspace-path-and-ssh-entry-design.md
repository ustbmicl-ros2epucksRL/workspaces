# Workspace 路径与 SSH 统一入口设计

状态：已获用户方案确认，待书面复核  
日期：2026-08-04

## 1. 目标

本次变更只收口部署路径和访问说明，不改变场景、ROS 图、容器内部工程路径、Run
生命周期或前端功能：

1. 宿主机 workspace 可以位于任意绝对路径，不依赖 `/home/zjc/workspace`。
2. 三个项目继续采用同级目录布局，避免为分散仓库引入额外变量和挂载分支。
3. 本机和 SSH 转发后的浏览器入口统一为 `http://127.0.0.1:18090`。
4. 删除过期、重复或可能被误认为当前操作方式的文档说明。

## 2. 路径契约

支持的默认布局为：

```text
<workspace>/
├── masar-simulation-platform/
├── masar_ws/
└── ROS2swarm_w/
```

`<workspace>` 可以是任意宿主机绝对路径。`deploy/platform.sh` 按以下优先级解析：

1. 设置了 `MASAR_WORKSPACE_ROOT` 时，将其解析为真实绝对目录。
2. 未设置时，使用 `masar-simulation-platform` 当前 checkout 的父目录。
3. 路径不存在、不是目录，或者缺少三个同级项目时立即失败并列出缺失项。

解析结果传给 Compose，并整体挂载到容器内 `/root/workspace`。容器内部继续使用：

```text
/root/workspace/masar-simulation-platform
/root/workspace/masar_ws
/root/workspace/ROS2swarm_w
```

这些是稳定的容器接口，不是宿主机路径。现有 ROS 配置、地图文件、构建脚本和 Agent
无需改成宿主机路径。

本阶段不支持三个项目分散在不同父目录，也不增加三个独立的源码路径变量。

## 3. Compose 与启动脚本

`make docker-init`、`make docker-start`、`make docker-stop`、`make docker-status`、
`make docker-logs` 和 `make docker-doctor` 继续是唯一受支持的生命周期入口。

启动脚本负责规范化并导出 `MASAR_WORKSPACE_ROOT`，Compose 只消费解析后的值。保留
Compose 文件相对路径默认值仅用于同级标准布局和配置检查，不在任何文档中要求用户
进入固定宿主机目录。

启动成功和状态输出统一显示：

```text
http://127.0.0.1:18090
```

并提供远程服务器所需的单条 SSH 映射提示，不新增第二个平台端口。

## 4. SSH 访问契约

平台继续只在服务器 loopback 上暴露统一入口 `127.0.0.1:18090`。服务器部署时，
客户端 SSH 配置固定使用一对一映射：

```sshconfig
LocalForward 18090 127.0.0.1:18090
```

本机部署和 SSH 转发后的浏览器都访问：

```text
http://127.0.0.1:18090
```

当前文档不再提供 `18089` 备用入口，也不使用 `7897`。本地 `18090` 已被其他程序
占用时，操作者应先确认并处理本地端口归属，而不是让平台文档长期维护第二套 URL。

只转发 Caddy 统一入口。Agent、rosbridge、noVNC、Webots 和 Docker 内部端口不单独
转发。

## 5. 文档职责与清理

保留以下根目录文档的单一职责：

- `OPERATIONS.md`：唯一启动、访问、停止和排障手册。
- `SPEC.md`：需求与可验收契约。
- `PLATFORM_DESIGN.md`：当前架构与产品决策。
- `IMPLEMENTATION_PLAN.md`：阶段实施状态和剩余工作。
- `ACCEPTANCE.md`：真实自动化与运行验收证据。

清理规则：

1. 删除活跃实现和操作说明中的固定本机 checkout 路径、备用访问端口和无关代理端口；
   设计或验收文档只可在说明已删除约定时提及这些历史值。
2. 删除已由根目录文档完整覆盖的 2026-08-03 Compose 专项设计与实施计划，避免两套
   部署说明并存。
3. 删除已失效的旧外部容器操作步骤；设计、计划或验收中的必要历史事实必须明确
   标记为历史或已取代，不能再表现为可执行操作。
4. 不重写真实 Run ID、测试结果或历史故障证据。
5. 文档中的宿主机路径使用 `<workspace>`；容器路径明确标注为容器内部稳定接口。

## 6. 错误处理

- `MASAR_WORKSPACE_ROOT` 指向不存在目录时，启动脚本在运行 Docker 前失败。
- 缺少任一同级项目时，`doctor` 和 `start` 返回具体缺失路径。
- 本地 `18090` 转发失败属于 SSH 客户端或本地端口占用，不触发服务器容器重启。
- 服务器 `18090` 被非平台进程占用时，现有只读端口检查继续拒绝启动，不自动杀进程。
- 文档清理不删除验收记录、运行归档或用户源码。

## 7. 分阶段实施与验收

### 阶段 1：路径契约

1. 先增加路径解析和 Compose 挂载契约测试。
2. 规范化显式 `MASAR_WORKSPACE_ROOT`，保留 checkout 父目录自动推导。
3. 验证默认路径和一个不同绝对路径下的标准三项目布局。
4. 运行平台脚本单测、Compose config、Bash 语法和 `docker-doctor`。

阶段 1 全部通过后才进入阶段 2。

### 阶段 2：SSH 与文档收口

1. 将访问说明统一为 `18090 -> 18090`。
2. 更新启动日志、环境示例和五份根目录文档。
3. 删除重复的 2026-08-03 Compose 专项文档。
4. 扫描活跃实现与使用说明，确认不存在备用访问端口、无关代理端口或写死的宿主机
   workspace 路径；仅允许日期化设计/验收文档中的删除说明。

阶段 2 全部通过后才进入阶段 3。

### 阶段 3：完整回归

1. 运行 Python 全套测试、前端单测、TypeScript/Vite、Ruff 和 compileall。
2. 验证 Compose 配置、镜像 digest、八服务健康状态和当前 Run 状态。
3. 验证服务器 `127.0.0.1:18090` 入口，并记录 SSH 一对一转发的人工配置步骤。
4. 将最终结果追加到 `ACCEPTANCE.md`，确保 `OPERATIONS.md` 与实现一致。

## 8. 非目标

- 不改变平台服务端口或 Caddy 路由。
- 不支持分散目录的三个工程。
- 不调整 ROS1/ROS2 镜像、Domain 42、仿真器或场景启动顺序。
- 不处理 Cursor 自身的代理端口或其他宿主机 SSH 会话。
- 不删除真实历史验收证据。
