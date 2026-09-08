# MASAR 统一仿真平台可执行 Spec

状态：已批准，Compose 托管 E1-E4 全部验收通过
文档/部署契约版本：0.5-compose-managed（Python 包版本见 `pyproject.toml`，当前为 0.1.0）
最后更新：2026-08-04
对应设计：[PLATFORM_DESIGN.md](./PLATFORM_DESIGN.md)

## 1. Context

当前 `ROS2swarm_w` 与 `masar_ws` 有多个相对独立的启动入口，分别运行在 ROS2 Humble/Webots 与 ROS1 Noetic/Gazebo 容器中。操作者需要进入不同容器、按顺序执行脚本、切换 VNC 窗口并分别检查 ROS 话题，难以在一次实验中同步观察仿真、地图、RViz、传感器和任务进度。

目标用户是当前服务器上的单一实验操作者。平台完成后，用户从一个 Web 工作台选择合法场景、完成预检、启动/停止实验、发送已有任务命令、同步观察全部相关面板，并下载本次运行的日志和 rosbag。

## 2. Verified Current State

原始业务环境验证始于 2026-07-31；当前部署、显示与传感器契约更新于 2026-08-04。

1. 单一 Compose 项目管理 `ros1-runtime`、`ros2-runtime`、`ros1-web`、`ros2-web`、`api`、`display`、`webots-guard`、`caddy` 和 `public-tunnel` 9 个常驻服务。
2. ROS1 runtime 使用公开固定 digest `talent18/masar_ros1_srv-g:v2@sha256:3639f637...`；ROS2 runtime 使用 `talent18/masar_ros2_bridge-w:v2@sha256:5550e15e...`。二者均使用 host network、host IPC、GPU/X11 和 workspace bind mount。
3. ROS1/ROS2 `dynamic_bridge`、六个场景、真实浏览器数据面和三模块 Run 生命周期均已验收；当前证据统一记录在 `ACCEPTANCE.md`。
4. ROS Web Tools 网关由独立 ROS1/ROS2 sidecar 提供，不安装进业务 runtime。
5. `start_webots.launch.py` 已把默认关闭的 `webots_stream` 与 `webots_port` 传给 `WebotsLauncher`；平台生产显示使用 VirtualGL + noVNC，W3D stream 只保留兼容能力。
6. `start_simulation.sh` 的配置区列出 Webots world、pattern、机器人数量、机器人类型、传感器类型和 logging 等参数；平台通过受限命令模板传入一次性参数，不修改该脚本的全局变量。
7. `masar_ws/README.md` 记录 Gazebo/Webots 探索、DEFORM、控制器和完整 Tepper 入口；Webots 探索由平台强制先启动 ROS2，再启动 ROS1。
8. 原 Qt GUI 继续发布 `/system_mission_command` 与 `/response_group/return_home` 并订阅业务状态；平台模式只禁用该 Run 内的 Qt 实例，由 Web 工作台接管控制。
9. `EmergencyTask.msg` 与 `ResponseGroupStatus.msg` 提供完整搜救的任务和响应组业务状态字段。

## 3. Goals

### G-1 统一入口

用户通过 `http://127.0.0.1:18090` 完成场景选择、运行控制和观测，不需要手工进入两个容器执行命令。

### G-2 保持业务行为

平台运行与现有原命令运行使用同一 launch、参数默认值和算法节点。平台接入不得改变算法、任务状态机、导航、安全或控制输出。

### G-3 同屏同步观测

场景要求的仿真、RViz、任务分配与建图、传感器、状态和日志默认同时显示。选择只用于单个面板内部的数据源，不用于隐藏其他核心面板。

### G-4 可诊断和可归档

平台显示节点/话题健康度，并保存 resolved config、事件、日志、ROS 图快照、摘要和 bag。

## 4. Non-Goals

- 本期不接入 `multi-agent-neat` 训练流程。
- 不迁移、复用或修复 `masar-unified-platform`。
- 不使用端口 `18088`。
- 不替换或删除原 Qt GUI。
- 不把任意 shell、任意 ROS topic publish 或 Docker socket 暴露给浏览器。
- MVP 不实现历史时间轴回放、多次实验图表对比和统计显著性分析。
- 不为聚合、分散等行为设计新的算法评价指标。
- 不增加现有入口未支持的仿真器/业务笛卡尔积，例如 MVP 不提供“完整搜救 + Webots”。

## 5. Scenario Catalog

平台只允许下表组合。`scenario_id` 是 API 与归档中的稳定标识。

| scenario_id | 一级模块 | 仿真器 | 执行入口 | RViz | 业务控制 |
|---|---|---|---|---|---|
| `swarm.webots.ros2swarm` | 群行为与控制验证 | Webots | 平台 wrapper 调用现有 `create_enviroment.launch.py` | 否 | 运行/停止 |
| `control.gazebo.response-deform` | 群行为与控制验证 | Gazebo | `run_response_deform_demo.sh` | 否 | START/STOP/响应组返航 |
| `control.gazebo.response-controller` | 群行为与控制验证 | Gazebo | `run_response_controller_demo.sh <mode>` | 否 | 运行/停止 |
| `exploration.gazebo.tepper` | 探索组独立运行 | Gazebo | `launch_robosar_g.sh` | 是 | START/STOP/HOME |
| `exploration.webots.robosar` | 探索组独立运行 | Webots | ROS2 `start_robosar_webots.sh` 后 ROS1 `launch_robosar_w.sh` | 是 | START/STOP/HOME |
| `rescue.gazebo.tepper-response` | 完整搜救系统 | Gazebo | `run_tepper_response_demo.sh` | 是 | START/STOP/HOME/响应组返航 |

### 5.1 ROS2swarm 参数允许列表

`swarm.webots.ros2swarm` 至少支持现有代码已列出的：

- `pattern`: `aggregation_pattern`、`attraction_pattern`、`dispersion_pattern`、`navigation_pattern`、`roundflocking_pattern`、`transport_pattern`、`drive_pattern` 以及仓库中实际存在的 `*_neat` launch。
- `robot`: `e_puck`、`burger`。
- `sensor_type`: `lidar`、`ir`，且必须通过所选机器人/行为兼容性校验。
- `number_robots`: 正整数，MVP UI 限制为 1 到 20；更高数量只能修改场景清单后启用。
- `webots_world`: 群行为验证固定为 `command_world.wbt`，由场景清单内部提供；前端不显示世界选择控件，API 也拒绝其他世界值。
- 出生区域、最小距离、障碍物数量、Webots mode、logging 和 transport object shape 使用现有 launch 约束。

平台 wrapper 必须构造与 `start_simulation.sh` 中 `create_enviroment.launch.py` 调用等价的受限命令，不编辑 `start_simulation.sh` 文件。

### 5.2 响应控制器参数允许列表

DEFORM demo 的 `response_goal_sequence_node.py` 接收 START/STOP，并明确忽略 exploration mission HOME；响应组返航使用 `/response_group/return_home`。平台必须按这两个不同语义提供按钮。

`control.gazebo.response-controller` 的 `mode` 只允许：

- `roundflocking-checkpoint`
- `roundflocking-heuristic`
- `transport`（前端显示为 `transport-synchronized_slots`）
- `transport-checkpoint`
- `transport-heuristic`

这些值来自现有脚本 usage，不允许前端自由输入。

## 6. Scenario Manifest Contract

场景使用版本化 YAML。实现必须通过 JSON Schema 等价校验，拒绝未知字段。

```yaml
schema_version: 1
scenario_id: exploration.webots.robosar
module: exploration
simulator: webots
runtime:
  ros1_master_uri: http://127.0.0.1:11311
  ros2_domain_id: 42
display_profile: exploration
parameters: {}
steps:
  - id: ros2_webots_bridge
    agent: ros2
    command_ref: ros2.exploration_webots
    depends_on: []
    readiness:
      all:
        - type: ros2_topic
          name: /clock
          message_type: rosgraph_msgs/msg/Clock
        - type: ros2_topic
          name: /robot_0/odom
          message_type: nav_msgs/msg/Odometry
          qos_profile: sensor_data
    start_timeout_seconds: 180
    grace_seconds: {sigint: 10, sigterm: 5, sigkill: 5}
  - id: ros1_robosar
    agent: ros1
    command_ref: ros1.exploration_webots
    depends_on: [ros2_webots_bridge]
    readiness:
      all:
        - type: ros1_topic
          name: /sim_map
          message_type: nav_msgs/OccupancyGrid
        - type: ros1_topic
          name: /frontier_filter/filtered_frontiers
          message_type: robosar_messages/PointArray
    start_timeout_seconds: 180
    grace_seconds: {sigint: 10, sigterm: 5, sigkill: 5}
stop_order: [ros1_robosar, ros2_webots_bridge]
data_profile: exploration_webots
artifact_profile: exploration_default
minimum_free_bytes: 2147483648
```

`command_ref` 映射到平台源码中的静态命令模板。YAML 不包含原始 shell。

## 7. Runtime State Machine

```text
IDLE
  -> PREFLIGHT
  -> STARTING
  -> RUNNING
  -> STOPPING
  -> COMPLETED

PREFLIGHT | STARTING | RUNNING | STOPPING -> FAILED
```

### 7.1 状态规则

1. `IDLE` 时可创建运行。
2. `PREFLIGHT` 校验容器、磁盘、端口、ROS master、显示/GPU、场景参数和 Agent。
3. `STARTING` 按依赖图启动步骤，并等待每步 readiness。
4. 所有 required readiness 通过后才进入 `RUNNING`。
5. 用户停止、超时或正常退出进入 `STOPPING`，按 `stop_order` 终止本 run 的进程组。
6. 正常停止并完成归档为 `COMPLETED`；启动失败、必要进程异常或停止超时为 `FAILED`。
7. `PREFLIGHT`、`STARTING`、`RUNNING`、`STOPPING` 均视为活动运行。此时创建第二个运行返回 HTTP 409。
8. API 重启后根据持久化 run lease 和 Agent 进程所有权恢复状态；不能确认所有权时公开状态为 `FAILED`、`reason_code=NEEDS_INSPECTION`，不得猜测并清理进程。`FAILED_NEEDS_INSPECTION` 只允许作为内部 reason code 映射，不是额外的公开状态。

### 7.2 停止语义

- 第一次 stop 是幂等请求，返回当前 stop operation。
- 重复 stop 不创建新信号风暴。
- 先发送 SIGINT，等待场景清单中的 grace period；再对同一进程组发送 SIGTERM；最后才对该进程组 SIGKILL。
- 任何信号目标必须由 Agent 的 `run_id -> pgid` 记录解析。
- 未登记 PID/PGID、旧项目 Agent 或其他容器进程不在清理范围。

## 8. API Contract

前缀：`/api/v1`。响应为 JSON，事件使用 WebSocket `/api/v1/events`。

### 8.1 场景与能力

```text
GET /capabilities
GET /scenarios
GET /scenarios/{scenario_id}
POST /scenarios/{scenario_id}/validate
```

`validate` 只做静态参数校验，不启动进程。

### 8.2 运行

```text
POST /runs
GET  /runs/current
GET  /runs/{run_id}
POST /runs/{run_id}/stop
GET  /runs/{run_id}/events?after=<cursor>
GET  /runs/{run_id}/logs?process=<id>&after=<cursor>
GET  /runs/{run_id}/artifacts
GET  /runs/{run_id}/artifacts/{artifact_id}/download
```

创建请求：

```json
{
  "scenario_id": "swarm.webots.ros2swarm",
  "parameters": {
    "pattern": "aggregation_pattern",
    "robot": "e_puck",
    "sensor_type": "lidar",
    "number_robots": 5,
    "webots_world": "command_world.wbt"
  },
  "recording": {
    "enabled": true,
    "profile": "default"
  }
}
```

成功返回 HTTP 202 和稳定 `run_id`。参数错误返回 422；存在活动运行返回 409；Agent 不可用返回 503。`lidar` 是平台验收默认值；e-puck 的物理 `ps0..ps7` 红外话题仍可独立展示。手工选择 `sensor_type=ir` 会进入 ROS2swarm 当前已知的异常路径，不作为默认闭环。

### 8.3 任务命令

```text
POST /runs/{run_id}/mission-commands/start
POST /runs/{run_id}/mission-commands/stop
POST /runs/{run_id}/mission-commands/home
POST /runs/{run_id}/response-commands/return-home
```

每个端点只在场景清单允许、运行处于 `RUNNING`、消息类型已验证时可用。服务端负责发布现有消息。请求体不能指定 topic、type 或任意数值。

任务命令与“停止实验进程”是两种不同操作，UI 文案和确认行为必须区分。

### 8.4 数据源

```text
GET /runs/{run_id}/data-sources
GET /runs/{run_id}/robots
```

`/data-sources` 返回场景 profile 允许的数据源模式，不伪装成当前 ROS 图快照。例如：

```json
{
  "graph": "ros2",
  "topic_pattern": "/<robot>/scan",
  "message_types": ["sensor_msgs/msg/LaserScan"],
  "capability": "laser",
  "required": true,
  "availability": "UNKNOWN"
}
```

浏览器 `RosDataRuntime` 使用 rosapi 周期发现真实话题，将允许模式扩展为精确 topic，校验消息类型并负责订阅、首帧、`observed_hz`、`last_message_age_ms` 和陈旧状态。上述观测值是数据链路健康信息，不写入算法评价结果。

`/robots` 根据该 Run 已解析的参数返回期望机器人清单，包括 `robot_id`、显示名称、模型、topic prefix 和 `expected=true`。该清单用于在 ROS 发现尚未完成时保持完整选择器与状态行，不代表机器人当前在线；实时状态仍由浏览器对真实 ROS 消息的观测决定。

### 8.5 事件格式

```json
{
  "schema_version": 1,
  "event_id": "01J...",
  "run_id": "01J...",
  "sequence": 184,
  "wall_time": "2026-07-31T11:42:18.321Z",
  "sim_time_ns": 950000000,
  "source": "ros1-agent",
  "kind": "TOPIC_STALE",
  "severity": "warning",
  "payload": {
    "topic": "/robot_5/odom",
    "age_ms": 800
  }
}
```

同一 `run_id` 内 `sequence` 严格递增。没有仿真时钟时 `sim_time_ns` 为 null。

## 9. UI Requirements

### UI-001 全局框架

顶栏必须显示当前一级模块、场景、运行状态、已运行时间和全局控制。左侧是场景配置与预检，中央是同步可视化，右侧是运行状态，下方是事件和日志。

### UI-002 同时显示

探索组和完整搜救场景进入 `RUNNING` 后，首屏必须同时包含仿真、RViz、任务分配与建图、传感器四个可见面板；不得要求用户切换互斥标签才能知道另一个面板是否正常。`/sim_map` 的原生地图观测由 RViz 承担；任务分配与建图面板不重复实现 OccupancyGrid 渲染器。

### UI-003 群行为布局

群行为与控制场景不显示 RViz。桌面端采用上下布局：仿真面板在上方横跨中央工作区，位置/轨迹与传感器面板在下方等宽并排；移动端按仿真、位置/轨迹、传感器顺序纵向排列。多机器人轨迹必须位于一个统一世界坐标系：Webots 使用 `/global_info`，Gazebo 使用 `/gazebo/model_states`；禁止直接把各机器人的局部 `odom` 叠加为总体轨迹。

### UI-004 面板操作

仿真、RViz、任务分配与建图、传感器和日志面板均支持调整尺寸、最大化、恢复和弹出。动态标题或状态变化不得改变固定工具栏高度。

桌面右侧状态栏不得用固定小高度裁剪内容。视口有剩余高度时，各状态面板按权重增长并填满右栏；视口高度不足时，面板保持内容所需的最小高度，由右栏整体纵向滚动。搜救对象列表在任务态势面板内独立滚动，滚动到底后最后一项仍必须位于面板边界内。

### UI-005 数据源选择

传感器面板使用同一个机器人选择器联动相机、激光、IMU、里程计和控制速度等实际可用数据源。改变选择不卸载其他核心面板；当前任务分配与建图面板没有地图话题选择器。

### UI-006 缺失数据

不存在话题时显示“未提供”；允许但尚未发布显示“等待数据”；超时显示 `STALE` 和最后消息时间；类型不匹配显示错误。以上状态不得显示伪造的实时数值。事件/快照数据在收到首条消息前显示 `—`，只有真实空快照才可显示计数 `0`。未创建 Run 以及 `COMPLETED`/`FAILED` 终态不得把场景默认机器人数量显示成实时机器人、轨迹图例或传感器选项。

### UI-007 群行为指标边界

`swarm.webots.ros2swarm`、`control.gazebo.response-deform` 和 `control.gazebo.response-controller` 不显示聚合度、平均间距或速度一致性等未定义指标。

### UI-008 现有业务字段

- `/percent_completed` 只出现在探索组和完整搜救的“任务分配与建图”面板；其 ROS 值是 `0..1` 比例，UI 显示时乘以 100。
- `/task_allocation` 表示最近一次新分配批次，显示为“本轮分配”，不得解释为全部当前任务数。
- 默认 HIGH 分配器不发布 `/tasks_completed` 消息，因此 exploration、exploration_webots 和 rescue profile 不公开“已完成任务”字段。
- 任务队列和 `EmergencyTask.status` 只出现在完整搜救。
- `ResponseGroupStatus` 中的规划状态、slot error、clearance 等只在响应组实际发布该消息的场景显示。

### UI-009 图形交互

Webots、Gazebo 和 RViz 面板支持其原生鼠标拖动、旋转或缩放。最大化/恢复不得重启实验。

### UI-010 断线恢复

浏览器刷新或短暂断开后，使用 `run_id` 和事件 cursor 恢复当前运行，不重新提交启动请求。

### UI-011 非占位数据面板

每个可见面板必须至少消费一种 profile 允许的真实 ROS 消息、Run 事件或 Agent 进程日志。面板不得只显示固定“等待数据”或说明文字。话题可在浏览器连接后创建，因此运行期间必须周期性重新发现，并保留既有订阅。任务命令只在 Run 为 `RUNNING` 时可用。

### UI-012 模块化数据内容

- 群行为：由仿真器世界真值驱动的动态机器人二维轨迹/位姿、按 `e_puck` 或 `burger` 可用能力展示的传感器、机器人在线状态、真实话题健康、进程日志；`discussed_dispersion_pattern` 额外展示真实讨论事件/状态。型号表只提供未发现话题时的默认说明；一旦运行图发现该机器人的真实传感器话题，实际话题优先。Odometry 仍可作为所选机器人的传感器读数，但不决定总体轨迹。
- 探索：“任务分配与建图”显示 Qt 任务分配图、地图覆盖率、候选前沿和本轮分配；“探索任务态势”只显示在线探索机器人以及火源/受害者/待推送障碍的真实快照计数、位置和探测机器人。传感器按机器人选择相机/Scan/IMU/Odometry，另有真实进程日志；删除静态任务地图和跨面板重复字段。
- 完整搜救：包含探索全部内容。传感器选择器必须将 `agent_N` 显示为“探索机器人
  N”，将 `tb_N` 显示为“响应机器人 N”；探索 Khepera 使用相机、LaserScan、
  Odometry 和 cmd_vel，响应 Burger 另有 IMU，两组均不显示不存在的红外字段。
  响应组默认摘要只保留六项真实数据：合并后的响应/规划状态、运动使能、当前任务、
  任务队列、编队 RMS/最大误差和最小净空；不单列语义相近或低频使用的行为模式、
  编队尺度、响应机器人数量和最近结果。

## 10. ROS Web Gateway Requirements

### RW-001 连接

前端使用单例连接管理器分别维护 `ros1` 和 `ros2` rosbridge 连接。组件通过 adapter 获取数据，不直接创建 WebSocket。

### RW-002 允许列表

数据订阅仅允许场景 data profile 中声明的话题模式和类型。控制发布不通过浏览器 rosbridge 完成。

### RW-003 TF

ROS1/ROS2 sidecar 保留 `tf2_web_republisher` 进程和显式帧 `TfClient` 适配器，但当前生产 React 面板不请求 TF。`/sim_map` 与 TF 的交互观测由 RViz noVNC 完成；群行为总体轨迹直接消费仿真器世界位姿，不为局部 Odometry 猜测或合成跨机器人 TF，也不传输完整 TF 树。

### RW-004 视频

普通机器人相机通过 `web_video_server` 的 MJPEG stream 提供。单个浏览器最多保持当前所选机器人的一个主视频流，切换后释放旧流；HTTP 已连接但尚未提交首帧时使用新的 retry URL 每 1.5 秒有界重连，直到 `naturalWidth > 0`，不得要求操作者先切换机器人。Gazebo 响应组四台 Burger 均保留 `/tb_i/camera/color/image_raw`，但浏览器不并行预热或缓存四路图像。D435 深度与双红外渲染默认停用且不进入平台 data profile，避免无消费者传感器持续占用 Gazebo 渲染预算。低频、latched 的 `/task_allocation_image` 单独使用 snapshot：首帧前每 1.5 秒重试，成功后每 3 秒刷新，并通过变化的查询参数避免浏览器缓存旧响应。刷新必须先在候选图像中完成解码，成功后才替换可见帧；STOP/HOME 使 allocator publisher 退出或刷新失败时保留最后成功帧。

### RW-005 地图

`MapPanel` 对上层提供稳定接口。MVP 使用现有 web-video-server 的任务分配图快照和浏览器原生 SVG 绘制世界坐标轨迹，不引入 CDN 或未维护的三维渲染依赖。`ros3djs` 1.1.0 及旧 ROS2D/EaselJS 链均不进入当前依赖。

### RW-006 背压

LaserScan/Odometry UI 更新上限 10 Hz，队列长度 1；日志虚拟化。PointCloud2 当前不在任何 data profile 中，前端不订阅或渲染。

## 11. Process and Compatibility Requirements

### PC-001 原命令回归

以下命令在未设置任何平台参数时，其 dry-run 输出的业务命令和启动顺序必须与修改前基线一致：

```bash
./launch_robosar_g.sh --dry-run --headless --no-vglrun
./launch_robosar_w.sh --dry-run --no-vglrun
./run_response_deform_demo.sh --dry-run --headless --no-vglrun
./run_response_controller_demo.sh --dry-run transport
./run_tepper_response_demo.sh --dry-run --headless --no-vglrun
./start_robosar_webots.sh --dry-run --no-vglrun
./start_simulation.sh --dry-run --no-vglrun
```

### PC-002 默认关闭

任何新增 launch argument 或环境变量均须默认关闭平台行为。未设置时不得改变节点、topic、参数值、仿真速度、控制输出、GUI 和清理语义。

### PC-003 进程所有权

Agent 只管理当前 `run_id` 创建的进程组。禁止按名称清理所有 ROS/Gazebo/Webots 进程。

### PC-004 Qt 保留

直接运行现有搜救入口时 Qt GUI 保持原行为。只有平台场景显式设置默认关闭的 skip 参数时，才由 Web 工作台成为当前运行的控制面。

## 12. Preflight Requirements

每次运行至少检查：

1. 两个所需容器状态与共享工作区可见性。
2. Agent 版本与 scenario schema 兼容。
3. 场景所需端口没有被未知进程占用。
4. ROS master/domain 配置匹配场景。
5. 所需构建产物存在；缺失时只执行清单允许的构建步骤。
6. VNC DISPLAY、xauth、VirtualGL/GPU 或 Webots stream 能力满足场景。
7. 归档目录可写且可用空间不少于场景 `minimum_free_bytes`。
8. 不存在平台登记的其他活动运行。
9. Webots 双容器场景中 ROS2 步骤先启动，ROS1 `/sim_map` 仍为权威来源。

预检失败不启动任何业务步骤，并返回逐项结果与修复建议。

## 13. Artifact Contract

### 13.1 必需文件

每次进入 `STARTING` 的运行都创建 `runs/<run_id>`。即使启动失败，也至少保存：

- 原始请求与 resolved scenario。
- 平台/Agent/场景 schema 版本。
- wall clock、仿真时钟信息与步骤时间。
- events.jsonl。
- 每个步骤的 stdout/stderr。
- ROS 节点和话题快照。
- 终止原因与最终状态。
- 文件 checksum。

### 13.2 rosbag

- 录制关闭时 manifest 明确记录 `recording.enabled=false`。
- 录制开启时 ROS1/ROS2 选择来自 artifact profile，topic 列表来自场景 data profile；网页不能传 topic、`-a` 或通配全部话题。
- ROS1 与 ROS2 bag 分目录保存。
- 下载 API 必须支持大文件流式传输和 Range 请求。

### 13.3 摘要边界

MVP summary 可以包含运行时长、最终状态、数据缺失、任务数量/状态、探索完成度最终值和已有响应组字段。不得自动计算未定义的群行为评分。

## 14. Acceptance Criteria

### 核心编排

1. `GET /scenarios` 精确返回第 5 节的 6 个 MVP 场景，且不返回完整搜救 Webots 组合。
2. 任意场景处于活动状态时，第二次 `POST /runs` 返回 409，且没有在任一容器创建新业务进程。
3. Webots 探索场景只有在 ROS2 `/clock` 和 `/robot_0/odom` readiness 通过后才启动 ROS1 步骤。
4. 任一 required readiness 超时后运行进入 `FAILED`，已启动步骤按逆序停止并完成失败归档。
5. 对同一 run 连续发送两次 stop，不会终止该 run 之外的进程，也不会创建第二个停止流程。
6. 控制 API 拒绝未在场景中声明的 command、topic、消息类型和参数字段。

### 界面与数据真实性

7. 探索 Gazebo、探索 Webots 和完整搜救进入运行页时，仿真、RViz、任务分配与建图、传感器四个面板在 1440x900 viewport 同时可见。
8. 群行为场景不渲染 RViz 面板，也不出现“聚合度”“平均间距”“速度一致性”字段；`command_world.wbt` 为固定世界且不显示世界选择控件，机器人类型以下拉框提供 `e_puck`、`burger`；1440x900 viewport 中仿真位于轨迹和传感器上方并横跨两者，390x844 viewport 中三个面板纵向排列且无横向溢出。
9. 当 `/robot_2/camera` 不存在时，相机区域显示“未提供”，页面 DOM 和 WebSocket 消息中均不存在该相机的伪造数值；但真实发现的相机话题必须覆盖型号默认能力说明。Gazebo Burger 的四路 D435 彩色话题 `/tb_i/camera/color/image_raw` 均可选择，浏览器只连接当前选择的一路并在切换时释放旧流，不得在后台并行预热四路；停用的深度与双红外话题不进入 data profile；Webots Burger 未发现相机话题时仍显示未配备。完整搜救必须分别列出 5 台探索机器人和 4 台响应机器人，切换后相机、`/feedback/scan` 或 `/tb_i/scan`、`/tb_i/imu` 与 Odometry 必须属于同一选择；两组都不得显示红外字段。
10. 当已连接 topic 超过 profile 阈值未更新时，UI 在 1 个检测周期内从 ONLINE 变为 STALE，并显示最后消息 age。
11. `/percent_completed` 只在 exploration/rescue 的任务分配与建图面板中按 `ratio * 100` 显示；`/task_allocation` 标为“本轮分配”；默认探索 profile 不公开从未发出消息的 `/tasks_completed`。静默分配/对象源显示 `—`，真实空对象快照才显示 `0`。任务图取得首帧后，后续 snapshot 失败不得清空可见图像；探索 STOP/HOME 后至少保留最后成功的任务分配与建图帧。
12. 完整搜救响应摘要只显示第 UI-012 条规定的六项核心数据；`ResponseGroupStatus.rms_slot_error` 只在消息存在且类型匹配时显示，缺失时显示状态而不是 0。响应面板不得与后续健康面板重叠。高视口中右栏面板必须共同填满可用高度；低视口中右栏必须可滚动，任务态势列表不得被父面板裁剪。
13. 调整尺寸、最大化、恢复和弹出任一核心面板不会触发新的 `POST /runs`，也不会重启 ROS 进程。
14. 浏览器刷新后恢复同一 `run_id`，从最后事件 cursor 继续显示，启动步骤计数不增加。

### 可视化与网关

15. Webots R2025a GUI 通过 `:103` VirtualGL + noVNC 在 SSH 转发后的 Chromium 中显示非空画面并可拖动/缩放；renderer 必须为 NVIDIA，连续两轮启动/停止均成功且无 `18141`/Webots/ROS 子进程残留。
16. Gazebo noVNC 与 RViz noVNC 在独立面板中同时显示，鼠标输入只作用于获得焦点的面板；Gazebo renderer 必须为 NVIDIA，D435 相机视锥不在 Gazebo GUI 内重复可视化，响应组基准场景的 GUI FPS 不低于 30。
17. RViz 能显示 `/sim_map` 及其现有机器人/TF 图层；任务分配与建图面板能显示 `/task_allocation_image`、地图覆盖率、候选前沿和本轮分配，不维护第二套原生 OccupancyGrid 渲染器。
18. 普通相机和 D435 选择新源后旧视频请求在 5 秒内释放，同一浏览器不保留两个 MJPEG 主视频流。默认选中机器人的静默首连必须自动重试并显示首帧，不得依赖一次人工切换；D435 不得创建后台 snapshot 预热循环。
19. LaserScan 与 Odometry 前端渲染频率不超过 10 Hz；消息突发时保留最后值而非无限排队。

### 兼容与安全

20. 第 11.1 节的 7 个原命令回归全部通过，平台参数关闭时无业务命令差异。
21. 直接运行原搜救入口仍启动原 Qt GUI，START/STOP/HOME 行为与基线一致。
22. 全部平台服务默认只监听 `127.0.0.1`；`ss -ltnp` 不出现平台端口绑定 `0.0.0.0` 或 `::`。
23. 浏览器请求任意 shell 字符串、任意 ROS topic publish 或路径穿越 artifact 文件时返回 4xx，且服务端不执行请求内容。
24. stop 只向 Agent 登记的 PGID 发信号；测试预置的同名非平台进程在 stop 后仍运行。
25. 平台运行时代码、配置、依赖和进程参数不引用 `masar-unified-platform`、旧 `agent.py` 或端口 `18088`；设计文档中的废弃声明不计为依赖。

### 归档

26. 成功、启动失败和用户停止三类运行均生成完整 manifest、events、process logs、summary 和 checksum。
27. 录制开启时能下载 ROS1/ROS2 对应 bag；下载支持 Range，checksum 校验通过。
28. 群行为 summary 不包含未定义的算法评分字段。

## 15. Testing Plan

| 层级 | 测试内容 | 最低数量/覆盖 |
|---|---|---|
| Unit | scenario schema、参数 allow-list、状态机、run lock、命令 allow-list、topic availability、artifact path | 每个状态转移和拒绝分支至少 1 个 |
| Agent unit | 进程组登记、日志游标、幂等 stop、信号升级、未知 PID 拒绝 | 每个信号阶段和恢复分支至少 1 个 |
| Contract | Platform API 与两个 Agent；event schema；scenario manifest | 所有公开端点和 6 个场景 |
| ROS integration | ROS1 sidecar、ROS2 sidecar、dynamic bridge、TF、video、bag | ROS1-only、ROS2-only、双容器各 1 套 |
| Script regression | 第 11.1 节命令及现有 pytest | 7 个 dry-run + 两项目现有相关测试 |
| E2E | 六个场景的启动、运行、命令、停止、归档 | 每场景至少 smoke 1 次 |
| Browser | 1440x900 与 390x844 布局、面板交互、断线恢复 | Chromium；核心视图截图和 canvas 非空检查 |
| Failure injection | readiness 超时、Agent 断开、rosbridge 断开、磁盘不足、端口冲突 | 每类至少 1 次 |

## 16. Rollback Plan

1. `make docker-stop` 先停止活动 Run，再关闭 9 个 Compose 服务；不会删除业务镜像、现有 ROS 工程、Run 归档或无关容器。
2. 现有工程兼容改动按独立提交回滚；因为参数默认关闭，紧急情况下不设置平台参数即可恢复原路径。
3. 平台模式参数保持默认关闭；在无平台 Run 时仍可进入对应 runtime 执行原顶层脚本和 Qt GUI。
4. 场景 catalog 版本不兼容时拒绝启动，不自动降级或执行旧清单中的原始命令。
5. 归档文件不在回滚时自动删除。

## 17. Effort Estimate

以下为单人熟悉代码库后的工程量范围，不含长时间物理仿真验证：

| 阶段 | 内容 | 人工估算 |
|---|---|---|
| 基础契约 | monorepo scaffold、schema、API、Agent、run lock | 5 至 8 人日 |
| Web 网关 | ROS1/ROS2 sidecar、rosbridge、TF、video | 4 至 6 人日 |
| 图形链路 | Webots stream、双 noVNC、反向代理 | 4 至 7 人日 |
| 场景接入 | 6 个 manifest、readiness、停止和兼容参数 | 7 至 10 人日 |
| 前端工作台 | 三模块、面板系统、数据适配与控制 | 8 至 12 人日 |
| 归档与下载 | events/logs/bag/summary/checksum | 4 至 6 人日 |
| 回归与 E2E | 故障注入、浏览器和原命令验收 | 5 至 8 人日 |

MVP 合计约 37 至 57 人日。历史回放和多实验对比是后续独立里程碑。

## 18. Files Reference

| 文件 | 证据或预期变更 |
|---|---|
| `ROS2swarm_w/start_simulation.sh` | 现有群行为参数与命令构造基线 |
| `ROS2swarm_w/start_robosar_webots.sh` | 双容器 Webots 入口参数与顺序基线 |
| `ROS2swarm_w/src/launch_webots/launch/start_webots.launch.py` | 默认关闭的 stream/port 透传 |
| `ROS2swarm_w/src/launch_webots/launch_webots/robot_nodes.py` | e-puck/Burger odom 与 cmd_vel 映射证据 |
| `ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py` | ROS2 robot 到 ROS1 agent 的 scan/odom/cmd 映射证据 |
| `masar_ws/launch_robosar_g.sh` | Gazebo 探索入口，不改变默认逻辑 |
| `masar_ws/launch_robosar_w.sh` | Webots ROS1 探索入口，不改变默认逻辑 |
| `masar_ws/run_response_deform_demo.sh` | DEFORM demo 入口 |
| `masar_ws/run_response_controller_demo.sh` | 响应控制器 mode allow-list 来源 |
| `masar_ws/run_tepper_response_demo.sh` | 完整搜救入口 |
| `masar_ws/src/robosar/robosar_gui/src/robosar_gui.py` | Qt 控制和观测话题契约 |
| `masar_ws/src/robosar/robosar_messages/msg/EmergencyTask.msg` | 任务类型和状态契约 |
| `masar_ws/src/robosar/robosar_messages/msg/ResponseGroupStatus.msg` | 响应组业务字段契约 |
| `masar-simulation-platform/.superpowers/visual-review/layout-v3.html` | 已确认的多面板与传感器修订原型 |

## 19. Implementation Baseline

本节是 MVP 的冻结实现基线。实现者不得用另一套技术替换它而不更新 Spec。

### 19.1 目录与技术栈

```text
masar-simulation-platform/
├── backend/masar_platform/       # FastAPI、SQLite、状态机、编排和归档
├── agents/runtime_agent/         # ROS1/ROS2 共用的标准库 HTTP Agent
├── scenarios/                    # 6 个 YAML manifest 和 JSON Schema
├── frontend/                     # React + TypeScript + Vite 工作台
├── deploy/                       # Compose、Dockerfile、Caddy、sidecar 配置
├── tests/                        # unit、contract、integration、场景 smoke 与故障注入
└── runs/                         # 运行时生成，不提交 Git
```

基线版本：Python 3.10；FastAPI 0.115.6；Uvicorn 0.34.0；Pydantic 2.10.5；PyYAML 6.0.2；SQLite 3.37+ WAL；固定构建镜像中的 Node.js 22.23.2；React 19.0.0；Vite 6.4.3；TypeScript 5.7.2；Vitest 4.1.10；Playwright 1.62.1；`roslib` 2.1.0。依赖 lockfile 必须记录这些直接依赖和所有传递依赖。Vite 与 Playwright 从原计划版本升级到同代安全修复版本，因为 2026-08-01 的 npm audit 已确认原版本存在公开漏洞；`ros3d` 1.1.0 与旧 ROS2D/EaselJS 链均不进入当前生产依赖。

当前部署由 `make docker-start` 调用 `deploy/platform.sh`，并由单一
`deploy/compose.yaml` 管理 9 个服务。两个 Runtime Agent 是业务 runtime 的容器
主进程，容器重启后自动恢复。平台不挂载 Docker socket到 API、前端、Agent 或
sidecar。依赖保存在镜像中；workspace、API/场景源码、sidecar/display/guard 配置和
前端构建产物采用 bind mount，修改普通源码不要求重建服务镜像。

首次启动在缺少 overlay 时顺序执行 ROS1 `catkin_make` 和 ROS2 `colcon build`，
完整日志写入 `.runtime/logs`。本地完整 digest 命中时不访问 registry，只拉取缺失
的公开 runtime 镜像。ROS2 runtime、sidecar、场景与 readiness 固定 Domain `42`。

Compose 中的每个服务，以及未来的 host API 和两个 Runtime Agent，都必须设置 `NO_PROXY` 与 `no_proxy` 为至少 `127.0.0.1,localhost,::1`，避免服务器代理截获本机健康检查和 WebSocket。

### 19.2 端口和 URL

除下述 Webots 原生实现例外，全部服务绑定 `127.0.0.1`，Caddy 是内部 HTTP 入口；可选的 `public-tunnel` 通过宿主机 HTTP 代理建立出站 SSH 隧道，不新增监听端口。Webots R2025a 固定使用 `QHostAddress::Any` 且没有 bind-host 参数，因此 `webots-guard` 在 host network 对 `18141` 安装 IPv4/IPv6 INPUT 规则并拒绝所有非 loopback 流量；其有效访问边界仍是 `127.0.0.1:18141`。

宿主机 workspace 支持任意绝对路径，但必须保持以下同级布局：

```text
<workspace>/masar-simulation-platform
<workspace>/masar_ws
<workspace>/ROS2swarm_w
```

`deploy/platform.sh` 默认从当前平台 checkout 的父目录推导 workspace，并接受
`MASAR_WORKSPACE_ROOT` 作为显式覆盖；容器内部统一使用 `/root/workspace`。浏览器
统一入口是 `http://127.0.0.1:18090`。远程服务器只需配置：

```sshconfig
LocalForward 18090 127.0.0.1:18090
```

本地和 SSH 转发后的浏览器地址保持一致，不单独暴露 Agent、rosbridge、noVNC、Webots
或 Docker 端口。

配置 `MASAR_PUBLIC_TUNNEL_PROXY=http://127.0.0.1:7897` 后，`public-tunnel` 会输出
Pinggy 匿名 HTTPS URL；免费 URL 最多持续 60 分钟，重连后 URL 可能变化，且不提供
平台级身份认证或只读权限。

Gazebo/RViz 显示 sidecar 必须把 noVNC 绑定到 IPv4 loopback，把底层 VNC 显式绑定到 `127.0.0.1` 与 `::1`，不得依赖 x11vnc 的隐式 IPv6 默认值。每个显示组件启动后必须立即原子记录 `run_id`、PID、PGID 和 `/proc` start ticks；停止前必须重新证明所有权，PID 被复用时拒绝发送信号。

| 服务 | 地址 | Caddy 路径 |
|---|---:|---|
| Caddy/Web | `127.0.0.1:18090` | `/` |
| Platform API | `127.0.0.1:18100` | `/api/`、`/ws/` |
| ROS1 Agent | `127.0.0.1:18101` | 不代理 |
| ROS2 Agent | `127.0.0.1:18102` | 不代理 |
| ROS1 rosbridge | `127.0.0.1:18111` | `/ros1/` |
| ROS2 rosbridge | `127.0.0.1:18112` | `/ros2/` |
| ROS1 web video | `127.0.0.1:18121` | `/video/ros1/` |
| ROS2 web video | `127.0.0.1:18122` | `/video/ros2/` |
| Gazebo noVNC | `127.0.0.1:18131` | `/display/gazebo/` |
| RViz noVNC | `127.0.0.1:18132` | `/display/rviz/` |
| Webots noVNC | `127.0.0.1:18133` | `/display/webots/` |
| Webots W3D compatibility (default off) | `127.0.0.1:18141` | 不作为生产前端路径 |

Agent 只接受含 `X-Platform-Run`、`X-Platform-Signature` 和单调 `X-Platform-Sequence` 的请求。API 与 Agent 共享初始化时生成的 32 字节 HMAC secret，secret 只存在于 `.runtime/agent.secret` 和容器内只读挂载。

场景 manifest 的 `runtime.ros1_master_uri` 与 `runtime.ros2_domain_id` 是进程、ROS Web sidecar 和 readiness 的共同图身份。平台六个场景、Humble ROS Web sidecar 与原生 `ROS2swarm_w/start_simulation.sh` 统一使用 ROS2 domain `42`，避免运行时重启 sidecar 或出现浏览器连接到错误 ROS 图。Runtime Agent 启动的所有子进程同时设置 `MASAR_PLATFORM_MODE=1`，使既有 ROS1 脚本不再创建脱离 Agent 的子会话、不启动原 Qt GUI，也不执行原生模式下的全局清理。

### 19.3 command refs

command ref 是源码中的静态数组，不接受用户 shell 字符串。参数通过 argv 列表传入：

ROS2 群控 START/STOP 发布前，Agent 必须读取当前 `/swarm_command` 订阅数，并以该数量执行 `--wait-matching-subscriptions`；发布 QoS 固定为 volatile/reliable，并保留有限的发布器存活时间。订阅数为 0、无法解析或在命令超时内无法匹配时返回失败，不得仅因 CLI 进程退出码为 0 就报告投递成功。

| ref | argv 模板摘要 | stop grace |
|---|---|---:|
| `ros2.swarm` | `vglrun -d :0 ros2 launch launch_webots create_enviroment.launch.py webots_world=command_world.wbt pattern={pattern} number_robots={n} ... webots_stream:=false`，环境 `DISPLAY=:103` | 10/5/5 s |
| `ros1.response_deform` | `./run_response_deform_demo.sh --no-rviz --vglrun --shared-master` + allowlisted launch args，环境 `DISPLAY=:101`、`ROBOSAR_VGLRUN_DEVICE=:0` | 10/5/5 s |
| `ros1.response_controller` | `./run_response_controller_demo.sh {mode} --no-rviz --vglrun --shared-master`，环境 `DISPLAY=:101`、`ROBOSAR_VGLRUN_DEVICE=:0` | 10/5/5 s |
| `ros1.exploration_gazebo` | `./launch_robosar_g.sh --no-rviz --vglrun`，环境 `DISPLAY=:101`、`ROBOSAR_VGLRUN_DEVICE=:0` | 10/5/5 s |
| `ros2.exploration_webots` | `./start_robosar_webots.sh --vglrun`，环境 `DISPLAY=:103`、`ROBOSAR_VGLRUN_DEVICE=:0`，并设置 `webots_stream=false` | 10/5/5 s |
| `ros1.exploration_webots` | `./launch_robosar_w.sh --no-rviz --no-vglrun` | 10/5/5 s |
| `ros1.tepper_response` | `./run_tepper_response_demo.sh --no-rviz --vglrun --no-gui --shared-master`，环境 `DISPLAY=:101`、`ROBOSAR_VGLRUN_DEVICE=:0` | 10/5/5 s |

三个时间分别表示 SIGINT 后等待、SIGTERM 后等待和最后 SIGKILL 前等待。RViz 由平台在独立显示 `:102` 启动；原脚本中的 RViz 参数因此显式关闭。

### 19.4 场景 readiness、profile 和 STALE

| scenario_id | required readiness | timeout | profile |
|---|---|---:|---|
| `swarm.webots.ros2swarm` | Webots noVNC、`/clock`、`/robot_0/odom` | 180 s | `swarm_sensor` |
| `control.gazebo.response-deform` | `/clock`、`/tb_0/odom`、`/response_group/status` | 120 s | `response_demo` |
| `control.gazebo.response-controller` | `/clock`、`/tb_0/odom`、`/response_group/behavior_mode` | 120 s | `response_demo` |
| `exploration.gazebo.tepper` | `/clock`、`/sim_map`、`/frontier_filter/filtered_frontiers` | 180 s | `exploration` |
| `exploration.webots.robosar` | ROS2 `/clock` + `/robot_0/odom`，ROS1 `/sim_map` + `/frontier_filter/filtered_frontiers` | 180 s | `exploration_webots` |
| `rescue.gazebo.tepper-response` | `/clock`、`/sim_map`、`/response_group/status` | 180 s | `rescue` |

每个 ROS readiness 在 manifest 中显式声明 `message_type`、`max_age_seconds`，ROS2 还可声明 `qos_profile`。控制面通过签名 `POST /v1/runs/{run_id}/probe` 将这些冻结字段发送给对应 Agent；Agent 使用无 shell、有限时长的 CLI 订阅检查 ROS graph 精确类型和一条新消息。健康采样周期为 250 ms；未在 `max_age_seconds` 内观察到新消息为 `STALE`，类型错误为 `TYPE_MISMATCH`。TCP/HTTP 探针只允许 loopback，文件探针只能位于 Agent workspace 内。

profile 固定定义：`swarm_sensor` 含 scan/odom/cmd_vel、可选 camera/IMU/红外/battery/safety、讨论状态和 Webots 世界位姿 `/global_info`；`response_demo` 含 Gazebo 世界位姿 `/gazebo/model_states`、Burger scan/IMU/odom/cmd_vel/彩色相机，以及 `/emergency/tasks`、`/response_group/active_task`、状态、行为模式、运动门和任务结果；`exploration` 含 `/sim_map`、`/information_map`、frontier、allocation、progress、agent status、emergency objects/events 及探索机器人传感器；`exploration_webots` 以 ROS1 业务话题为权威，ROS2 raw robot topic 只作为可选传感器源；`rescue` 是 exploration 与 response_demo 的并集。PointCloud2、响应组中心路径和编队槽位不在当前 profile 中。

探索 Webots 的 preflight 必须在创建 Run 子进程前验证 `ros1_bridge` 可执行、Webots noVNC 可用和 NVIDIA VirtualGL renderer。控制器 update period 必须与世界 `basicTimeStep` 一致；低频或 latched 业务话题以消息存在/最近观测为健康依据，不以固定高频率误判离线。

控制器独立 demo 不发布 `ResponseGroupStatus`，其 readiness 因此使用 fixture 持续发布的 `/response_group/behavior_mode`。DEFORM 与 controller 的平台 argv 显式使用脚本已有的 `--shared-master`；该参数仅作用于平台启动，不改变两个原脚本的默认独立 master 行为。

### 19.5 API、错误和事件恢复

所有 4xx/5xx 使用同一错误体：

```json
{
  "error": {
    "code": "RUN_ALREADY_ACTIVE",
    "message": "an active run already exists",
    "details": {"run_id": "01J..."}
  },
  "request_id": "01J..."
}
```

事件保存在 `events.jsonl`，保留 30 天或 10000 条，以先达到者为准。WebSocket 每 15 秒发送 ping；前端重连延迟为 250、500、1000、2000、4000 ms，随后固定 5000 ms。重连后先请求 `GET /runs/{run_id}/events?after=<sequence>`，再恢复 WebSocket。

SQLite 文件为 `runs/platform.sqlite3`，开启 WAL。`runs`、`run_steps`、`leases`、`events`、`artifacts` 五张表必须有主键和 `run_id` 索引。lease heartbeat 周期 2 s，过期阈值 10 s；API 重启时无法证明 Agent 所有权则状态为 `FAILED`、reason code 为 `NEEDS_INSPECTION`。

### 19.6 Manifest、摘要和 artifact 格式

每个场景必须有独立文件 `scenarios/<scenario_id>.yaml`，并在 `scenarios/catalog.yaml` 注册。六个文件都必须包含 `schema_version`、`scenario_id`、`module`、`simulator`、`parameters`、`steps`、`stop_order`、`data_profile` 和 `artifact_profile`，不得只使用第 6 节示例文件。

`manifest.json` 固定包含 `run_id`、`scenario_id`、resolved parameters、platform_version、agent_versions、started_at、ended_at、final_state、failure_reason、step_results、`artifact_ids` 和 `recording`。`summary.json` 固定包含 `run_id`、duration_seconds、final_state、missing_data[]、topic_health[]、`task_counts`、`exploration_progress_final`（无则 null）和 `response_group`（无则 null）。`artifact_id` 为 `sha256:` 加文件 SHA-256 的前 32 个十六进制字符，完整摘要保存在 `checksum`；相同内容允许共享 `artifact_id`，单次运行中的文件身份由 `(run_id, path)` 唯一确定。`checksums.sha256` 必须精确覆盖除其自身外的全部已索引文件，并使用 GNU coreutils 格式。

场景 `artifact_profile` 固定为 `none`、`swarm_default`、`exploration_default`、`rescue_default` 四种；浏览器录制请求当前只接受 `profile=default`，再由场景 profile 决定 ROS1/ROS2 图选择和最大字节率，实际 topic/type 来自对应 data profile。下载端点只支持单个 HTTP Range，并在响应头提供 `Accept-Ranges: bytes` 和 checksum。服务端必须在已打开且通过 `O_NOFOLLOW`、文件类型、长度与完整 SHA-256 校验的同一文件描述符上流式读取，避免路径替换竞态。

### 19.7 移动布局与控制认证

在 390x844 viewport，页面使用单列 CSS grid；顶栏固定高度不超过 56 px；仿真、任务分配与建图/RViz、传感器、状态、事件五类面板按顺序显示标题和状态，任一面板展开时其余面板保持可滚动且不重叠。桌面布局使用三列 grid，1440x900 首屏满足第 14 节条件。

SSH 隧道是用户认证边界；Caddy 只接受同源 `Origin`，API 对 POST 要求 CSRF header。浏览器不能访问 Agent 端口和 Docker socket。平台不提供账号系统。

### 19.8 Stage A 实测基线

Stage A 于 2026-08-02 通过。三台与五台 Burger Webots 群行为的 30 秒实测 RTF 分别为 `0.980` 和 `0.940`，均使用 NVIDIA VirtualGL 和可交互的非空 noVNC 画面。Webots 探索严格先启动 ROS2 `start_robosar_webots.sh`，其 readiness 通过后再启动 ROS1 `launch_robosar_w.sh`，停止顺序相反。

Gazebo 探索 Run `run-287b2dd9af264a0597cc9ec31dc188a1` 的 30.03 秒墙钟样本推进 26.94 秒仿真时间，RTF 为 `0.897`；Gazebo、RViz、OccupancyGrid 与真实相机浏览器验收通过。一次约 20 分钟的历史运行出现原生进程 `free(): invalid pointer`，平台已正确归档和清理；该问题属于 `masar_ws` 长运行稳定性风险，不改变 Run 所有权和停止语义。

### 19.9 Stage B 实测基线

Stage B 于 2026-08-03 完成复验。三台 e-puck 的 discussed-dispersion 与五台 e-puck 的 aggregation Run 均以固定的 `command_world.wbt` 运行；该固定参数不渲染为前端控件，机器人类型以下拉框提供 `e_puck`、`burger`。桌面端 Webots 位于上方并横跨两列，轨迹与传感器位于下方并排，移动端按三者顺序转为单列。真实面板显示多机器人世界坐标轨迹、所选机器人相机/LaserScan/八路红外、全部机器人健康、讨论观点/分散距离和 Run stdout/stderr。

讨论模式的第一次验收揭示 `ros2 topic pub --once` 只等待一个匹配订阅者，导致部分策略节点漏收 START。Agent 改为先读取当前订阅数并等待全部当前订阅者，随后全新 Run 的单次平台 START 使三台 `/robot_N/opinion` 均稳定达到 `0.5 Hz`，真实 Chromium 门禁通过。两次验收 Run 均以 `COMPLETED/USER_STOP` 结束，无 Run-owned 残留且归档 checksum 全部通过。Burger aggregation 的算法行为问题已在平台外复现，按用户决定留给后续 ROS2swarm 算法阶段，不纳入平台闭环判定。

五机器人 readiness 使用 Domain 42 的单个 rclpy Context 批量观察 `/clock` 与全部机器人 odom，避免多组 ROS2 CLI 与 controller spawner 并发争用。复验还发现 rosbridge 在主线程调用 `/rosapi/topics` 时可阻塞 WebSocket 接收循环；ROS2 Web sidecar 现使用 `call_services_in_new_thread:=true`。Run `run-4fdfe4f599094947bd10cefb97e2e064` 在 70 秒内达到 `RUNNING`，robot_2 与其余四台机器人的相机、LaserScan、八路红外、odom、健康和 SVG 轨迹均通过真实 Chromium 验收，停止后 11 个归档 checksum 全部通过。

`sensor_type=lidar` 是群行为平台 UI 的验收默认值，但不限制前端展示 e-puck 的物理红外 `ps0..ps7`。手工 API 选择 ROS2swarm 的 `sensor_type=ir` 会触发其现有 `ir_layer` 尾斜杠话题错误；该模式层问题与 Burger aggregation 一样留待 ROS2swarm 算法/节点阶段处理，不影响默认平台闭环。

世界坐标复验使用 Webots Run `run-bffc31333a064ea08e5628dc3cf87a94` 和 Gazebo DEFORM Run `run-c25cd415a2394c28a17397544edc9620`。Webots 页面从 `/global_info` 显示 `3/3 世界位姿`；Gazebo 页面从 `/gazebo/model_states` 显示 `4/4 世界位姿`，任务启动后四条轨迹各累积 81 个点。真实 Gazebo 首轮暴露 ROS1 Web sidecar 缺少 `gazebo_msgs`；补装 `ros-noetic-gazebo-msgs` 后同一订阅链路通过。两轮停止均为 `COMPLETED/USER_STOP`，无 Run 进程或 `1234/18141` 监听，归档 checksum 全部通过，业务容器未重启。空闲页显示 `0/0 世界位姿`、零轨迹/健康行和唯一“未发现机器人”传感器选项。

### 19.10 Stage C 实测基线

Stage C 于 2026-08-03 通过。Gazebo Runs `run-1e6dc13688634b0bb9bae467ffdb8ceb`、`run-4ba8c33b877149a8acf8e1da26caef8b` 和 Webots Runs `run-919530af7fae462893dfc5ebfff3bd7a`、`run-61aa650ac5814725bb95fcc1f950b284` 均完成真实浏览器、任务控制、停止清理和归档验收。Webots 两轮都严格先启动 ROS2 `start_robosar_webots.sh`，其 readiness 通过后再启动 ROS1 `launch_robosar_w.sh`，停止时按 RViz、ROS1、ROS2 逆序关闭；两个业务容器未重启。

同一机器人、同一 capability 存在多个 ROS1/ROS2 候选源时，前端按 `ONLINE`、`STALE`、已发现但等待首帧、`DISCONNECTED`、`TYPE_MISMATCH`、`UNAVAILABLE` 的顺序选择，不能让静默重复源遮住实时来源。在线探索机器人数量优先由实时 per-robot Odometry 推导，再回退到 agent-status 快照。任务分配图使用 snapshot 而非 MJPEG 长连接，解决低频 latched 单帧在 Chromium 中无法提交首帧的问题。

### 19.11 Stage D 实测基线

Stage D 于 2026-08-03 完成实机闭环。最终 Run `run-8adea267bc8f43f4b89d396b5096f1f6` 在 Gazebo/Tepper 中同时运行探索组和四台 Burger 响应组；真实 Chromium 显示任务分配图、地图覆盖、候选前沿、本轮分配、五台在线探索机器人、应急对象、机载相机、Gazebo、RViz，以及六项紧凑响应摘要。响应组实测进入导航/路径跟踪状态，活动推送任务、任务队列、运动使能、编队误差和最小净空持续更新。

响应摘要使用 `220px` 最小基准高度，任务态势使用 `180px`、其他状态面板使用 `145px` 最小基准高度。右侧状态栏按纵向流布局：高视口按权重增长填满可用高度，低视口保持最小高度并整体滚动，搜救对象列表在面板内独立滚动。自动几何断言和生产 Chromium 截图确认六项指标完整可见、不与健康面板重叠、任务文字不被裁剪且右栏无无意义底部留白。该 Run 在 `PREFLIGHT` 阶段拒绝 START 并返回 `409 RUN_NOT_ACTIVE`，进入 `RUNNING` 后任务命令成功。

停止后 Run 为 `COMPLETED/USER_STOP`，录包、RViz、`tepper_response` 按顺序关闭，三个 Agent 记录均为 `STOPPED`；`current=null`，无 Run 进程或 `1234/18141` 监听，16 个归档文件全部通过 checksum，两个业务容器的 ID、启动时间和 restart count 保持不变。

## 20. 文档入口

根目录 `README.md` 提供项目汇总和快速开始；完整运维手册、架构设计、实施计划和
验收证据分别位于 `docs/OPERATIONS.md`、`docs/PLATFORM_DESIGN.md`、
`docs/IMPLEMENTATION_PLAN.md` 和 `docs/ACCEPTANCE.md`。实施计划必须以本文件的
requirement ID 和 acceptance criteria 编号拆解，不再重新发明场景或指标。
