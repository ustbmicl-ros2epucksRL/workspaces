# MASAR 统一仿真平台可执行 Spec

状态：已批准，实施中  
版本：0.3-approved  
最后更新：2026-07-31  
对应设计：[PLATFORM_DESIGN.md](./PLATFORM_DESIGN.md)

## 1. Context

当前 `ROS2swarm_w` 与 `masar_ws` 有多个相对独立的启动入口，分别运行在 ROS2 Humble/Webots 与 ROS1 Noetic/Gazebo 容器中。操作者需要进入不同容器、按顺序执行脚本、切换 VNC 窗口并分别检查 ROS 话题，难以在一次实验中同步观察仿真、地图、RViz、传感器和任务进度。

目标用户是当前服务器上的单一实验操作者。平台完成后，用户从一个 Web 工作台选择合法场景、完成预检、启动/停止实验、发送已有任务命令、同步观察全部相关面板，并下载本次运行的日志和 rosbag。

## 2. Verified Current State

验证日期：2026-07-31。

1. `masar_ros1_srv-g` 与 `masar_ros2_bridge-w` 均运行中、使用 host network，并共同挂载 `/home/zjc/workspace:/root/workspace`。
2. ROS1 容器是 Ubuntu 20.04 + Noetic；ROS2 容器是 Ubuntu 22.04 + Humble + Webots R2025a。
3. 隔离 ROS master 上的 ROS1/ROS2 `dynamic_bridge` 通信已验证可用。
4. 两个容器当前都未安装 ROS Web Tools 网关包。
5. `WebotsLauncher.__init__` 的当前安装签名为 `stream=False`、`port='1234'`，现有 `start_webots.launch.py:38-45` 尚未传入这两个参数。
6. `start_simulation.sh:20-57` 已列出 Webots world、pattern、机器人数量、机器人类型、传感器类型和 logging 等配置，但当前 CLI 仅处理显示与 dry-run；平台不得通过修改脚本全局变量来保存一次性实验配置。
7. `masar_ws/README.md:19-137` 记录了五类现有顶层运行入口和 Webots 双容器启动顺序。
8. Qt GUI 在 `robosar_gui.py:83-108` 发布 `/system_mission_command` 与 `/response_group/return_home`，订阅任务分配图、探索完成度和应急对象。
9. `EmergencyTask.msg:3-29` 与 `ResponseGroupStatus.msg:3-26` 已提供完整搜救的业务状态字段。

## 3. Goals

### G-1 统一入口

用户通过 `http://localhost:18090` 完成场景选择、运行控制和观测，不需要手工进入两个容器执行命令。

### G-2 保持业务行为

平台运行与现有原命令运行使用同一 launch、参数默认值和算法节点。平台接入不得改变算法、任务状态机、导航、安全或控制输出。

### G-3 同屏同步观测

场景要求的仿真、RViz、地图、传感器、状态和日志默认同时显示。选择只用于单个面板内部的数据源，不用于隐藏其他核心面板。

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
- `webots_world`: 只能选择 `launch_webots/worlds` 中清单允许的 `.wbt`。
- 出生区域、最小距离、障碍物数量、Webots mode、logging 和 transport object shape 使用现有 launch 约束。

平台 wrapper 必须构造与 `start_simulation.sh:209-230` 等价的 `ros2 launch launch_webots create_enviroment.launch.py ...` 命令，不编辑 `start_simulation.sh` 文件。

### 5.2 响应控制器参数允许列表

DEFORM demo 的 `response_goal_sequence_node.py:147-156` 接收 START/STOP，并明确忽略 exploration mission HOME；响应组返航使用 `/response_group/return_home`。平台必须按这两个不同语义提供按钮。

`control.gazebo.response-controller` 的 `mode` 只允许：

- `roundflocking`
- `roundflocking-checkpoint`
- `roundflocking-heuristic`
- `transport`
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
  ros2_domain_id: 0
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
          name: /percent_completed
          message_type: std_msgs/Float32
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
    "sensor_type": "ir",
    "number_robots": 5,
    "webots_world": "command_world.wbt"
  },
  "recording": {
    "enabled": true,
    "profile": "default"
  }
}
```

成功返回 HTTP 202 和稳定 `run_id`。参数错误返回 422；存在活动运行返回 409；Agent 不可用返回 503。

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

每个数据源至少返回：

```json
{
  "graph": "ros2",
  "topic": "/robot_2/scan",
  "type": "sensor_msgs/msg/LaserScan",
  "capability": "laser",
  "availability": "ONLINE",
  "observed_hz": 9.9,
  "last_message_age_ms": 42
}
```

`observed_hz` 与 `last_message_age_ms` 是数据链路健康信息，不写入算法评价结果。

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

探索组和完整搜救场景进入 `RUNNING` 后，首屏必须同时包含仿真、RViz、任务地图和传感器四个可见面板；不得要求用户切换互斥标签才能知道另一个面板是否正常。

### UI-003 群行为布局

群行为与控制场景不显示 RViz。释放区域用于扩大仿真、位置/轨迹和传感器面板。

### UI-004 面板操作

仿真、RViz、地图、传感器和日志面板均支持调整尺寸、最大化、恢复和弹出。动态标题或状态变化不得改变固定工具栏高度。

### UI-005 数据源选择

相机、激光、机器人或地图话题选择器位于所属面板内。改变选择不卸载其他核心面板。

### UI-006 缺失数据

不存在话题时显示“未提供”；允许但尚未发布显示“等待数据”；超时显示 `STALE` 和最后消息时间；类型不匹配显示错误。以上状态不得显示伪造的实时数值。

### UI-007 群行为指标边界

`swarm.webots.ros2swarm`、`control.gazebo.response-deform` 和 `control.gazebo.response-controller` 不显示聚合度、平均间距或速度一致性等未定义指标。

### UI-008 现有业务字段

- `/percent_completed` 只出现在探索组和完整搜救。
- 任务队列和 `EmergencyTask.status` 只出现在完整搜救。
- `ResponseGroupStatus` 中的规划状态、slot error、clearance 等只在响应组实际发布该消息的场景显示。

### UI-009 图形交互

Webots、Gazebo 和 RViz 面板支持其原生鼠标拖动、旋转或缩放。最大化/恢复不得重启实验。

### UI-010 断线恢复

浏览器刷新或短暂断开后，使用 `run_id` 和事件 cursor 恢复当前运行，不重新提交启动请求。

## 10. ROS Web Gateway Requirements

### RW-001 连接

前端使用单例连接管理器分别维护 `ros1` 和 `ros2` rosbridge 连接。组件通过 adapter 获取数据，不直接创建 WebSocket。

### RW-002 允许列表

数据订阅仅允许场景 data profile 中声明的话题模式和类型。控制发布不通过浏览器 rosbridge 完成。

### RW-003 TF

需要坐标变换的地图图层通过 `tf2_web_republisher` 请求固定帧和显式可见机器人帧。当前直接使用 OccupancyGrid 与 Odometry 的二维路径不发起多余 TF 请求；默认不传输完整 TF 树。

### RW-004 视频

相机通过 `web_video_server` 提供。单个浏览器默认最多一个活动主视频流，切换后释放旧流。

### RW-005 地图

`MapPanel` 对上层提供稳定接口。MVP 使用隔离并本地打包的 ros2djs 显示 OccupancyGrid、轨迹和机器人朝向，不使用 CDN。ros3djs 1.1.0 因旧 Three.js 传递依赖未通过当前漏洞门禁，在安全版本或受维护替代品验证前不进入生产依赖。

### RW-006 背压

LaserScan/Odometry UI 更新上限 10 Hz，队列长度 1；日志虚拟化；PointCloud2 默认禁用。

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
- 录制开启时 topic 列表来自 artifact profile，不允许网页任意 `-a`。
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

7. 探索 Gazebo、探索 Webots 和完整搜救进入运行页时，仿真、RViz、地图和传感器四个面板在 1440x900 viewport 同时可见。
8. 群行为场景不渲染 RViz 面板，也不出现“聚合度”“平均间距”“速度一致性”字段。
9. 当 `/robot_2/camera` 不存在时，相机区域显示“未提供”，页面 DOM 和 WebSocket 消息中均不存在该相机的伪造数值。
10. 当已连接 topic 超过 profile 阈值未更新时，UI 在 1 个检测周期内从 ONLINE 变为 STALE，并显示最后消息 age。
11. `/percent_completed` 只在 exploration/rescue profile 中显示；群行为 profile 即使意外发现同名话题也不订阅。
12. `ResponseGroupStatus.rms_slot_error` 只在消息存在且类型匹配时显示；缺失时显示状态而不是 0。
13. 调整尺寸、最大化、恢复和弹出任一核心面板不会触发新的 `POST /runs`，也不会重启 ROS 进程。
14. 浏览器刷新后恢复同一 `run_id`，从最后事件 cursor 继续显示，启动步骤计数不增加。

### 可视化与网关

15. Webots R2025a streaming 能在通过 SSH 转发后的 Chromium 中显示非空画面，并可拖动/缩放视角。
16. Gazebo noVNC 与 RViz noVNC 在独立面板中同时显示，鼠标输入只作用于获得焦点的面板。
17. 原生地图能显示 `/sim_map`，并在 TF 可用时叠加至少一个机器人位姿。
18. 选择新的相机源后旧视频请求在 5 秒内释放；同一浏览器不保留两个主视频流。
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

1. 平台本体通过停止 host service/compose sidecar 回滚，不删除任何现有 ROS 工程或容器。
2. 现有工程兼容改动按独立提交回滚；因为参数默认关闭，紧急情况下不设置平台参数即可恢复原路径。
3. 平台失败后仍可直接进入容器运行原顶层脚本和 Qt GUI。
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
| `ROS2swarm_w/start_simulation.sh:20` | 现有群行为参数与命令构造基线 |
| `ROS2swarm_w/start_robosar_webots.sh:18` | 双容器 Webots 入口参数与顺序基线 |
| `ROS2swarm_w/src/launch_webots/launch/start_webots.launch.py:38` | 增加默认关闭的 stream/port 透传 |
| `ROS2swarm_w/src/launch_webots/launch_webots/robot_nodes.py:13` | e-puck/Burger odom 与 cmd_vel 映射证据 |
| `ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py:90` | ROS2 robot 到 ROS1 agent 的 scan/odom/cmd 映射证据 |
| `masar_ws/launch_robosar_g.sh` | Gazebo 探索入口，不改变默认逻辑 |
| `masar_ws/launch_robosar_w.sh` | Webots ROS1 探索入口，不改变默认逻辑 |
| `masar_ws/run_response_deform_demo.sh` | DEFORM demo 入口 |
| `masar_ws/run_response_controller_demo.sh` | 响应控制器 mode allow-list 来源 |
| `masar_ws/run_tepper_response_demo.sh` | 完整搜救入口 |
| `masar_ws/src/robosar/robosar_gui/src/robosar_gui.py:83` | Qt 控制和观测话题契约 |
| `masar_ws/src/robosar/robosar_messages/msg/EmergencyTask.msg:3` | 任务类型和状态契约 |
| `masar_ws/src/robosar/robosar_messages/msg/ResponseGroupStatus.msg:3` | 响应组业务字段契约 |
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
├── tests/                        # unit、contract、integration、E2E
└── runs/                         # 运行时生成，不提交 Git
```

基线版本：Python 3.10；FastAPI 0.115.6；Uvicorn 0.34.0；Pydantic 2.10.5；PyYAML 6.0.2；SQLite 3.37+ WAL；Node.js 22.13.1；React 19.0.0；Vite 6.4.3；TypeScript 5.7.2；Playwright 1.62.1；`roslib` 2.1.0；`ros2d` 0.10.0；EaselJS 1.0.2。依赖 lockfile 必须记录这些直接依赖和所有传递依赖。Vite 与 Playwright 从原计划版本升级到同代安全修复版本，因为 2026-08-01 的 npm audit 已确认原版本存在公开漏洞；`ros3d` 1.1.0 同样因其旧 Three.js 链产生两个高危审计项而从生产基线移除。

控制平面作为 host systemd 服务运行，sidecar 使用 Docker Compose；浏览器不能接触 Docker socket。两个既有业务容器只运行 Agent 和业务进程，不修改镜像层。

`deploy/compose.yaml` 可以提供带 `dev-api` profile 的 API 镜像用于构建和健康检查，但生产控制面仍以 host systemd 服务为准。Compose 中的每个服务，以及 host API 和两个 Runtime Agent，都必须设置 `NO_PROXY` 与 `no_proxy` 为至少 `127.0.0.1,localhost,::1`，避免服务器的 SSH 反向代理截获本机健康检查和 WebSocket。

### 19.2 端口和 URL

除下述 Webots 原生实现例外，全部服务绑定 `127.0.0.1`，Caddy 是唯一外部入口。Webots R2025a 固定使用 `QHostAddress::Any` 且没有 bind-host 参数，因此 `webots-guard` 在 host network 对 `18141` 安装 IPv4/IPv6 INPUT 规则并拒绝所有非 loopback 流量；其有效访问边界仍是 `127.0.0.1:18141`。

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
| Webots stream | `127.0.0.1:18141` | `/webots/` |

Agent 只接受含 `X-Platform-Run`、`X-Platform-Signature` 和单调 `X-Platform-Sequence` 的请求。API 与 Agent 共享每次运行生成的 32 字节 HMAC secret，secret 只存内存和 root-only runtime 文件。

场景 manifest 的 `runtime.ros1_master_uri` 与 `runtime.ros2_domain_id` 是进程、ROS Web sidecar 和 readiness 的共同图身份。`swarm.webots.ros2swarm` 固定使用 ROS2 domain `42`；RoboSAR Webots 桥和其余场景使用 domain `0`。Runtime Agent 启动的所有子进程同时设置 `MASAR_PLATFORM_MODE=1`，使既有 ROS1 脚本不再创建脱离 Agent 的子会话、不启动原 Qt GUI，也不执行原生模式下的全局清理。

### 19.3 command refs

command ref 是源码中的静态数组，不接受用户 shell 字符串。参数通过 argv 列表传入：

| ref | argv 模板摘要 | stop grace |
|---|---|---:|
| `ros2.swarm` | `ros2 launch launch_webots create_enviroment.launch.py webots_world={world} pattern={pattern} number_robots={n} ... webots_stream:=true webots_port:=18141` | 10/5/5 s |
| `ros1.response_deform` | `./run_response_deform_demo.sh --no-rviz --no-vglrun --shared-master` + allowlisted launch args | 10/5/5 s |
| `ros1.response_controller` | `./run_response_controller_demo.sh {mode} --no-rviz --no-vglrun --shared-master` | 10/5/5 s |
| `ros1.exploration_gazebo` | `./launch_robosar_g.sh --no-rviz --no-vglrun` | 10/5/5 s |
| `ros2.exploration_webots` | `./start_robosar_webots.sh --no-vglrun`，并设置 `webots_stream=true webots_port=18141` | 10/5/5 s |
| `ros1.exploration_webots` | `./launch_robosar_w.sh --no-rviz --no-vglrun` | 10/5/5 s |
| `ros1.tepper_response` | `./run_tepper_response_demo.sh --no-rviz --no-vglrun --no-gui --shared-master` | 10/5/5 s |

三个时间分别表示 SIGINT 后等待、SIGTERM 后等待和最后 SIGKILL 前等待。RViz 由平台在独立显示 `:102` 启动；原脚本中的 RViz 参数因此显式关闭。

### 19.4 场景 readiness、profile 和 STALE

| scenario_id | required readiness | timeout | profile |
|---|---|---:|---|
| `swarm.webots.ros2swarm` | Webots stream TCP、`/clock`、`/robot_0/odom` | 180 s | `swarm_sensor` |
| `control.gazebo.response-deform` | `/clock`、`/tb_0/odom`、`/response_group/status` | 120 s | `response_demo` |
| `control.gazebo.response-controller` | `/clock`、`/tb_0/odom`、`/response_group/behavior_mode` | 120 s | `response_demo` |
| `exploration.gazebo.tepper` | `/clock`、`/sim_map`、`/percent_completed` | 180 s | `exploration` |
| `exploration.webots.robosar` | ROS2 `/clock` + `/robot_0/odom`，ROS1 `/sim_map` + `/percent_completed` | 180 s | `exploration_webots` |
| `rescue.gazebo.tepper-response` | `/clock`、`/sim_map`、`/response_group/status` | 180 s | `rescue` |

每个 ROS readiness 在 manifest 中显式声明 `message_type`、`max_age_seconds`，ROS2 还可声明 `qos_profile`。控制面通过签名 `POST /v1/runs/{run_id}/probe` 将这些冻结字段发送给对应 Agent；Agent 使用无 shell、有限时长的 CLI 订阅检查 ROS graph 精确类型和一条新消息。健康采样周期为 250 ms；未在 `max_age_seconds` 内观察到新消息为 `STALE`，类型错误为 `TYPE_MISMATCH`。TCP/HTTP 探针只允许 loopback，文件探针只能位于 Agent workspace 内。

profile 固定定义：`swarm_sensor` 只含 scan/odom/cmd_vel/camera/battery/safety 和可选 `/global_info`；`response_demo` 含 Burger odom、cmd_vel、`ResponseGroupStatus`、behavior mode 和 task result；`exploration` 含 `/sim_map`、frontier、allocation、progress、agent status、emergency objects；`exploration_webots` 以 ROS1 业务话题为权威，ROS2 raw robot topic 只作为可选传感器源；`rescue` 是 exploration 与 response_demo 的并集。

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

`manifest.json` 固定包含 `run_id`、`scenario_id`、resolved parameters、platform_version、agent_versions、started_at、ended_at、final_state、failure_reason、step_results 和 `artifact_ids`。`summary.json` 固定包含 `run_id`、duration_seconds、final_state、missing_data[]、topic_health[]、`task_counts`、`exploration_progress_final`（无则 null）和 `response_group`（无则 null）。`artifact_id` 为 `sha256:` 加文件 SHA-256 的前 32 个十六进制字符；`checksums.sha256` 使用 GNU coreutils 格式。

录包 profile 固定为 `none`、`swarm_default`、`exploration_default`、`rescue_default` 四种。每种 profile 都列出 ROS1/ROS2 topic、消息类型和最大字节率；下载端点支持 HTTP Range，并在响应头提供 `Accept-Ranges: bytes` 和 checksum。

### 19.7 移动布局与控制认证

在 390x844 viewport，页面使用单列 CSS grid；顶栏固定高度不超过 56 px；仿真、地图/RViz、传感器、状态、事件五个面板按顺序显示标题和状态，任一面板展开时其余面板保持可滚动且不重叠。桌面布局使用三列 grid，1440x900 首屏满足第 14 节条件。

SSH 隧道是用户认证边界；Caddy 只接受同源 `Origin`，API 对 POST 要求 CSRF header。浏览器不能访问 Agent 端口和 Docker socket。平台不提供账号系统。

## 20. Spec Review Checklist

书面批准本 Spec 前，请只检查以下设计结论是否准确：

1. 六个 MVP 场景是否覆盖当前要统一操作的入口。
2. 完整搜救 MVP 仅列 Gazebo/Tepper 是否符合当前真实入口。
3. 群行为模块的数据真实性边界是否正确。
4. 平台模式下跳过 Qt、由 Web 控制，但原命令保留 Qt 的方式是否正确。
5. `18090` 作为新统一入口是否可接受。

实施计划必须以本文件的 requirement ID 和 acceptance criteria 编号拆解，不再重新发明场景或指标。
