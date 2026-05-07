# RoboSAR 轻量应急任务设计文档

## 1. 文档目标

本文档基于你最新给出的约束，重新定义当前系统中的应急模块设计。

本次设计明确采用轻量方案，不再走之前那种复杂的事件融合、严重度评估、多层状态机路线，而是围绕下面这条简单逻辑展开：

1. 事件由 GUI 手动触发，或者由随机事件节点生成
2. 事件通过 ROS 广播
3. 当前系统中距离事件最近的 `n` 台机器人被选中
4. 这 `n` 台机器人暂时退出当前探索/覆盖任务
5. 它们切换到 ROS2 `ROS2swarm_w` 中已经训练好的应急控制器
6. 任务结束后，这些机器人再回到 ROS1 的探索覆盖流程

本版方案默认：

- 不考虑火源大小
- 不考虑障碍物重量
- 不做复杂事件推理
- 不处理多事件并发
- 同类事件统一按同一种模板执行

也就是说，先把“事件触发 -> 机器人切换 -> 完成后恢复”这条最小闭环跑通。

---

## 2. 当前系统的真实基础

## 2.1 ROS1 `catkin_ws2` 的职责

当前 `/root/catkin_ws2` 仍然是整个系统的主任务链，主要负责：

- 多机器人 SLAM
- frontier / coverage 任务生成
- 任务分配
- 路径规划
- 控制器执行
- GUI

当前主入口仍然是：

- `/root/catkin_ws2/launch_robosar.sh`

现有正常工作流可以概括为：

1. SLAM 建图
2. Task Generator 生成 frontier / coverage 点
3. Task Allocator 分配目标
4. Mission Executive 做路径下发
5. Controller 控制机器人执行

也就是说，ROS1 这边已经具备“常规探索覆盖任务”的完整闭环。

## 2.2 ROS2 `ROS2swarm_w` 的职责

你指出的仓库实际位于：

- `/root/ROS2swarm_w`

我已经确认其中存在你提到的训练好的 checkpoint 控制器目录：

- `/root/ROS2swarm_w/src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/navigation`
- `/root/ROS2swarm_w/src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/roundflocking`
- `/root/ROS2swarm_w/src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/transport`

并且对应的 ROS2 pattern 代码已经存在：

- `navigation_pattern.py`
- `roundflocking_pattern.py`
- `transport_pattern.py`

从代码可以确认：

1. `navigation_pattern`
   - 用于导航到目标点
   - 读取 `/global_info`
   - 使用 `navigation_target_<robot_id>` 作为每台机器人各自目标

2. `roundflocking_pattern`
   - 用于环绕目标
   - 读取 `/global_info`
   - 目标对象名为 `flocking_target`

3. `transport_pattern`
   - 用于协作运输/推动物体
   - 读取 `/global_info`
   - 任务对象名为 `transport_box`
   - 目标区域名为 `target_zone`

这意味着：

- 火源类应急任务可以直接映射到 `roundflocking_pattern`
- 坍塌/推障类应急任务可以直接映射到 `transport_pattern`

## 2.3 ROS2 侧当前还能提供什么

我还确认了两个关键节点：

1. `global_info_supervisor.py`

路径：

- `/root/ROS2swarm_w/src/launch_webots/launch_webots/global_info_supervisor.py`

作用：

- 作为 Webots Supervisor 发布 `/global_info`
- 把场景中机器人和物体的位置、速度统一发布出去

它会识别的任务对象包括：

- `flocking_target`
- `transport_box`
- `target_zone`
- `navigation_target_*`

2. `robosar_bridge_adapter.py`

路径：

- `/root/ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py`

作用：

- 把 ROS2 Webots 侧的 `robot_N` 话题映射到 ROS1 RoboSAR 侧的 `agent_N`
- 保持当前 ROS1 算法链能够直接使用 Webots 侧机器人

因此，当前架构已经天然具备：

- ROS1 负责探索闭环
- ROS2 负责 Webots 任务控制模式
- `ros1_bridge` 负责双边通信

这正适合做一个轻量的应急切换模块。

---

## 3. 本次重写后的设计原则

本次设计只保留三条核心原则。

### 3.1 原则一：ROS1 只做“编排”，ROS2 做“应急动作”

本版不在 ROS1 中重新发明火源环绕控制器或协作推障控制器。

ROS1 只负责：

- 产生事件
- 选最近的 `n` 台机器人
- 暂停这些机器人当前探索任务
- 把事件和机器人名单发给 ROS2
- 等待 ROS2 回报完成
- 释放机器人回到探索池

ROS2 负责：

- 把事件变成 Webots 可识别的目标对象
- 拉起对应 pattern
- 控制选中的机器人执行应急动作
- 发布任务完成状态

### 3.2 原则二：只支持统一模板事件

不做：

- 火源大小估计
- 重量估计
- 复杂碰撞建模
- 多级任务优先级

只做两个统一模板：

1. 火源事件
   - 一个事件点
   - 最近 `n_fire` 台机器人执行 `roundflocking_pattern`

2. 坍塌事件
   - 一个事件点
   - 最近 `n_collapse` 台机器人执行 `transport_pattern`

### 3.3 原则三：一期只支持单个活动事件

为避免复杂调度，一期明确建议：

- 同一时刻只允许一个活动应急事件

这样可以大幅降低实现复杂度。

---

## 4. 事件与控制器的直接映射

## 4.1 火源事件

推荐映射：

- ROS1 事件类型：`fire`
- ROS2 执行控制器：`roundflocking_pattern`
- Webots 目标对象：`flocking_target`

执行方式：

1. 在 Webots 中把 `flocking_target` 移到事件点
2. 选中的 `n_fire` 台机器人启动 `roundflocking_pattern`
3. 机器人自动围绕该目标执行环绕行为

### 任务结束条件

本版推荐简单做法：

- 到达固定持续时间后结束
- 或 GUI 手工点击“结束”

不建议一期就做自动灭火判定。

## 4.2 坍塌事件

推荐映射：

- ROS1 事件类型：`collapse`
- ROS2 执行控制器：`transport_pattern`
- Webots 任务对象：`transport_box`
- Webots 目标区域：`target_zone`

执行方式：

1. 在 Webots 中把 `transport_box` 放到事件点
2. 用固定规则设置 `target_zone`
3. 选中的 `n_collapse` 台机器人启动 `transport_pattern`
4. 机器人执行统一的协作推动/运输动作

### 关于 `target_zone` 的简化约定

由于本版不考虑障碍物重量和精细推理，推荐直接采用统一模板：

- `target_zone = event_point + fixed_offset`

例如：

- `(x + 0.6, y)`

或按地图中预设的统一朝向偏移。

也就是说，所有坍塌事件都按同一个“推到某个固定偏移位置”的模板处理。

### 任务结束条件

`transport_pattern` 本身就基于：

- `transport_box`
- `target_zone`
- `goal_threshold`

因此可以直接由 ROS2 侧判断是否完成。

## 4.3 `navigation_pattern` 的定位

`navigation_pattern` 当前可以保留，但在一期轻量方案里不是主角。

原因：

- `roundflocking_pattern` 自身就会朝 `flocking_target` 收敛
- `transport_pattern` 自身就会围绕 `transport_box` 和 `target_zone` 执行

所以一期可以不额外做“先导航到任务点，再切换控制器”这一步。

如果后续需要：

- 先到集合点
- 再开始环绕/推障

再把 `navigation_pattern` 加入流程即可。

---

## 5. 轻量应急模块的推荐结构

本版建议只增加 4 个小模块。

## 5.1 ROS1：事件发布器

作用：

- 从 GUI 或随机生成器创建事件

建议名字：

- `emergency_event_publisher`

输入来源：

- GUI 手工点击
- 定时随机生成

输出：

- `/emergency/event`

## 5.2 ROS1：最近机器人选择器

作用：

- 收到事件后，从当前机器人中选出距离事件最近的 `n` 台

建议名字：

- `emergency_selector`

功能：

1. 根据 `map -> agent_i/base_link` TF 获取机器人位置
2. 按欧氏距离排序
3. 选前 `n`
4. 发布锁定机器人列表
5. 把事件和选中机器人信息广播给 ROS2

## 5.3 ROS2：事件场景适配器

作用：

- 把 ROS1 发来的简单事件，转换成 Webots 中的真实任务对象位置

建议名字：

- `event_scene_adapter`

功能：

1. 对火源事件：
   - 移动 `flocking_target`

2. 对坍塌事件：
   - 移动 `transport_box`
   - 设置 `target_zone`

## 5.4 ROS2：Pattern 执行器

作用：

- 为选中的机器人拉起对应模式
- 任务结束后关闭对应模式

建议名字：

- `pattern_executor`

它的职责不是自己做控制，而是：

- 根据事件类型启动 `roundflocking_pattern` 或 `transport_pattern`
- 作用域仅限被选中的机器人 namespace

---

## 6. 推荐的通信设计

为了保持简单，本版建议一期直接用标准消息，不先引入复杂自定义接口包。

## 6.1 推荐方案：先用 `std_msgs/String` + JSON

原因：

- ROS1 和 ROS2 都能直接桥接
- 不需要额外维护一套跨版本自定义 msg
- 当前 ROS2 `global_info` 本身就是 JSON 风格

### 事件话题

话题：

- `/emergency/event`

消息类型：

- `std_msgs/String`

内容示例：

```json
{
  "event_id": "event_001",
  "type": "fire",
  "x": 1.2,
  "y": -0.8,
  "robot_count": 3,
  "timeout": 60.0
}
```

坍塌示例：

```json
{
  "event_id": "event_002",
  "type": "collapse",
  "x": 0.5,
  "y": -1.0,
  "robot_count": 3,
  "target_offset_x": 0.6,
  "target_offset_y": 0.0,
  "timeout": 90.0
}
```

### 机器人锁定话题

话题：

- `/emergency/locked_agents`

消息类型：

- `std_msgs/String`

内容示例：

```json
{
  "event_id": "event_001",
  "agents": ["agent_0", "agent_2", "agent_4"]
}
```

### ROS2 完成回传话题

话题：

- `/emergency/result`

消息类型：

- `std_msgs/String`

内容示例：

```json
{
  "event_id": "event_001",
  "status": "done",
  "agents": ["agent_0", "agent_2", "agent_4"]
}
```

## 6.2 后续再升级为自定义 msg

如果一期跑通后再追求规范性，再把 JSON 换成自定义 msg 即可。

一期不建议先花时间在消息包维护上。

---

## 7. 整体执行流程

下面是本版最重要的执行闭环。

## 7.1 正常状态

系统平时按当前已有逻辑运行：

1. ROS1 生成 frontier / coverage 任务
2. 机器人正常探索
3. ROS2 只负责 Webots 平台与桥接

## 7.2 事件生成

事件有两种来源：

1. GUI 手动生成
2. 随机事件节点自动生成

生成后发布：

- `/emergency/event`

## 7.3 ROS1 选机器人

`emergency_selector` 收到事件后：

1. 读取所有活跃机器人位姿
2. 计算到事件点距离
3. 选最近 `n` 台
4. 发布 `/emergency/locked_agents`

同时内部记录：

- 当前活动事件 ID
- 当前参与机器人

## 7.4 ROS1 暂停这些机器人的探索控制

这是当前集成里最关键的一步。

因为如果不暂停：

- ROS1 的探索控制器仍会给这些机器人发 `cmd_vel`
- ROS2 的 pattern 也会给这些机器人发 `cmd_vel`
- 两边会冲突

本版推荐最小修改方式：

### 修改 `robosar_controller`

让 `lazy_traffic_controller` 订阅：

- `/emergency/locked_agents`

对被锁定的机器人：

- 不再发送探索控制速度
- 只发送一次零速度后退出控制

这样 ROS2 pattern 就可以安全接管这些机器人。

### 修改 `high_ta.py`

让 `high_ta.py` 也订阅：

- `/emergency/locked_agents`

对被锁定机器人：

- 不再分配新的 frontier / coverage 任务

这样探索系统就不会继续抢占这些机器人。

## 7.5 事件桥接到 ROS2

`ros1_bridge` 将：

- `/emergency/event`
- `/emergency/locked_agents`

桥接到 ROS2。

## 7.6 ROS2 创建任务场景

`event_scene_adapter` 在 ROS2 侧收到事件后：

### 如果是火源

- 把 `flocking_target` 放到 `(x, y)`

### 如果是坍塌

- 把 `transport_box` 放到 `(x, y)`
- 把 `target_zone` 放到 `(x + dx, y + dy)`

这样 `/global_info` 就会自动反映这些对象位置，现成的 pattern 可以直接读取。

## 7.7 ROS2 执行对应 pattern

`pattern_executor` 根据事件类型和选中机器人，启动对应模式：

### 火源

- 对选中机器人拉起 `roundflocking_pattern`

### 坍塌

- 对选中机器人拉起 `transport_pattern`

然后给这些 pattern 发送启动命令：

- `/swarm_command = START`

注意：

- 本版建议 pattern 只对选中的机器人按需启动
- 不建议让所有 pattern 常驻运行

这样可以避免 `/swarm_command` 的全局语义带来干扰。

## 7.8 任务完成

### 火源

建议一期由以下任一条件结束：

1. 固定时间到
2. GUI 点击结束

### 坍塌

建议由 ROS2 根据 `transport_box` 与 `target_zone` 的距离判定完成，并回发：

- `/emergency/result`

## 7.9 回到探索

ROS1 收到 `/emergency/result` 后：

1. 清除锁定机器人列表
2. 通知 `robosar_controller` 恢复对这些机器人的探索控制
3. 通知 `high_ta.py` 把这些机器人重新加入可分配集合
4. 这些机器人重新接收 frontier / coverage 任务

到这里，应急闭环结束。

---

## 8. 需要修改或新增的内容

## 8.1 ROS1 侧

### A. GUI 增加事件按钮

当前 GUI 需要增加：

- 生成火源事件
- 生成坍塌事件
- 结束当前事件

也可以额外加：

- 随机生成事件

### B. 新增 `emergency_event_publisher`

建议新建一个简单 Python 节点，负责：

- 接收 GUI 输入
- 或定时随机生成
- 发布 `/emergency/event`

### C. 新增 `emergency_selector`

负责：

- 选最近 `n` 台机器人
- 发布锁定列表
- 维护当前事件状态

### D. 修改 `high_ta.py`

新增逻辑：

- 忽略 `/emergency/locked_agents` 中的机器人

### E. 修改 `lazy_traffic_controller.cpp`

新增逻辑：

- 对锁定机器人停止探索控制输出

### F. 修改 `launch_robosar.sh`

新增启动：

- `emergency_event_publisher`
- `emergency_selector`

## 8.2 ROS2 侧

### A. 新增 `event_scene_adapter.py`

职责：

- 监听桥接过来的 `/emergency/event`
- 调整 Webots 任务对象位置

### B. 新增 `pattern_executor.py`

职责：

- 根据事件类型和 selected robots 拉起 pattern
- 结束时关闭 pattern
- 发布 `/emergency/result`

### C. 可选：调整启动脚本

如果你想让这套机制开机即用，可以在：

- `/root/ROS2swarm_w/start_robosar_webots.sh`

或：

- `/root/ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py`

中加入对应 executor/adapter 节点。

---

## 9. 一期最小实现建议

如果目标是尽快跑通，不建议一步做到很复杂。

推荐按以下顺序做。

## 第一步：先只做 GUI 手工触发火源事件

目标：

- GUI 点击后
- 最近 `n` 台机器人退出探索
- ROS2 执行 `roundflocking_pattern`
- 时间到后恢复探索

这一步只需要打通：

- ROS1 事件发布
- 最近机器人选择
- controller handover
- ROS2 scene adapter
- ROS2 pattern executor

## 第二步：再做坍塌事件

目标：

- GUI 点击后
- 最近 `n` 台机器人执行 `transport_pattern`
- 结束后恢复探索

这里的关键是固定 `target_zone` 的设置规则。

## 第三步：再加入随机事件

随机事件节点只是在第一步和第二步之上增加另一种输入源，不影响整体结构。

---

## 10. 本版设计为什么合适

这个轻量方案之所以适合你当前系统，是因为它顺着现有代码来，而不是逆着现有代码去重构。

### 10.1 充分复用了 ROS1 现有探索链

不需要重写：

- SLAM
- Task Graph
- Frontier Filter
- Mission Executive
- 原有探索任务逻辑

只需要在选中机器人时把它们临时从探索池里拿出来。

### 10.2 充分复用了 ROS2 已训练好的模式控制器

不需要重写：

- 火源环绕控制器
- 协作推障控制器

只需要：

- 把事件位置变成 Webots 目标对象
- 启动对应 pattern

### 10.3 最大限度降低了桥接复杂度

本版不建议一开始就新增复杂自定义 msg/action，而是：

- 用标准话题先打通闭环

这对于跨 ROS1 / ROS2 系统尤其重要。

---

## 11. 需要特别注意的两个实现点

## 11.1 ROS1 控制器和 ROS2 Pattern 不能同时控制同一台机器人

这是本方案里最关键的技术点。

必须保证：

- 一旦机器人进入应急模式
- ROS1 探索控制器就停止对该机器人发控制

否则速度话题会冲突。

## 11.2 一期不要做多事件并发

因为一旦并发：

- 机器人选择会冲突
- 事件恢复会复杂
- ROS2 pattern 启动和停止管理也更复杂

所以一期强烈建议：

- 只允许一个 active event

---

## 12. 最终推荐方案

基于当前代码和你给出的新约束，我建议最终采用下面这套最小可行方案：

1. ROS1 继续负责常规探索任务
2. GUI 或随机节点发布简单事件
3. ROS1 选择最近 `n` 台机器人并锁定
4. ROS1 停止这 `n` 台机器人的探索控制输出
5. ROS2 根据事件类型把 Webots 任务对象移动到事件点
6. ROS2 为这 `n` 台机器人启动现成的 checkpoint pattern
   - 火源 -> `roundflocking_pattern`
   - 坍塌 -> `transport_pattern`
7. 任务完成后 ROS2 回传结果
8. ROS1 解除锁定，这些机器人重新回到探索覆盖任务

这套方案改动小、路径清晰，而且完全符合你现在的目标：

- 不复杂
- 能演示
- 能闭环
- 便于后续逐步增强

---

## 13. 相关代码位置

ROS1：

- `/root/catkin_ws2/launch_robosar.sh`
- `/root/catkin_ws2/src/robosar/robosar_task_allocator/src/high_ta.py`
- `/root/catkin_ws2/src/robosar/robosar_controller/src/lazy_traffic_controller.cpp`
- `/root/catkin_ws2/src/robosar/robosar_gui/src/robosar_gui.py`

ROS2：

- `/root/ROS2swarm_w/src/ros2swarm/ros2swarm/movement_pattern/basic/navigation_pattern.py`
- `/root/ROS2swarm_w/src/ros2swarm/ros2swarm/movement_pattern/basic/roundflocking_pattern.py`
- `/root/ROS2swarm_w/src/ros2swarm/ros2swarm/movement_pattern/basic/transport_pattern.py`
- `/root/ROS2swarm_w/src/ros2swarm/launch/pattern/movement_pattern/basic/checkpoints/`
- `/root/ROS2swarm_w/src/launch_webots/launch_webots/global_info_supervisor.py`
- `/root/ROS2swarm_w/src/launch_webots/launch_webots/robosar_bridge_adapter.py`

