# 响应组可泛化编队导航重构需求与设计基线

状态：已确认；阶段 1--4 已完成，干净 Tepper 整链验收通过  
日期：2026-07-07  
适用范围：`run_response_deform_demo.sh` 与 `run_tepper_response_demo.sh` 的响应组导航、任务切换、暂停/恢复和返航链路  
关联现状文档：`docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md` 继续保留为当前实现说明；本文在响应组导航部分优先级更高

## 1. 结论

当前 demo 表现好，但 Tepper 中的问题不是再调几个阈值就能解决。核心结构存在四个缺陷：

1. 任务状态、编队规划和最终速度输出没有共同的运行门控；启动顺序和 latched 消息足以让未 START 的机器人运动。
2. 四台机器人各自重复计算同一条中心路径和同一套缩放，既浪费计算，也无法保证它们在同一个中心参考、同一个尺度和同一个重规划周期上工作。
3. 当前全局规划是“二值膨胀地图上的最短 A*”，只保证中心路径可通，不主动偏向通道中线；缩放通过固定比例逐次试错，无路后又进入局部恢复分支。
4. 编队质量、到达判据和任务目标几何互相脱节。导航、环绕、推箱和返航应该使用不同的目标构型，但共享同一个中心规划与安全约束。

推荐重构为一个集中式虚拟编队中心导航器：全局层只规划一次编队中心路径，局部层用连续仿射变换生成四个槽位，并允许每个槽位在传感器约束下产生小幅、零均值偏移。每台 TB3 跟踪自己的动态槽位，最终命令继续通过唯一的 `cmd_vel` 复用/硬安全门。

这不是完整 MPC，也不是把整个 ROS navigation stack 再复制一套。它保留当前可用的任务管理、任务控制器、回家位姿捕获和速度仲裁，只替换已经证明不具泛化性的编队导航内核。

实现必须遵守单主链原则：响应组运行时只能有一个中心规划器、一个路径进度源、一个到达状态源和一个最终速度发布链。模块级替代通过测试后应在同一阶段完成运行切换并删除冲突的响应专用旧实现；不得用 feature flag 长期并存两套规划/到达逻辑。vendor 的独立兼容 demo 可以保留旧可执行文件，但不得再被 response launch 引用。

## 2. 重构前实现的代码事实

本节保留 2026-07-07 重构开始前的事实快照，用于解释方案来源；阶段完成情况与最新运行证据见第 13 节及 `docs/RESPONSE_FORMATION_REFACTOR_PROGRESS.md`。

### 2.1 START/STOP/恢复链路

- GUI 的 START/STOP 发布到 `/system_mission_command`：`robosar_gui.py:251-270`。
- Tepper 任务管理器启动时会发布 `STOP`：`emergency_task_manager_node.py:168-177`。
- 速度复用节点却仍允许从参数读取初始模式，代码默认值是 `NAV_FORMATION`：`response_cmd_mux_node.py:56-58`；demo 配置也明确使用 `default_mode: NAV_FORMATION`：`response_group.yaml:50-54`。
- DEFORM 控制器只要收到过目标，就在自己的定时器里持续发布 `/tb_i/deform_cmd_vel`；它不知道 GUI 是否运行：`plan_manager.cpp:289-408`。
- STOP 后再次 START 时，如果任务仍处于 EN_ROUTE/ACTING，`_dispatch_if_idle()` 会因为已有 active task 直接返回，未恢复 `NAV_FORMATION` 或 `TASK_CONTROL`：`emergency_task_manager_node.py:205-218,291-325`。

因此，当前系统只有“控制源选择”，没有独立且默认关闭的“运动许可”。这解释了未 START 运动和 STOP 后无法可靠恢复两个现象。

### 2.2 手动返航链路

- GUI 的响应组返航按钮发布 `/response_group/return_home`：`robosar_gui.py:278-280`。
- 第 0 阶段 `response_goal_sequence_node.py` 订阅该话题。
- Tepper 使用的 `emergency_task_manager_node.py` 没有订阅该话题；所以 Tepper 的响应组返航按钮当前没有完整接收链路。
- Tepper 配置默认 `auto_return_home: true`：`tepper_response_group.yaml:20-24`，这与“任务结束后由命令触发返航”不一致。

### 2.3 中心路径和重复规划

- `response_group_deform.launch` 启动四个 `plan_manager_node`。
- 每个实例都订阅全员 odom、完整地图和动态模型，并各自运行一次 A*：`plan_manager.cpp:21-64,526-688`。
- 四个实例还共同发布同一个 latched `/response_group/center_path`：`plan_manager.cpp:58-61`，可视化结果由最后发布者覆盖。
- A* 的移动代价只有直移/斜移距离，不含离墙距离：`plan_manager.cpp:635-654`。即使编队包络膨胀后路径合法，也会贴着膨胀边缘走，而不是靠近走廊中部。
- 地图膨胀使用正方形编队的外接圆：`plan_manager.cpp:690-748`。它安全但保守，且不能表达沿路径方向和横向不同的形变。

### 2.4 硬编码缩放、凝聚和无路恢复

- 规划器从 1.0 开始按 `formation_scale_step` 枚举到 `min_formation_scale`：`plan_manager.cpp:531-554`。
- Tepper 当前参数是边长 0.6 m、最小比例 0.45、步长 0.10，对应最小边长 0.27 m：`tepper_response_demo.launch:18-25`。
- 临近目标时 `commandedFormationScale()` 强制恢复到 1.0：`plan_manager.cpp:984-1005`。
- 凝聚逻辑会按最大槽位误差给领先车缩放线速度：`formation_assignment.cpp:71-125`，Tepper 将等待比例设为 0。
- 有地图且无路时进入 `path_blocked_`，再控制机器人向“当前中心槽位”收敛并反复标脏重规划：`plan_manager.cpp:321-371`。
- 没有地图时会创建“中心到目标”的直线路径：`plan_manager.cpp:508-519,557-562`。

这些分支彼此补偿，但没有一个统一的可行性模型。复杂环境越多，分支之间越容易振荡。

### 2.5 到达和任务几何

- 当前协调节点同时计算严格槽位、宽松目标区域和 `deform_reached`，条件存在重叠：`response_group_coordinator_node.py:208-255`。
- Tepper 的普通目标区域和 transport 目标区域使用不同大半径，但都只检查距 staging 中心的距离，无法区分“围绕目标”“在箱后排成两列”等几何质量。
- `compute_staging_pose()` 已正确计算 transport 推送方向的反向位置：`task_manager.py:110-135`，这部分应保留并加强。
- victim/fire 当前仍在目标前方后撤 1 m，而不是把编队中心送到环绕目标：`tepper_response_group.yaml:45-52`。

### 2.6 可复用能力评判

可以直接保留：

- `ResponseTaskQueue` 的去重、优先级和任务状态模型。
- `compute_staging_pose()` 的 transport 推送方向计算。
- `home_pose.py` 的首次 odom 捕获和编队中心计算。
- `response_task_controller_node.py`、roundflocking/transport checkpoint 与 TB3 适配。
- `response_cmd_mux_node.py` 的单一最终 `/tb_i/cmd_vel` 发布权和命令超时机制。
- 探索组 `MissionExecutive::processEStop()` 调用 traffic controller 清路径并发零速的链路。

只参考、不直接复用：

- 探索组 `MultiAStar` 适合多台独立搜索机器人，不表达一个可变形编队的中心和占位包络。
- 探索组 `lazy_traffic_rvo.hpp` 提供了 time-to-collision 和邻居建模思路，但当前候选速度依赖随机采样，接口与探索组 `Agent` 强耦合。响应组应复用其数学思路和测试样例，不直接调用整套控制器。
- ROS `move_base`/DWA 已在探索组使用，但为虚拟编队中心再启动一套 move_base，需要伪造 base frame、聚合传感器并把中心速度重新分配到四台差速车，集成成本高于本项目需要。

应删除或替换：

- 四份独立中心 A*。
- `formation_scale_step` 枚举缩放。
- `formation_cohesion_*` 领先车停车逻辑。
- 临近目标强制恢复标准正方形。
- 缺地图直线路径兜底。
- blocked direct/hold recovery 分支。
- Gazebo model states 作为主安全感知；它只能作为仿真诊断输入。

## 3. 目标与非目标

### 3.1 目标

1. demo 与 Tepper 共用同一套编队导航内核，只通过地图、出生点和任务配置区分环境。
2. 未 START 时两组都不运动；STOP 后两组在一个控制周期内停止；START 后从当前任务阶段恢复。
3. 响应组中心始终以一条共享的全局路径为参考；全局路径倾向走廊中部，并按编队最低可行包络检查通行性。
4. 编队尺度连续变化，允许纵向和横向不同缩放，并允许个别槽位短时小幅偏移；无需列举 0.9、0.8 等形状档位。
5. 静态地图负责全局可达性，四台 LiDAR、odom 和邻车状态负责局部避障与临时形变；仿真专有的 model states 不成为正确性前提。
6. 无路时停止并分类报告原因。动态阻塞持续重规划，静态不可达明确失败，不向目标盲走。
7. 环绕任务把编队中心送到目标；transport 把编队中心送到箱体后方并按推送方向排成两列；返航恢复捕获的初始中心和可观测队形。
8. 到达判据以中心误差、编队质量、碰撞安全和稳定保持为准，不要求数学上完美的四个角点。

### 3.2 非目标

- 不在本轮替换 roundflocking/transport 的 checkpoint 或解析式任务控制器。
- 不引入分布式非线性 MPC、CasADi 在线优化或新的学习策略。
- 不把响应组加入探索组任务分配器。
- 不承诺任意窄通道都可通过；小于四台 TB3 物理无碰撞最小构型的通道必须判定不可达。
- 不依赖 Gazebo 模型命名来保证真实机器人安全。

## 4. 需求基线

下列 MUST 是实现验收条件；SHOULD 是默认实现方向，若实现阶段发现成本或平台限制，变更必须回写本文。

### R-001 统一运行门控

- 最终速度复用层 MUST 维护独立的 `motion_enabled`，启动默认值 MUST 为 false。
- 当前环境中的唯一任务源（demo 的 goal sequence 或 Tepper 的 emergency task manager）MUST 以 latched Bool 发布 `/response_group/motion_enabled`；导航器和最终 mux 只消费该状态，不各自解释 GUI 命令。
- `motion_enabled=false` 时，无论 deform/task 输入是否新鲜，所有 `/tb_i/cmd_vel` MUST 为零。
- GUI START MUST 同时启动/恢复探索组与响应组。
- GUI STOP MUST 同时停止两组；响应组保留 active task、路径目标和任务阶段，探索组清空当前交通控制路径并可由后续 START 重新分配。
- 响应组 START 恢复时，EN_ROUTE 恢复 `NAV_FORMATION`，ACTING 恢复 `TASK_CONTROL`，RETURNING_HOME 恢复返航。
- START/STOP 必须幂等；重复点击不应重复派发任务或重置 home。
- Tepper 启动脚本 MUST 在启动 GUI 前启动响应 overlay 并确认 task source 已发布初始 `motion_enabled=false`，避免 GUI 首次命令在订阅者尚未存在时丢失。

### R-002 分组返航

- 探索组 HOME 只触发探索组返航。
- 响应组 HOME 只触发响应组返航，并可从 STOP 状态获得返航运动许可。
- 响应组 home MUST 使用启动后首次完整 odom 快照计算的中心；配置坐标只在完整 odom 超时后作为显式报错信息，不应静默替代。
- 默认关闭自动返航；所有任务完成后进入 `COMPLETE_WAITING_FOR_HOME`。
- 返航途中 STOP 后可由 START 继续。

### R-003 单一中心规划

- 一个响应组 MUST 只有一个全局路径规划实例。
- 规划输入为当前实测编队中心、任务定义的中心目标和 `/sim_map`/`/map` OccupancyGrid。
- 四台机器人 MUST 消费同一条中心路径、同一个路径进度和同一组形变参数。
- 中心参考点必须由中心在路径上的投影加前视距离得到，不能由每台车独立寻找最近路径点。
- 路径输出和调试状态必须有唯一发布者。

### R-004 通道中部偏好

- 静态地图 MUST 先生成障碍距离场/clearance field。
- 不满足最小物理编队包络的格子 MUST 不可通行。
- 可通行格子的 A* 代价 SHOULD 同时包含路径长度和离墙惩罚，使宽走廊中的解偏向中部。
- 斜向扩展 MUST 禁止穿越两个相邻占用格形成的角点。
- 路径简化/平滑 MUST 对最小编队包络做连续碰撞检查，不能只检查折线端点。
- 起点或目标非法时 MUST 明确报告，不允许无界搜索“最近自由格”后悄悄改变任务目标。

推荐代价形式：

```text
step_cost = segment_length * (1 + clearance_weight * penalty(clearance))
penalty(d) = 1 / max(d - minimum_envelope_radius, map_resolution)^2
```

权重只影响“在多条可行路中偏向中间”的程度；硬安全由最小包络约束保证。

### R-005 连续、非刚性编队形变

名义槽位 `q_i` 仍为以中心为原点的正方形角点。运行槽位定义为：

```text
p_i* = c_ref + R(path_yaw) * A(s_longitudinal, s_lateral) * q_i + delta_i
sum(delta_i) = 0
```

- `A` MUST 支持沿路径和垂直路径两个方向的连续缩放，而不是固定形状编号。
- 最小缩放 MUST 从 `robot_radius`、内部安全间距和名义槽位几何推导，不用独立的 `min_formation_scale` 魔数。
- 当前可用尺度 MUST 从路径前方 clearance 和四车共享的 LiDAR 局部可行性计算；不能运行多次完整 A* 逐档试比例。
- 尺度变化 MUST 做速率限制和滞回，防止通道边界附近来回收放。
- `delta_i` 允许单车为障碍临时内收/外移；偏移由局部安全可行域限制，并通过零均值约束避免把实际编队中心拉离全局路径。
- 离开障碍后槽位偏移 SHOULD 平滑衰减到零，但临近目标不要求强制恢复 1.0 标准正方形。

### R-006 局部动态避障

- 主输入 MUST 是每台 TB3 的 `/scan`、`/odom` 和响应组邻车状态。
- 四台扫描 MUST 转到统一坐标系并带时间戳；过期传感器数据不得参与控制。
- 局部控制先生成“跟踪动态槽位”的期望速度，再通过确定性的 time-to-collision/速度可行域过滤，选择不碰撞且最接近期望的速度。
- 搜索机器人和其他动态物体可以由 LiDAR 直接形成短时局部障碍；Gazebo `/model_states` 只用于仿真增强和诊断对照。
- 最终 mux 保留独立的硬碰撞停止距离和命令新鲜度超时，但不再承担常规路径规划或用大幅势场指令覆盖编队控制。
- 任务控制阶段只有与任务目标的预期接触可按任务 profile 放行；不能为整个 BOX 任务关闭所有机器人/墙体安全。

### R-007 地图和无路径行为

- 启用 OccupancyGrid 规划时，规划器在配置的启动期限内未收到合法地图 MUST 抛出错误、进入 ERROR 并保持零速；不得创建中心到目标的直线路径。
- 已收到地图后若地图失效或 frame 不一致，MUST 停止并报告 `MAP_INVALID`。
- 静态地图不存在最小构型路径时，MUST 报告 `NO_STATIC_PATH`，不得做直线恢复。
- 前方仅被动态障碍阻塞时，MUST 进入 `BLOCKED_DYNAMIC`，保持安全构型或零速，并在障碍数据变化或退避计时到期时重规划。
- 重规划退避 SHOULD 有上限，且地图/目标/障碍发生实质变化时立即重试；不允许 10 Hz 连续全图 A*。
- 动态阻塞超时后任务进入 FAILED，但任务管理器必须保留具体失败原因，便于重新派发或人工决策。

### R-008 删除凝聚兜底

- 删除 `formationCohesionLinearScale()`、`cohesionLinearScale()` 和 `formation_cohesion_*` 配置。
- 编队质量由共享中心参考、连续槽位、速度可行域和控制速度上限自然维护。
- 若某车跟踪误差过大，整个编队中心速度按连续质量系数减速；不得只识别“领先车”并令其硬停。
- 质量系数是正常控制律的一部分，不是超过 0.8 m 才触发的单点兜底。

### R-009 任务几何

- ROUNDFLOCKING/Victim/Fire 的导航中心目标 MUST 是被环绕对象中心；名义四槽位应天然均匀分布在目标周围。
- 到达 ROUNDFLOCKING 交接点时允许正方形有小幅形变，但目标必须位于编队凸包内或满足等价的角向覆盖条件。
- TRANSPORT 的导航中心目标 MUST 位于箱体相对推送方向的后方。
- TRANSPORT 的 `path_yaw`/终端 yaw MUST 与推送方向一致，使正方形自然形成前后两列；随后切换现有 transport 控制器完成精确接触和推送。
- RETURN_HOME 使用捕获的初始中心和初始槽位朝向。

### R-010 到达判据

导航到达使用以下组合，而不是单个机器人的精确角点或单一大圆：

1. 中心误差在当前任务 profile 的中心容差内。
2. 编队 RMS 槽位误差和最大槽位误差均在允许范围内。
3. 实测最小车间距、最小障碍距满足安全约束。
4. 条件连续保持一小段时间，排除穿越目标区域的瞬时命中。
5. 任务特定几何成立：环绕任务检查角向覆盖/目标在凸包内，transport 检查箱后纵向与横向排布。

不要求：

- 临近目标恢复名义边长 0.6 m。
- 四车朝向完全一致。
- `deform_reached` 与外部到达屏障同时为真。

### R-011 可观测性和诊断

- 必须发布唯一中心路径、实际中心、参考中心、四个动态槽位、当前纵/横尺度、局部偏移、最小 clearance、规划状态、运动许可和失败原因。
- RViz marker 必须能直接看出“中心是否在路径上”“哪台车因障碍偏槽”“当前是静态无路还是动态阻塞”。
- 关键状态变化使用事件日志；控制周期日志使用 throttle，避免 10 Hz 刷屏。

### R-012 环境无关配置

- demo 与 Tepper 使用同一算法与同一参数语义。
- 环境配置只能包含地图话题、机器人尺寸、名义边长、速度/加速度上限、传感器话题和任务 profile 容差。
- 不允许为某张地图硬编码路径点、比例序列、恢复方向或特定机器人 ID 行为。

## 5. 方案比较

### 方案 A：在现有四规划器上继续修补

改动最小，可以加入 START 门控、clearance 代价和更细缩放。但四份 A*、四份中心进度和分散状态仍在；动态形变与一致到达会继续重复实现。适合短期 demo，不满足 Tepper 泛化目标。

### 方案 B：单一虚拟中心 + 连续仿射槽位 + 局部速度安全投影（推荐）

只规划一次中心路径，四车共享构型。全局层解决静态可达和通道中部，局部层处理临时障碍与小幅非刚性形变，最终 mux 提供硬停止。计算量、实现复杂度和泛化能力最平衡，也最贴合当前 ROS1/TB3 资产。

### 方案 C：分布式非线性 MPC/CBF

能把运动学、碰撞和编队误差统一写成约束，理论表达最完整。但需要在线优化器、可行性恢复、更多调参和真实机算力验证。对当前四台低速 TB3 属于过重方案，不作为本轮主线。

## 6. 目标架构

```text
GUI / command line
   | START / STOP / exploration HOME / response HOME
   v
mission/task state ----------------------------+
   | active_task + behavior_mode               |
   | latched motion_enabled                    |
   v                                           |
single formation_navigation_node               |
   |                                           |
   +-- /sim_map -> clearance field             |
   +-- all odom -> actual center/shape          |
   +-- all scans -> local obstacle snapshot     |
   |                                           |
   +-- clearance-aware center path              |
   +-- continuous affine slots                  |
   +-- deterministic velocity safety projection|
   |                                           |
   +--> /tb_i/deform_cmd_vel                    |
   +--> /response_group/status                  |
   +--> path/slot/diagnostic markers            |
                                               |
task controller -> /tb_i/task_cmd_vel           |
                                               v
                  response_cmd_mux_node
                  mode select + motion gate
                  command timeout + hard safety
                           |
                           +--> /tb_i/cmd_vel
```

以下节点/角色合并到单一导航器：

- `deform_dispatch_bridge_node.py`
- 四个 `plan_manager_node`
- `response_group_coordinator_node.py` 的导航到达部分

以下节点保留：

- `response_goal_sequence_node.py`：demo 任务源，简化为只发布 active task。
- `emergency_task_manager_node.py`：Tepper 任务队列、暂停恢复、手动返航。
- `response_task_controller_node.py`：任务控制。
- `response_cmd_mux_node.py`：唯一最终速度出口与硬门控。
- `response_global_info_node.py`：任务控制器观测适配。

## 7. 状态机

```text
BOOT
  |
  +-- map/odom ready ----------> STOPPED
  +-- map deadline exceeded ---> ERROR_MAP_UNAVAILABLE

STOPPED --START---------------> PLANNING
PLANNING --path found---------> TRACKING
PLANNING --static impossible--> FAILED_NO_STATIC_PATH
TRACKING --STOP---------------> PAUSED
PAUSED --START----------------> TRACKING or TASK_CONTROL
TRACKING --dynamic blockage---> BLOCKED_DYNAMIC
BLOCKED_DYNAMIC --clear/replan> TRACKING
BLOCKED_DYNAMIC --timeout-----> FAILED_DYNAMIC_TIMEOUT
TRACKING --arrival------------> TASK_CONTROL
TASK_CONTROL --task done------> PLANNING(next task)
any non-error --response HOME-> RETURNING_HOME
RETURNING_HOME --STOP---------> PAUSED(home)
RETURNING_HOME --arrival------> COMPLETE_HOME
all tasks done---------------> COMPLETE_WAITING_FOR_HOME
```

状态转换只允许任务管理器和导航器各自在自己的职责内完成。最终 mux 不推断任务，只执行 mode 与 motion gate。

## 8. 任务目标 profile

| Profile | 中心目标 | 终端构型 | 导航到任务控制交接 |
|---|---|---|---|
| `SURROUND` | 对象中心 | 正方形/轻微仿射形变，目标位于内部 | 中心 + 编队质量 + 角向覆盖 |
| `TRANSPORT_APPROACH` | 箱体后方，方向与推送方向相反 | 沿推送方向的两列 | 中心投影 + 横向排布 + 安全间距 |
| `RETURN_HOME` | 启动时捕获的中心 | 捕获朝向下的可观测方形 | 中心 + 编队质量 |
| `WAYPOINT` | 配置航点 | 路径切向方形 | 中心 + 编队质量 |

任务控制器自己的完成判据保持独立：roundflocking 继续检查角度进展；transport 继续检查箱体进入目标区。

## 9. 参数收敛

保留的核心参数：

- `robot_radius`
- `nominal_formation_side`
- `inter_robot_safety_gap`
- `static_clearance_margin`
- `max_linear_velocity` / `max_angular_velocity`
- `max_linear_acceleration` / `max_angular_acceleration`
- `lookahead_distance`
- `clearance_cost_weight`
- `scan_timeout` / `odom_timeout` / `command_timeout`
- `dynamic_block_timeout`
- 每种任务 profile 的中心、质量与保持时间容差

删除的参数：

- `min_formation_scale`
- `formation_scale_step`
- `formation_expand_distance`
- `blocked_recovery_linear_scale`
- `allow_blocked_direct_recovery`
- `blocked_direct_recovery_max_distance`
- 全部 `formation_cohesion_*`
- 常规避障使用的多组势场 blend/替换阈值；仅保留硬安全距离

## 10. 错误处理

| 错误 | 行为 | 对外 detail |
|---|---|---|
| 启动无地图 | 零速，记录一次 fatal，导航节点退出或保持 ERROR | `MAP_UNAVAILABLE` |
| 地图尺寸/data 不匹配 | 零速，不规划 | `MAP_INVALID` |
| frame 不可变换 | 零速，等待短期恢复；超时 FAILED | `TF_UNAVAILABLE` |
| odom/scan 过期 | 对应机器人零速；全组减速/停止 | `SENSOR_STALE:<robot>` |
| 静态无路 | 不直线兜底，任务导航失败 | `NO_STATIC_PATH` |
| 动态阻塞 | 安全停止，按退避周期重规划 | `BLOCKED_DYNAMIC` |
| active task 数据非法 | 拒绝任务，不覆盖当前合法目标 | `INVALID_TASK_GOAL` |
| task controller 指令过期 | mux 输出零速 | `TASK_CMD_STALE` |

## 11. 验收标准

### 11.1 控制状态

- Tepper 启动后、GUI 未 START 的 10 秒内，所有 `/tb_i/cmd_vel` 和探索组 cmd_vel 都无非零指令。
- START 后两组开始；STOP 后一个控制周期内两组输出零速，并持续为零。
- STOP 位于 EN_ROUTE、TASK_CONTROL、RETURNING_HOME 三种状态时，START 都能恢复对应阶段，而不是重新派发或卡在 STOP。
- 两个 HOME 按钮只影响对应组；响应组返航结束后中心回到初始中心容差内。

### 11.2 规划与形变

- demo 五航点完整通过，中心路径只有一个发布者。
- 对同一地图/起终点，四台车观察到的 center path、path index、尺度完全一致。
- 宽走廊路径的平均左右 clearance 差显著小于当前最短 A*；路径不贴膨胀边缘。
- 尺度随 clearance 连续变化，不再出现固定 0.1 比例跳变。
- 局部障碍出现时允许单车偏槽；障碍移除后偏移平滑回落，实际中心仍贴近共享路径。
- 任何时刻最小车间距和最小墙距不低于配置的物理安全约束。

### 11.3 无路与传感器

- 不发布 `/sim_map` 时，导航明确报错且所有响应车保持零速。
- 构造静态封闭目标时返回 `NO_STATIC_PATH`，没有中心到目标直线命令。
- 用静止搜索机器人挡住初始出口时响应组等待；搜索机器人离开后自动重规划通过。
- 用移动障碍横穿通道时，响应组减速/形变/绕行，不碰撞，不依赖 model name。

### 11.4 任务交接

- SURROUND 到达时目标位于四车凸包内或角向覆盖通过，随后 roundflocking 控制器接管。
- TRANSPORT 到达时编队中心在箱后、朝向与推送方向一致、四车形成两列，随后 transport 控制器接管。
- 到达不要求边长恢复 0.6 m，但要求编队可观测、无碰撞且质量指标通过。

## 12. 测试层级

```text
纯函数单元测试
  clearance field / A* / corner cutting / smoothing
  affine scale / zero-mean offsets / slot assignment
  time-to-collision / deterministic candidate selection
  task profile / pause-resume / home transitions
          |
          v
ROS 节点集成测试
  no-map failure / START-STOP gate / stale command / status reason
          |
          v
Gazebo demo 回归
  五航点 / 收放 / 动态障碍 / 手动返航
          |
          v
Tepper 场景验收
  密集初始区 / 多任务 / surround / transport / 双组暂停恢复与返航
```

## 13. 参考依据与取舍

- ROS costmap 的成熟分层模型将静态地图、传感器障碍和 inflation 分开处理，支持用机器人 footprint 生成安全代价。本文采用相同职责拆分，但不为虚拟中心启动完整 move_base：[Nav2 costmap_2d 官方说明](https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/README.md)、[ROS Noetic InflationLayer 源码文档](https://docs.ros.org/en/noetic/api/costmap_2d/html/inflation__layer_8cpp_source.html)。
- Quasi-centralized formation planning 使用虚拟中心的集中路径和障碍附近的个体小幅偏离，和本项目“中心沿路径、单车可偏槽”的需求高度一致：[A Quasi-centralized Collision-free Path Planning Approach for Multi-Robot Systems](https://arxiv.org/abs/2103.10316)。
- ORCA 将每个机器人期望速度投影到成对无碰撞速度集合，并已在差速机器人上验证；本文只采用确定性速度安全投影的轻量思想，不在首轮引入完整外部库：[ORCA 项目与论文入口](https://gamma-web.iacs.umd.edu/ORCA/)。
- 分布式 MPC 能统一处理队形、非完整约束和避障，但其在线优化复杂度超过当前四台 TB3 的必要范围，因此作为未来升级方向而非本轮依赖：[Cooperative distributed nonlinear model predictive control of a formation of differentially-driven mobile robots](https://doi.org/10.1016/j.robot.2021.103993)。

## 14. 实施决策点

本文推荐方案 B。产品级取舍已经确认：手动响应组 HOME 定义为“终止当前响应任务链并返航”，返航可暂停/恢复，返航完成后不继续旧任务。

## 13. 分阶段实施状态

### 13.1 阶段 1：控制正确性与任务目标语义

状态：2026-07-07 已完成实现和完整 Tepper 运行验证。

已落地：

- demo/Tepper 的当前任务源以 latched Bool 独占发布 `/response_group/motion_enabled`，初始值为 false。
- 最终 cmd mux 默认 `STOP`，在命令选择前和所有旧安全层之后执行运动门控；关闭时四台 `/tb_i/cmd_vel` 都为零。
- STOP 保留活动任务阶段；START 按 EN_ROUTE、ACTING、RETURNING_HOME 恢复对应控制源。
- 探索组 HOME 不改变响应组；Tepper 响应组 HOME 已接通，并要求完整运行时 home snapshot。
- 自动返航关闭；手动 HOME 将当前任务终止为 FAILED、保留待执行任务但不再派发，并进入保留任务 `-1000`。
- SURROUND 的导航中心等于对象中心；TRANSPORT 的导航中心在箱体后方 `transport_rear_distance`，终端 yaw 与推送方向一致。
- 为运行完整系统，补齐 `graph_2d_grid.hpp` 和 `graph_3d_grid.hpp` 对 `boost/functional/hash.hpp` 的定义依赖，消除既有链接失败。

Tepper 实测确认 START 前 gate=false、mode=STOP、四车速度为零；STOP 后四车在控制周期内归零；START 恢复同一 task/status；两个 HOME 命令只影响各自组；响应 HOME 使用实测中心 `(4.00, 16.40)`。

阶段 1 未解决、且不得误判为已解决：旧四规划器返航过程中中心距离长期约 `10.7–10.8 m`、最大槽位误差约 `10.9–11.0 m`，未在观察窗口内收敛。这验证了阶段 2 的共享净空中心规划、连续形变和局部安全替换仍是必要工作。

### 13.2 阶段 2：共享导航核心和单主链

状态：2026-07-11 已完成。

- 运行时只保留一个 `formation_navigation_node`，中心 A*、路径进度、连续纵/横形变、四槽位、速度安全和 profile 到达判据均由模块化 C++ 核心提供。
- 静态地图缺失、目标碰撞和静态无路明确失败；不存在直线路径降级。
- 删除旧四规划器、dispatch bridge、Python coordinator 及对应冗余配置/消息；后续调整只修改单一主链。
- 差速机器人只原地转向后正向行驶；编队质量连续调速，不再使用领先车硬停或临近目标恢复。

### 13.3 阶段 3：任务控制与双组协同

状态：2026-07-12 已完成。

- transport 使用同步 ALIGN/PUSH 和由箱体/URDF 派生的物理槽位。横向规划宽度与纵向接触长度解耦；PUSH 中前排接触箱体、后排接触前排，四车共同传力。
- roundflocking 的半径按局部空间自适应，可缩小；每台车必须累计 `4π` 角进度，四车全部满足才 DONE。
- 双组避障采用确定性非对称优先，探索组主动让响应组；最终安全仍禁止跨组接触。
- 探索和响应的最终命令所有者均执行默认关闭的 mission gate。STOP 清理路径并拒绝迟到规划，START 幂等，两个 HOME 各自闭环。
- 普通导航/环绕的中心距边界统一为 `2*robot_radius + inter_robot_safety_gap`；BOX/PUSH 只对响应成员开放按 URDF 长度派生的受控接触。

### 13.4 阶段 4：长跑验收

状态：2026-07-13 已完成。

长跑迭代遵循“拒绝结果也必须落到可复现根因”的规则：

- 首轮 7 任务运行因跨组最小中心距 `0.199 m` 被拒绝；加大物理制动边界后，后续又在 roundflocking 的 `TASK_CONTROL` 交接处复现 `0.168 m`。根因是启发式 LiDAR 中点抽样会漏掉窄机器人，且最终速度层缺少移动外部机器人的预测反应余量。现使用扇区最小值并在任务模式按距离余量限速；不生成倒车，不影响 transport 内部受控接触。
- `final7` 已再次完成 7 个任务：两次 transport 形成约 `0.134--0.139 m` 的前后实体链，五次环绕四车均超过 `4π`，跨组最小中心距 `0.322 m`，无负线速度。
- `final7` 的剩余拒绝项是 transport 终态 HOME 起点判定。实测最小静态净空 `0.114644 m` 大于 `0.11 m` 实体半径、但小于 `0.15 m` 舒适间隙；因此启动合法性改为只判断实体 footprint，正常路径与速度安全仍保留舒适间隙。同一终态定向回归已在 `69.5 s` 内返航 DONE。
- `final8` 进一步暴露了任务顺序相关的离散切线问题：transport 接触队形切到一条首段急转的远路径后，槽位朝向瞬时旋转，前后车目标互相遮挡；接近零的安全速度还会让差速适配在候选方向和终端 yaw 间切换。现保持一次任务内槽位身份稳定，编队 yaw 的角速度由 `中心速度 / 编队半径` 自动限制并跨 ±π 取最短角，小速度死区统一使用编队 yaw。复用同一终态/目标的 Gazebo 对照已从旧实现 600 秒滞留改为目标距离从 `7.839 m` 单调降至 `0.359 m`，最终以 `0.222/0.173 m` 最大/RMS 槽位误差 ARRIVED。
- 后续干净候选又关闭了两项泛化问题。外部探索车的预测轨迹由全局静态硬墙改为中心 A* 软代价，局部 TTC/实体层负责实时安全；探索组不再在近距内与响应组对称停车，而是作为低优先级方持续执行可实现的分离速度。START/STOP/HOME 的最终门控使用原子状态和独立零速发布，不再被地图规划互斥锁延迟。
- roundflocking 已从四个彼此独立的启发式动作改为组级循环追踪：按目标极角排序并按前方角隙连续调速，保留自适应半径与 LiDAR 避障，避免成员争抢同一角位。该实现不增加环境路径点或固定避障动作。
- 最终隔离运行完成 `7/7 DONE + response HOME + exploration HOME + final STOP`。两次 transport 的前后实体链为 `0.132--0.135 m`，五次环绕四车均超过 `4π`；跨组最小中心距 `0.257 m`，START 前最大响应指令 `0.0`。最终 STOP 后九车 10 秒最大命令 `0.0`、最大漂移 `0.000075 m`。完整数字、运行 ID 和失败候选根因见实施日志。
- 最终回归为 Python `83/83`、planner `21/21`、formation `17/17`、navigation `8/8`、velocity safety `11/11`、exploration RVO `11/11`，完整工作区构建通过。
