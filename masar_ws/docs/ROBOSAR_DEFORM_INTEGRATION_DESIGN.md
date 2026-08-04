# RoboSAR 响应组可变编队集成说明

本文描述 `masar_ws` 当前唯一的响应组运行链。早期的四套 `PlanManager`、目标转发桥、独立到达协调器、离散缩放枚举和直线路径兜底均已退出运行链。2026-07-13 的阶段 4 基线完成过全部 Tepper 长跑；阶段 7 后续改动已通过定向测试，但最新完整 Tepper 闭环仍待复验，不能把统一平台“完整搜救场景可启动和观测”解释为全部算法任务已经再次完成。

## 1. 当前状态与实现边界

响应组主链重构与阶段 1-6 已完成：

1. 任务与 GUI 启停/返航语义统一。
2. 编队中心规划、连续形变和到达判据重构。
3. transport、roundflocking 与局部安全控制器重构。
4. Tepper 全任务长跑、跨组解锁、双组返航和最终静止验收。
5. 动态交通释放后的路线再求优、独立 demo 与生产控制器同步、分配图端点语义收口。
6. 标准 ROS 主站预清理与 ROS1/ROS2 桥接端口契约收口。

阶段 7 的探索分配、跨组让行、GUI 和 transport 对位改动已完成定向回归；最新完整
Tepper 运行仍存在进度日志记录的跨组让行停滞，最终全任务闭环状态以
`RESPONSE_FORMATION_REFACTOR_PROGRESS.md` 为准。

当前实现边界：

- 运行平台为 ROS1 Noetic、Gazebo 和可选 RViz。
- 搜索组为 `agent_0 ... agent_4`，响应组为 `tb_0 ... tb_3`。
- 响应导航按四车编队实现，但几何、安全距离和运动限制来自配置；更换机器人模型时必须同步标定这些物理量。
- `src/robosar_deform_vendor` 保留 DEFORM 相关 vendor 包，其中 `plan_manager/formation_navigation_node` 是当前响应编队导航器。
- ROS2/Webots 学习策略不属于本文件的运行闭环；Tepper 生产配置使用可解释的同步 transport 与协调式 roundflocking 控制器。

## 2. 唯一执行链

### 2.1 搜索发现到任务完成

```text
搜索组发现 victim / fire / box
-> emergency_task_manager_node.py 建立和排序 EmergencyTask
-> /response_group/active_task
-> formation_navigation_node 规划编队中心路径和连续槽位
-> /tb_i/deform_cmd_vel
-> response_cmd_mux_node.py 选择并限速
-> /tb_i/cmd_vel
-> 编队导航到任务 staging pose 后发布导航 DONE
-> emergency_task_manager_node.py 将模式切为 TASK_CONTROL
-> response_task_controller_node.py 执行 surround 或 transport
-> /tb_i/task_cmd_vel
-> response_cmd_mux_node.py 执行任务态安全规则
-> /response_group/task_result = DONE / FAILED
-> 继续下一个任务；全部完成后等待显式响应组 HOME
```

系统中只有一个中心规划器、一个编队形变状态和一个最终速度所有者。不存在并行保留的旧目标桥、四车独立全局规划器或独立到达协调器。

### 2.2 控制权与速度所有权

`/response_group/behavior_mode` 的主要状态为：

- `PAUSED`：输出零速度。
- `NAV_FORMATION`：选择 `/tb_i/deform_cmd_vel`。
- `TASK_CONTROL`：选择 `/tb_i/task_cmd_vel`。

`response_cmd_mux_node.py` 是响应组 `/tb_i/cmd_vel` 的唯一发布者。它同时订阅任务、模式、运动门、里程计和激光数据，以保证模式切换原子化且旧指令不会穿透。

## 3. GUI 启停与返航语义

全局命令发布在 `/system_mission_command`：

- `START`：同时开启搜索组与响应组运动门；暂停后再次 START 可恢复当前任务。
- `STOP`：同时关闭两组运动门并输出零速度，但不丢失任务进度。
- `EXPLORATION_HOME`：搜索组返回启动时捕获的各自初始位姿。

响应组返航使用 `/response_group/return_home`。返航目标由运行开始时四台响应机器人位姿计算得到，而不是依赖地图专用常量。所有应急任务完成后系统进入 `COMPLETE_WAITING_FOR_HOME`，只有显式 HOME 命令才建立返航任务；到家后进入 `HOME_COMPLETE` 并停止。

运动门默认关闭且使用锁存状态。节点启动、订阅重连或模式切换期间，未收到 START 时 `/cmd_vel` 必须保持为零。

## 4. 编队中心规划与连续形变

### 4.1 地图和失败语义

`formation_navigation_node` 必须收到配置的栅格地图；Tepper 使用 `/sim_map`。地图缺失会报告 `MAP_UNAVAILABLE`，静态地图不存在通路会报告 `NO_STATIC_PATH`，不会退化为中心到目标的直线。

中心路径由带净空代价的 A* 生成：

- 静态墙体、不可通行区域和紧急对象实体占位是硬约束。
- 搜索组机器人等可移动外部对象进入短时预测软代价，使规划倾向绕开拥挤区域，但不把瞬时人群永久封成静态墙。
- 外部机器人相对规划快照发生超过地图分辨率/编队包络派生阈值的位移时，中心路径重新求优；因此拥堵形成的临时绕路会在通道释放后缩短，而不只在新障碍撞入旧路径时更新。
- 编队足迹和槽位可行性参与路径评价，因此中心线优先位于通道中部，而不是只保证中心点能够通过。
- 起点或局部通道暂时被移动机器人占据时持续重规划；不会用错误的直线兜底穿越障碍。

### 4.2 槽位与形变

四个槽位以中心路径切向为朝向基准，并保持稳定的机器人—槽位身份。标称队形为正方形，但运行时允许：

- 根据可用净空连续缩放，而不是枚举固定比例列表。
- 对单个槽位施加有界横向/纵向偏移，绕过局部动态障碍后平滑恢复。
- 对尺度、偏移和朝向限制变化率，避免槽位跳变和机器人突然掉头。
- 只发布非负线速度；需要改变方向时先转向再前进，不以倒车追赶槽位。

规划参数集中在 `config/formation_navigation.yaml`。参数表达机器人几何、物理安全余量、速度/加速度上限和通用控制尺度，不包含针对 Tepper 某条走廊的路线坐标或离散缩放试探表。

## 5. 跨组交通与局部安全

响应导航器和两组局部控制器综合使用里程计、TF 与激光观测进行短时碰撞预测。跨组冲突采用非对称优先级：

- 正在执行应急导航或子任务的响应组拥有通行优先权，但仍对墙体和不可让行障碍执行硬安全停止。
- 搜索组不再在接触距离触发与响应组对称的永久等待，而是持续计算平滑分离速度，为响应组让出通道。
- 响应组若当前确实没有安全可行速度会短暂停止并重规划，不通过互相穿透解决冲突。

这一规则解决初始密集出生和狭窄处双方同时停车的死锁，同时保留普通导航和环绕期间的跨组无碰撞约束。搜索组当前从 Gazebo 模型状态获得响应组动态位姿；迁移到真机时应替换成同等语义的共享定位/跟踪话题，决策接口无需改变。

探索目标分配继续使用 `TA_HIGH_OA + Hungarian` 主链。车队候选集合来自已经聚类过滤的全部 frontier 区域；每选中一个候选就立即扣减与其观测范围重叠的任务效用，再选择下一候选，最后统一做最小路程匹配。这样同起点机器人不会因为各自只看到相同的五个近点而被迫同向扎堆；普通 `TA_HIGH` 的局部候选逻辑保持不变。

## 6. 子任务控制器

### 6.1 Roundflocking / surround

编队导航将中心送到受害者或火源目标，随后切换到协调式环绕控制器：

- 四台车按稳定相位均匀分布在目标周围。
- 期望半径由局部障碍净空连续调整；空间不足时可缩小，离开约束后平滑恢复。
- 机器人共享同一绕行方向和相位推进，不允许单车原地卡住而由其他成员替代完成。
- 每台车都必须围绕目标累计至少 `4π` 的有效角度进度，且半径处于有效范围，任务才发布 DONE。

到达子任务时不要求先恢复几何上绝对规整的正方形；要求队伍仍可观测、运动连续且不发生碰撞。

### 6.2 Transport

BOX 任务的导航 staging pose 位于箱体后方，即推送方向的反方向。切换 `TASK_CONTROL` 后由同步物理槽位控制器执行：

1. `ALIGN`：前排在箱体后表面外保留一个 `robot_gap`，后排与前排同样保留该净空；机器人在无实体接触时完成位置和推送朝向对准。
2. 首次位置就绪仍采用由槽位自由余量派生的严格容差；转向引起的小漂移只要仍在物理净空内就保持就绪，避免厘米级误差使平移/转向反复切换。全队短时稳定后进入 `PUSH`。
3. 前排与箱体直接接触，后排与前排直接接触，四台车使用共同推送节奏形成实体力链。
4. 箱体中心连续保持在目标阈值内后发布 DONE，由任务管理器恢复普通编队导航。
5. 超时不伪装为成功。

transport 是唯一允许响应组成员受控接触的状态。安全层在 `BOX + TASK_CONTROL` 中仅按预期接触几何放行前后排接触；机器人与墙体、搜索组、非目标对象之间始终保留硬安全约束。这样“全员合力”通过真实接触实现，而不是仅用虚拟速度耦合冒充贡献。当前主链不包含推送完成后的额外脱离状态。

## 7. 到达、恢复与失败处理

导航到达采用任务类型对应的组合判据：

- 中心到目标距离满足阈值。
- 四车槽位 RMS 和最大误差在允许范围内。
- 条件连续保持指定时间。
- 当前地图、任务、运动门和模式仍属于同一有效代次。

transport 导航只负责把队伍连贯地送到箱后 staging 区，实体接触由任务控制器判定；surround 的完成由四车实际角度进度判定。系统不使用“临近目标强制恢复标准正方形”“领先车误差过大就永久等待”或“无路时走直线”等兜底。

可恢复失败采用有界重规划和状态代次控制：地图或动态障碍更新后重试中心路径；旧任务、旧计时器和旧速度指令不能完成新任务。不可恢复的地图缺失、静态无路或任务超时会明确发布失败原因，便于上层处理。

## 8. 当前节点、话题和文件

### 8.1 核心节点

```text
emergency_task_manager_node.py       任务队列、模式、运动门、HOME 和结果推进
formation_navigation_node            中心 A*、净空规划、连续形变、导航到达
response_global_info_node.py          汇总任务控制器所需观测
response_task_controller_node.py      coordinated surround / synchronized transport
response_cmd_mux_node.py              模式选择、任务态安全、最终 cmd_vel
lazy_traffic_controller               搜索组路径跟踪、跨组连续让行
```

### 8.2 关键话题

```text
/system_mission_command
/response_group/return_home
/response_group/motion_enabled
/response_group/behavior_mode
/response_group/active_task
/response_group/status
/response_group/task_result
/response_group/center_path
/response_group/formation_slots
/global_info
/tb_i/odom
/tb_i/scan
/tb_i/deform_cmd_vel
/tb_i/task_cmd_vel
/tb_i/cmd_vel
```

### 8.3 入口与配置

```text
run_response_deform_demo.sh
run_tepper_response_demo.sh
run_response_controller_demo.sh

src/robosar/robosar_emergency_response/launch/response_deform_demo.launch
src/robosar/robosar_emergency_response/launch/response_group_deform.launch
src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch
src/robosar/robosar_emergency_response/config/formation_navigation.yaml
src/robosar/robosar_emergency_response/config/tepper_response_group.yaml
src/robosar/robosar_emergency_response/config/response_safety.yaml
```

控制器独立 demo 复用生产控制器和最终 mux，但由本地 fixture 直接打开运动门，且不配置该 world 中不存在的探索机器人。普通入口 `roundflocking` 使用 `heuristic`，`transport` 使用 `synchronized_slots`；带显式后缀的入口仅用于控制器对照。

## 9. 启动和观测

小型五航点验证：

```bash
./run_response_deform_demo.sh
```

Tepper 完整系统：

```bash
./run_tepper_response_demo.sh
```

完整入口默认接入标准 ROS/Gazebo master（未覆盖时为 `11311/11345`）。在启动业务节点前，入口先枚举 ROS 图中除 `/rosout` 外的旧节点，通过 `rosnode kill` 请求其退出，清理失效注册与旧参数，并验证 `/clock` 尚未被占用；仍有节点受外部 supervisor 拉起或拒绝退出时直接报出剩余节点并终止。该方式不依赖仿真器或机器人进程名，既能消除上一次运行遗留状态，也保持 `/root/workspace/ROS2swarm_w` 中 `ros1_bridge` 对标准 `11311` 的连接契约。退出时仍只清理本次入口记录的进程组。

默认清洁模式是完整系统入口，应先于外部桥启动。已有 ROS 图确实由外部系统管理时使用 `--shared-master`，此模式只检查而不清理，且已有 `/clock` 发布者时拒绝启动。完整 Tepper 入口删除独立端口分支，只维护标准主站这一条生产链；独立小型 demo 仍可使用自己的主站，因为它们不承担跨项目桥接。从第二个终端观测默认系统时使用 `ROS_MASTER_URI=http://127.0.0.1:11311`，Gazebo CLI 使用 `GAZEBO_MASTER_URI=http://127.0.0.1:11345`。

无图形或仅核对启动参数：

```bash
./run_response_deform_demo.sh --headless
./run_tepper_response_demo.sh --headless --no-rviz
./run_tepper_response_demo.sh --dry-run
```

排障时优先检查：

1. 未 START：`/response_group/motion_enabled=false` 且两组最终 `/cmd_vel` 为零。
2. 导航不动：检查 `/sim_map`、`/response_group/status`、`/response_group/center_path`、`/tb_i/deform_cmd_vel` 和最终 `/tb_i/cmd_vel`。
3. 子任务不推进：检查 `/response_group/behavior_mode`、`/global_info`、`/tb_i/task_cmd_vel` 和 `/response_group/task_result`。
4. transport：确认状态依次进入 `ALIGN/PUSH`，推送时形成实体接触链，箱体在目标阈值内连续保持后发布 DONE。
5. roundflocking：DONE 只由所需成员的目标相对累计角进度产生；任务时限作为“未完成成员无新增高水位角进度”的停滞窗口，而不是绝对截止时间，避免安全避障导致的慢环绕被误判失败。
5. surround：确认四台车角度进度分别增长到至少 `4π`，而不是只看任务持续时间。
6. HOME：响应组任务 ID 应切换为 `-1000`；搜索组状态应最终全部为 `SUCCEEDED`。

## 10. Tepper 阶段 4 长跑基线

2026-07-13 的隔离 ROS/Gazebo 长跑完成全部闭环：

- GUI START 前最大速度指令：`0.0`。
- 任务 `1 ... 7` 全部 DONE，失败任务为空。
- 两个 transport 均进入同步物理 PUSH；观测到前排接触箱体、后排接触前排。
- 五个 surround 均由四台车分别完成至少 `4π`，不是超时或单车代完成。
- 跨组碰撞监测通过；全程最小跨组中心距为 `0.257 m`。
- 响应组显式 HOME 完成，随后搜索组五车 HOME 状态全部为 `SUCCEEDED`。
- 最终 STOP 后连续 10 秒，九台机器人线/角速度最大值均为 `0`，最大物理漂移为 `0.000075 m`。
- 总运行时间约 `1091.4 s`。

最终长跑时的回归基线为 Python `83` 项、规划/动态障碍 C++ `21` 项。2026-07-14 收口并加入启动主站回归后为响应组 Python `90` 项、探索联合分配 `1` 项、规划/动态障碍 C++ `22` 项；编队/到达 `17` 项、导航/transport staging `8` 项、响应安全 `11` 项和搜索组 RVO `11` 项保持通过；相关包构建通过。该日新增 transport 定向 demo 已再次完成 `ALIGN -> PUSH -> DONE`；完整 Tepper 最新定向轮仍被既有跨组让行停滞阻断，不能替代 2026-07-13 的最终长跑闭环证据，细节见进度日志。

## 11. 维护原则与限制

- 只维护本文所述单一主链。与需求冲突或已被替代的运行实现应删除，不保留“双轨兜底”。
- 算法模块按中心规划、槽位形变、任务状态、任务控制和最终安全解耦；环境差异通过地图、观测接口和物理配置表达。
- 不用场景坐标、逐档比例枚举或大量应急阈值掩盖规划/控制问题。
- 当前跨组仿真观测依赖 Gazebo；真机迁移需要接入共享定位和目标跟踪。
- 更换底盘、箱体或传感器时必须重新测量碰撞几何、扫描自滤半径和动力学限制，但不需要改任务状态机或规划算法。
- checkpoint 控制器仍可用于独立实验；Tepper 已验收主链不依赖其泛化能力。

## 12. 关联文档

- `docs/RESPONSE_FORMATION_REQUIREMENTS.md`：稳定需求、泛化约束和验收条件。
- `docs/RESPONSE_FORMATION_REFACTOR_PLAN.md`：分阶段落地步骤与退出条件。
- `docs/RESPONSE_FORMATION_REFACTOR_PROGRESS.md`：每阶段开始前目标、实际改动、运行结果和问题收敛记录。
- `docs/superpowers/specs/2026-07-07-response-formation-navigation-design.md`：重构开始时冻结的完整需求分析。
- `docs/superpowers/plans/2026-07-07-response-formation-navigation.md`：原始 12 项完整实施计划和历史执行账本。
