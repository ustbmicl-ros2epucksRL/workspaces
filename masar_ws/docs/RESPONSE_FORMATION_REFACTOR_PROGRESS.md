# 响应组编队导航重构实施日志

本文档记录 `Response Formation Navigation` 重构的实际执行状态、验证证据和遗留问题，作为后续阶段继续工作的唯一进度入口。

- 需求与架构基线：`docs/RESPONSE_FORMATION_REQUIREMENTS.md`
- 完整实施计划：`docs/RESPONSE_FORMATION_REFACTOR_PLAN.md`
- 执行原则：逐阶段修改；每阶段先写测试，再实现；完成后运行 Tepper 大系统并记录可复现证据。
- 单主链原则：运行时只保留一条权威响应组链路。新实现一旦通过模块测试并具备完整替代能力，同阶段完成 launch 切换并删除冲突或冗余的响应专用旧实现，不长期保留双规划器、双状态机或双到达判据。
- 模块边界原则：地图净空、中心规划、编队几何、速度安全、导航状态和 ROS I/O 分离；核心算法不得依赖 Gazebo 模型名或具体地图路径点，环境差异只通过通用参数和任务 profile 表达。
- 工作树说明：当前工程包含此前尚未提交的响应组、仿真器和 GUI 改动。为保留这些实现现场，本轮在现有工作树中做显式小步补丁，不提交、不重置、不改写无关文件。

## 总体状态

| 阶段 | 范围 | 状态 | 系统级验证 |
|---|---|---|---|
| 阶段 1 | 运动门控、暂停恢复、手动返航、任务目标几何 | 已完成 | 已执行；控制链通过，旧导航未收敛 |
| 阶段 2 | 新导航核心、单一节点、质量到达判据、运行链切换及旧链删除 | 已完成 | demo 与 Tepper 已执行 |
| 阶段 3 | 任务控制、最终安全边界、双组协同与 GUI 命令闭环 | 已完成 | 7 任务运行已执行；发现项已修复 |
| 阶段 4 | 全任务长时回归、文档和配置收口 | 已完成 | 2026-07-13 干净 Tepper 整链验收通过 |
| 阶段 5 | 动态路线求优、独立 demo 同步、分配图语义收口 | 已完成 | 定向复现、模块回归及两类控制器 demo 实跑通过 |
| 阶段 6 | 标准主站清洁启动与桥接端口契约 | 已完成 | 旧 `/clock` 实际清理、标准端口启动和退出回收通过 |
| 阶段 7 | 探索效率恢复、GUI 与仓库清理；transport RELEASE 试验回滚 | 进行中 | 回滚后 `89 passed`，等待完整 Tepper 长跑 |

## 阶段 1：控制正确性与任务目标语义

### 开始前

开始时间：2026-07-07。

本阶段解决四个直接影响系统正确性的问题：

1. 响应组在 GUI START 前存在速度输出风险，最终 `cmd_vel` 必须由默认关闭的运动许可硬门控。
2. STOP 只暂停，START 必须恢复 EN_ROUTE、ACTING 或 HOME 的原阶段；探索组 HOME 不得停止响应组。
3. Tepper 响应组 HOME 按钮必须终止当前响应任务链并使用完整运行时初始位姿返航，禁止自动返航。
4. 环绕任务的编队中心直接落在目标物位置；运输任务的中心落在箱体后方并朝向推送方向。

阶段基线：

- 相关 Python 测试：`31 passed in 0.20s`。
- 已确认 Tepper 任务管理器未订阅 `/response_group/return_home`。
- 已确认响应命令复用器未消费统一运动许可，默认模式配置存在非 STOP 值。
- 已确认旧目标几何为 victim/fire/box 三套固定 staging distance，而不是按任务语义生成中心目标。

### 完成内容

完成时间：2026-07-07。

1. 新增统一运动许可 `/response_group/motion_enabled`。demo 由 goal sequencer 发布，Tepper 由 emergency task manager 发布，均为 latched、默认 false。
2. `response_cmd_mux_node.py` 默认从 STOP 启动；纯命令选择器首先检查 gate；旧避障/安全层处理完成后再次执行最终零速不变量，避免安全层从零指令合成运动。
3. STOP 仅冻结响应组；START 按任务状态恢复：EN_ROUTE→NAV_FORMATION、ACTING→TASK_CONTROL、HOME→NAV_FORMATION。
4. 探索组 HOME 在响应任务源中显式忽略。Tepper 新增 `/response_group/return_home` 订阅，手动 HOME 要求四车首帧 odom 完整，终止当前任务为 FAILED，并进入终端返航链。
5. `auto_return_home` 改为 false。任务完成后等待手动响应组 HOME。
6. Tepper 启动顺序改为 response overlay 先于 GUI，并等待 latched gate=false 后才暴露 GUI。
7. 删除 victim/fire/box 三套固定 staging distance。SURROUND 以对象为编队中心；TRANSPORT 使用箱体后方中心和推送方向 yaw，只保留 `transport_rear_distance`。
8. 为完整系统构建补齐 `robosar_navigation` 两个 graph 头文件缺失的 `boost/functional/hash.hpp`，修复 `boost::hash_combine<int>` 未解析链接符号。

### 验证结果

静态与构建验证：

- 应急响应包最终回归：`46 passed in 0.25s`。
- launch XML、两个运行脚本 Shell 语法、Python 编译：通过。
- `catkin_make --pkg robosar_emergency_response`：通过。
- 完整 `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo`：通过。首次运行暴露上述 Boost include 缺失；最小修复后同一构建命令通过。
- `git diff --check`：通过。

完整 Tepper 无界面运行：

- 启动命令：`run_tepper_response_demo.sh --headless --no-vglrun`。
- 观察窗口约 9 分钟 wall time、约 540 秒 simulation time；完整启动 Gazebo、探索组、SLAM、任务生成/分配和 response overlay。
- START 前：gate=false、mode=STOP，四台最终速度连续为 `(0, 0)`；3 秒内模型位置漂移小于约 0.003 m。
- START 后：gate=true、mode=NAV_FORMATION，派发 box 任务 1；四车产生非零速度并从初始区域向任务点移动。
- TRANSPORT 几何：box=`(5.895, 14.900)`、goal=`(5.695, 16.365)`，生成中心=`(6.030, 13.909)`，符合沿推送方向反向后撤 1 m，yaw 同推送方向。
- STOP 后：gate=false、mode=STOP，四车最终速度全部 `(0, 0)`，活动任务仍为 `task_id=1,status=EN_ROUTE`，5 秒位置仅有物理级微漂移。
- 再 START：恢复同一 `task_id=1,status=EN_ROUTE` 和 NAV_FORMATION，没有重复派发或丢阶段。
- 探索组 HOME：响应组 gate、mode、task 均不变。
- 响应组 HOME：任务 1 变为 FAILED，待执行任务保持 NEW；活动任务变为 `-1000`，目标使用运行时捕获中心 `(3.999999, 16.399998)`。
- HOME 中 STOP/START：分别得到 gate=false/STOP 和 gate=true/NAV_FORMATION，活动任务始终为 `-1000`。

阶段痛点结论：

- 用户问题 1（START 前运动、STOP/START、分组 HOME）已在完整系统中解决。
- 用户问题 8 的任务导航目标语义已落地：TRANSPORT 实测正确；SURROUND 有纯函数测试保证对象中心目标，实际环绕/控制器交接留待阶段 3 全链验证。
- 用户问题 2/3/4/5/7/9 属于导航核心和到达判据，本阶段未解决。返航运行中旧四规划器的 `center_dist` 连续约 `10.7–10.8 m`、`max_slot_error` 约 `10.9–11.0 m`，未在观察窗口内完成返航；这不是门控或 HOME 目标错误，而是下一阶段要替换的共享路径/形变/局部安全问题。
- 运行时 ROS 日志目录报告超过 1 GB；未擅自清理用户日志，后续长时回归前应由使用者确认后执行日志归档或 `rosclean`。

### 遗留问题与下一步

下一阶段进入共享导航核心：先实现可测试的净空图和单条编队中心 A*，再实现连续纵/横形变与确定性局部速度安全。阶段 2 的量化目标是让同一返航场景的中心距离持续下降，并把槽位误差从当前约 11 m 的失控状态恢复到可观测编队范围内；在中心规划核心通过前不删除旧运行链。

## 阶段 2：共享导航核心与单主链切换

### 开始前

开始时间：2026-07-07。

本阶段对应实施计划 Tasks 4–9。原计划曾把纯算法核心与运行接管拆开；根据“项目只保留一条清晰主逻辑”的补充原则，本阶段调整为一次完整替换：

1. 用距离场和净空代价生成唯一编队中心路径，缺地图或静态无路明确失败，不生成直线兜底。
2. 用纵/横独立的连续仿射尺度和零均值局部偏移替代比例枚举、临近目标恢复和领先车硬停。
3. 用确定性的速度候选投影处理队友、LiDAR 障碍和动态阻塞，算法输入不依赖 Gazebo `/model_states`。
4. 新建一个组级 `formation_navigation_node`，独占中心路径、路径进度、四个动态槽位和四路 `/tb_i/deform_cmd_vel`。
5. 到达判断统一为中心误差、编队质量、安全性、任务 profile 几何和保持时间。
6. 模块测试通过后，同阶段将 demo/Tepper launch 切到新节点，删除响应专用的四规划器 include、dispatch bridge、Python arrival coordinator 及其冗余模块和参数。

阶段基线：

- Python 回归：`46 passed in 0.24s`。
- 现有 formation assignment 测试可执行文件退出码为 0。
- 阶段 1 Tepper 返航基线：`center_dist≈10.7–10.8 m`、`max_slot_error≈10.9–11.0 m`，旧四规划器未在观察窗口内收敛。
- 当前工作树为包含既有未提交实现的主现场；本阶段继续原地小步修改，不创建会丢失现场内容的隔离 worktree。

### 完成内容

完成时间：2026-07-11。

1. 新增独立的 `ClearanceGrid`、`FormationPlanner`、`FormationControl`、`VelocitySafety`、`FormationNavigationState` 和确定性槽位分配模块；ROS I/O 只保留一个 `formation_navigation_node`。
2. 中心 A* 使用静态地图净空和通道中心代价；缺地图、目标碰撞、静态无路均输出明确失败，不再生成中心到目标的直线兜底。
3. 编队以中心路径切线建立槽位，纵/横尺度连续变化并允许零均值局部偏移；删除比例枚举、临近目标强制恢复、领先车停车等待等旧兜底。
4. 局部速度层融合 odom、scan、响应队友和外部动态机器人，使用确定性候选速度投影；差速适配只允许原地转向后正向行驶，不再生成倒车命令。
5. 到达判据统一为 profile 化的中心误差、RMS/最大槽位误差、任务几何、安全性和保持时间；SURROUND 检查目标位于队形凸包内，TRANSPORT 检查箱后两列几何。
6. launch 已切换为单组级导航器；删除旧四规划器 include、dispatch bridge、Python coordinator 和对应冗余配置/消息产物，运行时不保留双链。

### 验证结果

模块验证覆盖宽通道居中、窄通道拒绝、角点不可切割、平滑净空、连续形变、局部避障、暂停恢复、任务 profile 到达和 transport approach。小型 demo 已依次完成 5 个航点并完成 HOME；Tepper 已证明同一中心规划/形变实现可复用，未引入环境路径点。

## 阶段 3：任务控制、双组协同与命令闭环

### 开始前

本阶段针对长跑暴露的三类运行时问题：transport 后排未实体接触、roundflocking 成员卡住/未完成两圈、响应组与探索组在狭窄处对称让行；同时补齐 GUI STOP、探索 HOME、响应 HOME 的最终消费者语义。

### 完成内容

完成时间：2026-07-12。

1. transport 控制器使用同步 `ALIGN -> PUSH`；按机器人实际位置做不交叉槽位分配，前排先就位、后排再就位，进入 PUSH 后四车同步输出。
2. 将 TB3 横向规划宽度 `0.22 m` 与 Burger URDF 纵向碰撞长度 `0.13 m` 解耦。箱面接触由 `box_length/2 + robot_length/2` 派生；PUSH 前后排中心距为 `robot_length`，ALIGN 距离为 `robot_length + robot_gap`。最终 mux 只在 BOX/PUSH 对响应组成员放行几何派生的受控接触，探索机器人不放行。
3. roundflocking 使用按局部可见障碍/目标空间选择的自适应半径，允许临时缩小；完成要求四台车分别累计至少 `4π` 角进度，不再用临近目标恢复或单纯超时判 DONE。
4. 双组交通改为非对称优先：探索 RVO 主动让响应组，响应组仍做 TTC/实体安全但不触发同样的全径向急退，消除对称停止死锁。
5. 探索交通控制器新增默认关闭的最终运动门。START/HOME 打开，STOP 关闭、清路径并在每个发布周期强制零速；门关闭时拒绝迟到规划结果，解决上层停止后旧路径/RVO 继续合成速度的问题。
6. `mission_commander` 的 START 改为幂等事件，STOP/HOME 会清除待处理 START；探索 HOME 不再被陈旧 START 重新拉起。GUI 输出订阅连接数和命令日志，便于区分按钮送达与消费端失败。
7. 跨组和普通编队任务的最终中心距边界统一为 `2*robot_radius + inter_robot_safety_gap = 0.26 m`；transport 响应组内部接触仍使用独立的物理长度链。

### 系统反馈与修正

2026-07-12 首轮候选长跑完成全部 7 个任务，但被验收器拒绝，因此未作为最终通过：

- 两次 transport 均形成实体接触链。代表性证据：箱体到前排中心 `0.307/0.311 m`，两条前后链 `0.134/0.136 m`，四车同步线速度约 `0.070--0.072 m/s`。
- 五个 roundflocking 均由四车两圈证据完成；例如 task 3 为 `12.584/12.857/13.423/13.122 rad`，其余任务同样全部超过 `4π`。
- 全程未采集到负线速度，倒车逻辑已删除。
- 首次跨组最近距离为 `0.199 m`，低于验收线 1 mm；探索 HOME 路径与响应任务并行时最低到 `0.157 m`。根因是最终硬安全只在 `0.22 m` 才截断，未保留离散控制制动余量；已统一到完整的物理安全边界。
- 7 DONE 后响应 HOME 曾以 `START_IN_COLLISION` 失败。该轮属于中心保守包络不满足但四车实体 footprint 可安全退出的情况，因而先加入了逐车 footprint 检查；后续精确回归又发现 transport 终态附近需要进一步区分“实体碰撞”和“舒适间距”，见下方 2026-07-13 记录。

2026-07-13 对修复后的候选运行继续做了两轮根因验证：

1. `final6` 在 roundflocking 任务切换到 `TASK_CONTROL` 后出现一次 `tb_0` 与 `agent_4` 中心距 `0.168 m`；同一路段的 `NAV_FORMATION` 距离始终大于 `0.4 m`。根因不是中心 A*，而是任务控制启发式只取 LiDAR 扇区中点，窄机器人可能落在采样点之间；同时最终 mux 只有越线后的硬停止，没有为移动探索机器人保留控制反应时间。
2. 环绕启发式改为每个 LiDAR 扇区取最小值，避免丢失窄障碍；最终速度出口在 `TASK_CONTROL` 中增加基于中心距余量、对方最大速度和反应时间的预测限速。该层只缩放正向线速度并保留转向，不合成倒车，也不改变 transport 的响应组受控接触语义。
3. `final7` 再次完成 `7/7` 子任务且无任务失败。两次 transport 的前后实体链为约 `0.134--0.139 m`，四车同步出力；五次 roundflocking 四车累计角进度均超过 `4π`，代表性 task 3 为 `12.573/13.359/13.779/13.453 rad`。全程未出现负线速度，跨组最小中心距提高到 `0.322 m`。
4. `final7` 唯一拒绝项是第二次 transport 后自动响应 HOME 的 `START_IN_COLLISION`。按 `/sim_map` 与运行时位姿复算，最紧车辆 `tb_2` 静态净空为 `0.114644 m`：大于 Burger 实体半径 `0.11 m`，但小于实体半径加舒适间隙 `0.15 m`。原实现把舒适间隙用于“起点是否实体碰撞”，误拒绝了真实可释放状态。现在启动合法性只检查物理 footprint；进入正常空间后，路径代价、速度安全和编队包络仍继续使用安全间隙。使用同一 transport 终态注入 HOME，编队中心从约 `(12.55, 15.89)` 下降到 `(5.32, 14.19)`，并在 `69.5 s` 内报告 `DONE`。
5. Tepper 启动脚本新增 `/clock` 前置检查：Gazebo 启动前拒绝已有发布者，启动后要求 `/gazebo` 是唯一且存活的发布者，避免旧 pilot 的 epoch `/clock` 污染仿真状态机。

## 阶段 4：最终长跑与收口

### 开始前

最终验收必须同时满足：未 START 九车零速；单次 STOP 九车归零且 START 可恢复；7 个任务均 DONE；两次 transport 四车物理出力；五次环绕四车各两圈；无负线速度；跨组中心距不低于 `0.20 m`；响应组最终 HOME 完成并归零。探索 HOME 在响应任务完成后单独验证，避免人为让返航路径穿过正在执行的响应任务。

### 失败候选与根因收敛

`final8` 被主动判为失败并终止，没有沿用偶然能跑通的任务顺序：

- START 前 `/gazebo` 是 `/clock` 唯一发布者，九路最终速度为零；第 1 次 transport 完成同步 ALIGN/PUSH，响应组最小中心距 `0.134 m`，跨组最小距离保持 `0.922 m`。
- 下一任务随机派发到远端 victim `(4.35, 4.71)`。中心路径首段切线约从 `-2.795 rad` 快速转到 `-π/2`；固定槽位身份直接乘离散路径切线，导致 transport 接触队形中的前后车目标突然绕中心换位。局部速度安全不断给出侧向候选，而差速适配只能反复转向；同时接近零的小速度被当作有效平移方向，真正为零时又跳到任务终端 yaw，使振荡加剧。约 `600 s` 内中心仍滞留箱体区，最大槽位误差反复位于 `1.43--1.72 m`。
- 这不是地图点硬编码问题，而是“离散路径朝向、固定槽位身份与非完整约束”之间缺少连续状态。修复后，同一任务内重规划保持槽位身份；编队 yaw 按最短角方向限速，限速由 `中心速度 / 编队半径` 自动派生，使角点切向速度不超过中心速度预算；小于最大线速 5% 的安全速度作为死区并统一跟随编队 yaw，不再在微小候选方向和终端 yaw 间切换。
- 定向 Gazebo 回归复用了 `final8` 的四车/箱体终态与同一远目标。中心先离开原区 `1.812 m`，随后到目标距离按 `7.839, 7.111, 6.458, 5.898, 5.218, 4.290, 3.327, 2.333, 1.325, 0.359 m` 单调下降，最终 `ARRIVED` 时中心距 `0.144 m`、最大/RMS 槽位误差 `0.222/0.173 m`。与旧实现 600 秒不离开原区形成同状态对照。
- 终止 `final8` 前通过正式 STOP 命令采样九路最终命令 3 秒，最大绝对速度为 `0.0`。

后续候选继续暴露并关闭了三个与环境/时序相关的问题：

1. 响应中心规划最初把探索车预测轨迹当成静态硬墙，密集起始区可能长时间 `NO_PATH`。现将外部移动机器人作为带预测扫掠的软代价；静态地图仍决定可达性，局部 TTC/实体安全负责即时停让。移动机器人恰在目标处时只引起局部等待，不会把任务永久判为静态无路。
2. 一次干净启动中，START 已在任务管理器收到，但探索交通控制器回调被地图互斥锁阻塞约 `80.5 s`。运动许可改为原子状态；START/HOME 不再等待地图锁，STOP 通过独立发布器立即向五车发布零速并保留可恢复路径。隔离复测中任务管理器与交通控制器接收 START 的仿真时间差约 `1 ms`。
3. `final15` 在首个 transport 对位时稳定复现 `agent_3` 与 `tb_3` 中心距约 `0.27 m` 的对称死锁。逐层速度探针证明：响应任务命令为正，但响应最终安全因探索车而等待；探索侧又在 `0.35 m` 内执行“直接停车”，双方永久为零。删除这条近距停车分支后，低优先级探索车持续执行平滑分离向量，响应车保留硬安全等待。原现场热替换后两车距离从 `0.27 m` 增至 `1.98 m`，12 秒后为 `3.78 m`，transport 随即继续。

同一轮还发现四车各自独立环绕时会发生角序拥塞，90 秒内无法全部两圈。现将解析式环绕改为一组四车的循环追踪控制：按目标极角排序，根据前方自由角隙连续调节切向速度，同时保留自适应半径、径向恢复和 LiDAR 避障。热现场四车角进度为 `12.579/12.616/13.292/13.114 rad`，全部超过 `4π` 并 DONE。

### 最终干净长跑结果

完成时间：2026-07-13。

最终运行使用全新 ROS/Gazebo 主站和运行 ID `905b9026-7eeb-11f1-a18d-5853c0a43842`，入口为：

```bash
run_tepper_response_demo.sh --headless --no-vglrun
```

监视器在 START 前先运行约 12 秒，随后只发送一次 START。响应任务完成后由验收监视器请求响应 HOME；响应组到家后再单独发送一次探索 HOME，最后发送一次 STOP。正式 `ACCEPTANCE_FINAL` 证据为：

- `completed_tasks=[1,2,3,4,5,6,7]`，`failed_tasks=[]`；七个已发现任务全部依优先级执行并 DONE。
- `maximum_prestart_command=0.0`；START 前响应组没有速度输出。
- `minimum_cross_group_distance=0.257 m`，大于验收实体边界 `0.20 m`，`cross_group_collision_free=true`。
- `minimum_response_distance=0.132 m`。该值发生在 transport 的受控响应组接触中，不是导航碰撞。
- 响应 HOME 使用保留任务 `-1000` 完成，`home_completed=true`、`mode=STOP`、`motion_enabled=false`；从验收监视器启动到响应 HOME 完成 wall time 为 `1091.4 s`。

任务级证据：

- 两次 transport 均经过 `ALIGN -> synchronized PUSH -> DONE`。第一次前排距箱中心约 `0.29 m`，两条后排到前排中心距为 `0.134/0.135 m`；第二次接触最小中心距 `0.132 m`。四车组成两条“箱体—前车—后车”实体力传递链。
- 五次 surround/roundflocking 均由四车分别累计两圈完成，角进度依次为：
  - task 2：`13.067/12.571/13.114/12.603 rad`
  - task 4：`12.583/12.670/12.794/12.992 rad`
  - task 5：`12.933/12.573/12.904/12.584 rad`
  - task 6：`12.589/13.141/12.766/13.086 rad`
  - task 7：`12.724/13.301/12.566/13.328 rad`
- 标准环绕现场四车半径均约 `0.601 m` 且分布在四个象限；控制器允许空间不足时连续缩小到物理可行半径，不使用半径枚举。
- 探索 HOME 后五个 `/lazy_traffic_controller/agent_i/status` 均为 `SUCCEEDED(3)`。
- 最终 STOP 后连续采样 10 秒，九车最大线/角命令均为 `0.0`，最大模型漂移 `0.000075 m`。

### 最终回归与阶段结论

- Python：`83 passed`。
- clearance/dynamic obstacle/formation planner：`21/21`。
- formation control/arrival：`17/17`。
- navigation state/transport approach：`8/8`。
- velocity safety：`11/11`。
- exploration RVO/priority yield：`11/11`。
- 完整 `catkin_make -j4`、Python/Shell 语法和 `git diff --check`：通过。
- 断点恢复后的最终复核再次通过 Python `83/83`、全部相关 catkin gtest 零失败、两套启动脚本 dry-run 和旧运行节点引用扫描；`README.md` 与 `ROBOSAR_DEFORM_INTEGRATION_DESIGN.md` 已同步为当前单导航器、显式 HOME、实体 transport 接触和协调式自适应环绕口径。

阶段 4 已完成，当前重构主链已经闭环。后续工作应作为新阶段处理模型尺寸标定、性能基准或更多随机种子回归，不得恢复已删除的四规划器、比例枚举、直线无图兜底、领先车硬停或双重到达状态机。

## 阶段 5：闭环后的效率与 demo 一致性收口

### 开始前

开始时间：2026-07-14。

本阶段不改变已经通过长跑的任务语义，只处理闭环运行后发现的三个局部问题：首个 transport 可能保留探索车拥堵时生成的远绕路线；独立控制器 demo 在生产运动门改造后未同步打开 gate，并错误加载不存在的探索机器人；GUI 分配黑线未显式绘制任务端点。

### 根因与完成内容

1. 使用 Tepper 实际地图和当前 `FormationPlanner` 重放同一箱后 staging 目标。静态/无拥堵路径长 `10.3485 m`，最低仅到 `y=13.988 m`；当探索车分布在上方短通道时，软代价会安全地产生 `25.7193 m`、最低到 `y=5.438 m` 的临时绕行。旧实现只在移动障碍进入“当前剩余路径”时重规划；探索车离开短通道后并不与已经形成的远绕路相交，因此这条安全但低效的旧路不会恢复。
2. 动态交通相对规划快照发生实质位移后，现在直接重新优化中心路径。位移阈值仍由地图分辨率和最小编队包络派生，没有增加 Tepper 路点、周期定时器或新的经验超参数；同一任务内槽位身份和连续编队 yaw 仍由原导航状态保留。
3. `response_task_fixture_node.py` 作为独立 demo 的任务源，锁存发布 `motion_enabled=true`；生产任务链仍保持默认关闭并由 GUI START 打开。两个控制器 demo 在 mux 私有参数中将 `external_robot_names` 置空，因此不再等待不存在的 `agent_0...4` TF。
4. 普通 demo 名称改为验证 Tepper 当前生产控制器：`roundflocking -> heuristic`，`transport -> synchronized_slots`；显式 `*-checkpoint`/`*-heuristic` 入口仍可用于算法对照。这样 demo 和大系统复用同一控制模块，不维护第二套生产逻辑。
5. 任务分配图曾在每条“机器人—任务”黑线末端补画黑色 `x`，用于确认端点数据存在；阶段 7 已将它收口为与 frontier/coverage 一致的语义圆点。
6. 后续现场发现标准 ROS master 上已有仍在运行的 `/kingfisher/dodgeros_pilot` 发布 `/clock`。本阶段初版曾通过另一组 master 端口绕开冲突，虽然证明了双时钟根因，但会偏离 ROS2 桥接对标准 `11311` 的连接契约；该过渡方案已在阶段 6 删除。清理退出逻辑保留为只向本次入口记录的进程组发送信号，不按进程名全局终止其他 Gazebo、RViz 或 GUI。

### 验证结果

- `catkin_make -j4 --pkg plan_manager robosar_emergency_response robosar_task_allocator` 及最终增量全工程 `catkin_make -j4`：通过。
- 应急响应 Python 回归：`87 passed`。
- clearance / dynamic obstacle / formation planner：`22/22`；新增用例覆盖“当前远绕路不与已离开的交通相交，仍应刷新路线”。
- `run_response_controller_demo.sh roundflocking` 实跑：gate=true、mode=TASK_CONTROL、四车均有非零最终速度；mux 的外部机器人名单为空，日志无 `agent_i/base_link` 等待警告。
- `run_response_controller_demo.sh transport` 实跑：生产 `synchronized_slots` 控制器完成 ALIGN 并进入 synchronized PUSH；gate=true，日志无外部机器人 TF 等待警告。
- `launch_robosar_g.sh --dry-run --headless --no-vglrun`、两类 controller demo dry-run、launch XML、Python/Shell 语法和 `git diff --check`：通过。
- 阶段 5 的过渡性 master 测试确认：冲突来自同一 ROS 图中的旧 `/clock`，而不是 Tepper 响应逻辑。该测试结论保留，运行方式已由阶段 6 的“标准主站预清理”替代。

阶段 5 已完成。Tepper 的静态几何仍可能要求响应组先从初始上方区域向南进入主通道，这段下降是通路拓扑和箱后 staging 方向决定的；本阶段删除的是拥堵消失后继续保留的地图级大绕行，而不是强制走直线或穿过探索机器人。

## 阶段 6：标准主站清洁启动与 ROS1/ROS2 桥接契约

### 开始前

开始时间：2026-07-14。

本阶段只收口完整 Tepper 入口，不改变已经长跑闭环的导航、环绕、运输、HOME 或 GUI 状态机。目标是让 `/root/workspace/ROS2swarm_w` 的 `ros1_bridge` 始终可按标准端口连接，同时让一次普通启动能够自行消除上次运行残留，而不是要求操作者换端口。继续遵守“一条主逻辑链路”原则：完整系统不保留功能等价的第二套 master 模式。

### 根因与完成内容

1. `/root/workspace/ROS2swarm_w/start_robosar_webots.sh` 明确把 `ROS_MASTER_URI` 设置为 `http://${ROS1_MASTER_IP}:11311`。完整 Tepper 若默认转到其他端口，ROS1 系统自身可运行，但桥不会看到该 ROS 图；这是端口规避方案与跨项目接口契约的直接冲突。
2. `run_tepper_response_demo.sh` 现在默认使用调用者给定的标准主站；未给定时为 ROS `11311`、Gazebo `11345`。ROS 主机地址仍可按跨容器网络配置覆盖，但没有另一个默认端口分支。
3. 启动预检先枚举除 `/rosout` 外的已有节点，用一次有总超时的 `rosnode kill` 请求退出，再执行失效注册清理并等待图为空；仍被 supervisor 拉起或拒绝退出的节点会被明确列出并导致启动失败。之后删除旧参数树，最后验证 `/clock` 无发布者。整个过程基于 ROS 图，不绑定 Kingfisher、Gazebo 或具体机器人进程名。
4. 入口只保留两种语义：默认 `clean` 负责标准主站的完整系统启动；`--shared-master` 用于已有图由外部桥或编排器管理的情况，只检查、不清理，且拒绝双时钟。曾加入的独立端口选项、端口变量、回归用例和文档入口全部删除。独立 controller/deform 小 demo 仍可使用各自测试主站，它们不属于跨项目生产链。
5. 新增 EXIT 清理保护：无论正常退出、Ctrl+C 还是启动中途失败，都只清理本次脚本记录的进程组；重复触发清理不会再次发送信号。

### 验证结果

- Shell 语法、入口 dry-run、`--shared-master` URI 保留行为、文档差异检查通过。
- 响应组 Python 回归 `87 passed`；完整 Tepper 默认 dry-run 明确显示 `master_mode=clean`、ROS `11311`、Gazebo `11345` 和 master preflight，代码中已无 Tepper 独立端口选项或变量。
- 在标准 `11311` 上实建旧 roscore，并放入持续发布 `/clock` 的模拟遗留节点后，直接运行默认 headless Tepper：预检主动关闭旧节点，旧 `/clock` 消失；随后 `/gazebo` 成为唯一时钟发布者，`/response_group/motion_enabled=false`，`formation_navigation`、mission、response mux/task/global-info 等核心节点均存活。
- Ctrl+C 后本次启动的所有进程组均退出，测试 roscore 也由测试夹具回收；未发现遗留 `gzserver`、formation navigation 或 ROS master。该实跑验证的是本阶段改动涉及的启动/清理边界；此前阶段 4/5 的完整任务长跑闭环结论不因 master 入口收口而改变。

阶段 6 已完成：完整 Tepper 当前只有“标准主站清洁启动”这一条默认链路，并保留外部编排所需的非清理共享模式；不再通过换端口回避运行残留。

## 阶段 7：探索效率、GUI 与仓库收口

### 开始前

开始时间：2026-07-14。

本阶段处理闭环后现场发现的三个问题：transport 完成后仍处于“箱体—前车—后车”接触链，下一任务转向曾出现同伴翻车；任务分配线端点只有调试叉号而没有任务圆点；探索组对响应机器人存在重复避障和高频状态锁竞争。同时清理已退出引用链的旧实现、测试世界和生成缓存；此前确认的需求/实施规划文档继续保留。针对第一个问题曾试验 post-push RELEASE，后按用户要求撤回，不再改变已完成闭环的 transport 主链。

### 根因与当前完成内容

1. transport 原主链在箱体满足目标保持时间后立即发布 DONE，任务管理器随后切回 `NAV_FORMATION`。一次现场运行在接触链尚未自然散开时转向并出现翻车，因此曾尝试加入 post-push RELEASE。
2. RELEASE 试验采用分排直退和箱体旋转包络清距，并在长跑诊断期间临时放宽了 ALIGN 的派生位置容差。该试验扩大了状态机和完成判据，且后续运行暴露了新的任务切换不稳定性。
3. 按用户要求，本轮已完整撤销上述机器人控制调整：删除 RELEASE 状态、分排直退、箱体清距判据及对应测试；恢复 `ALIGN/PUSH -> 箱体目标保持 -> DONE`，并把 ALIGN 派生容差恢复为物理余量的 `0.45` 倍。四车物理接触合推、槽位分配和同步推进逻辑均保留。
4. 环绕长跑中四车在 90 秒分别累计 `8.572/6.913/9.579/10.161 rad`，均在安全环内持续前进，但旧绝对超时仍把任务判为 FAILED。现保留“两圈且全员”的几何完成判据，把原时限改成“未完成成员无新增角进度”的停滞窗口；已完成成员继续转圈不能掩盖卡住成员。这样慢速避障不会被误杀，真实卡死仍会失败，且未新增环境专用时长。
5. 对 SURROUND 到达切换做了回看：一次试验运行正常从任务 1 切到任务 3 的 TASK_CONTROL，另一次则在中心、RMS 和净空均合格时因单车误差 `0.66–0.72 m` 长期停留。曾提出的 `0.75 m` 放宽属于对单次样本的下游补丁，复核后已撤回，继续保持此前完成闭环所用的 `0.60 m`，不新增恢复状态机。
6. 探索侧此前把响应机器人同时加入普通 RVO/凝聚排斥和非对称优先权投影，同一对象被处理两次；`/gazebo/model_states` 回调还与地图/RVO共用互斥锁。现只保留一条优先权投影，状态更新使用独立快照锁。
7. 让行速度在最小实体距离到让行半径之间连续放宽；半径边界完整保留原 RVO 命令，进入实体距离才合成分离速度，并计入响应机器人径向速度。这样响应组仍有通行优先权，同时不再让远处静止响应车无谓迫使探索车倒退。
8. 已分配任务即使离开候选散点集合，也会按类型重画绿色 frontier 圆点或青色 coverage 圆点；黑线仍只表达分配关系。
9. 仓库清理遵循引用链审计：生产入口、controller/deform demo 和算法对照入口保留；只删除无 launch/build/test/doc 引用的遗留物。`docs/` 顶层四份文档作为当前入口，`docs/superpowers/` 两份已确认规划作为历史冻结快照保留。

### 当前验证

- transport/任务定向逻辑：`32 passed`；RELEASE 专用实现和 3 个专用用例均已删除。
- 响应组完整 Python 回归：`89 passed`。
- exploration RVO/priority yield：`12/12`；新增渐进闭合速度和让行半径边界原命令保持用例。
- `robosar_controller` 库和 `robosar_traffic_controller_node`：重新编译通过。
- Python 语法与差异检查：当前修改通过。

### 当前状态

阶段 7 尚未结束。post-push RELEASE 及其伴随的 ALIGN 容差调整已撤回，transport 恢复此前完成过整链闭环的 `ALIGN/PUSH -> DONE` 主链。探索、GUI 和环绕的独立改动保留；不再依据单次随机运行扩大 transport 状态机或放宽 SURROUND 到达阈值。完整 Tepper 最终闭环仍列为未完成验收项。

### 2026-07-14：transport 对位摆动与探索初始扎堆

开始前只处理两个局部根因，不改变 transport 状态机、完成判据、最终安全或探索运动控制器：一是四车到箱后长时间在厘米级位置误差与推送朝向之间反复切换；二是五台同起点探索车收到同一簇近前沿。

根因与改动：

1. 现场失败样本中 transport 已到 `ALIGN`，四车位置误差约为 `0.006/0.020/0.017/0.017 m`，但其中一车推送朝向误差约 `0.429 rad`；首次位置阈值为物理余量派生的 `0.018 m`，前排槽位又正处在箱体接触面。差速车贴箱原地转向会发生小位移，控制器因此反复切回追槽位，180 秒内没有进入 PUSH。
2. 不放宽首次阈值，也不增加恢复状态。ALIGN 前排槽位统一在接触面外增加已有 `robot_gap`，后排仍按 `robot_length + robot_gap` 排列；单车首次进入严格位置带后，只要转向漂移没有耗尽几何自由余量就保持位置就绪。PUSH 仍切换到“箱体—前车—后车”实体接触槽位。
3. `TA_HIGH_OA` 原来让每台车只从自己的五个最近 frontier 中独立贪心选点。同起点使五份候选几乎相同；目标集合固定后才调用效用折扣，Hungarian 只能重新匹配这组已扎堆目标，无法创造空间分散。
4. 现在仅 OA 车队分配器查看全部已聚类 frontier，并在每选中一个车队候选后立即执行已有的信息重叠折扣，再用原 Hungarian 匹配机器人。距离缓存键同时改为 `(task_type, task_id)`，避免 frontier 与 coverage 同号时串用代价；普通 `TA_HIGH` 仍保留最近五点逻辑。

验证与反馈：

- 新增用例先稳定复现两个缺口，修复后 transport 定向用例 `11 passed`、响应组完整 Python `90 passed`、探索联合分配 `1 passed`；Python 编译、差异检查及 `catkin_make -j4 --pkg robosar_emergency_response robosar_task_allocator` 通过。
- 干净 Tepper 启动中 START 前 gate=false、模式 STOP，四路响应 `cmd_vel` 均为零。START 后探索车从约 `(3.5--4.3, 14.5--15.2)` 的密集出生区分别展开到约 `(4.0,8.4)`、`(4.6,6.8)`、`(8.6,14.5)`、`(10.0,14.6)`、`(11.2,13.9)`，不再全部沿同一局部前沿簇运动。
- 同轮首个 transport 在 `12 s` 内从前排进入后排 ALIGN，但 `tb_2` 被距离约 `0.38--0.43 m` 的 `agent_2` 长期截断最终线速度；`tb_2` 的任务层持续给出约 `0.15 m/s`，而探索车只原地旋转，最终仍因 180 秒超时失败。该轮没有到达贴箱转向场景，暴露的是既有跨组让行停滞，不能据此修改本轮 transport 判据或宣称完整 Tepper 通过。
- 随后使用无探索车干扰、复用生产 `synchronized_slots` 控制器的 transport demo：`08:22:49` 分配槽位，`08:22:59` 释放到后排，`08:23:11` 进入同步 PUSH，`08:24:06` 报告 DONE。终态两条后排—前排中心距约 `0.135/0.134 m`，前排—箱体中心距约 `0.318/0.314 m`，确认四车实体力传递没有被非接触 ALIGN 改坏。

本轮两个代码目标均已完成定向验证，但阶段 7 仍未结束：探索目标分散已生效，transport 自身已通过 `ALIGN -> PUSH -> DONE`；完整 Tepper 最新轮仍存在探索车让行只转不走导致的跨组阻塞，需作为下一项独立根因处理并重新做完整闭环，不能用独立 demo 代替整链验收。
