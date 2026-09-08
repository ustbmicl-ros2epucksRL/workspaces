# 第四章研究实施方案：面向未知搜救环境的交互感知异构任务图角色分配

> 本方案只规划第四章的研究和实现，不把尚未完成的代码或实验写成既有结果。研发顺序固定为：先在独立 `rescue-task-allocation` 容器中对 Explore-Bench 母工程进行代码扩展并冻结本章方法，取得方法自身的有效性结果；再复现正式基线、开展公平比较和消融；最后才在隔离的 MASAR 副本中接入验证。当前可运行的 MASAR 容器和工作区不是开发环境。

## 1. 章节定位与最小承诺

### 1.1 章节问题

在未知搜索环境中，地图不断更新，frontier 候选不断生成，coverage 候选具有稳定 id 和 visited 生命周期，机器人还存在位置、空闲/忙碌状态和不同的到达代价。第四章研究：

> **如何利用 Robot–Frontier–Coverage 的异构任务上下文和候选之间的交互信息，在可达性、唯一分配和任务生命周期约束下，为搜索发现组产生稳定、有效且可回退的任务分配？**

这里的输出是 `robot -> task`，不是速度控制，也不是应急响应行为。第三章的行为策略在第五章中消费已经选定的 `EmergencyTask`；第四章先解决搜索组的任务层选择。

### 1.2 最小可行研究路线

```text
RS-Base（Role Selection/Explore-Bench 可复现母线）
       |
       +-- Type-L：加入 frontier/coverage 类型编码
       |
       +-- TypeLife-L：加入生命周期、服务时间、地图阶段上下文
       |
       +-- Hetero-RS：关系类型感知效用重评分（复用现有可行匹配器）
       |
       +-- Hetero-RS -Interaction：去掉候选交互边，验证交互模块贡献
       |
       +-- feasible solver + HIGH_OA fallback
```

推荐先实现 `Type-L` 和 `Hetero-RS` 两个清晰版本。时间不足时，`TypeLife-L` 可作为扩展消融，不能因此阻塞整章。

这里的“先完成自己的内容”具有明确验收含义：完整的数据流、训练、推理、约束分配、checkpoint、独立测试集结果和接口契约测试均已完成，而不是仅把网络类写出来。此阶段只判断方法是否能学习、能完成任务并满足可行性阈值，不根据其他算法结果反复调整模块。自己的方法和超参数冻结后，才开始正式基线比较与消融。

## 1.5 容器隔离方案（推荐且可直接执行）

可以为第四章新开 Docker 容器，直接将 Explore-Bench 母工程检出并改名为 `/home/zjc/workspace/rescue-task-allocation`，再通过“全 workspace 只读 + 母工程目录读写覆盖”的方式进入容器。这样容器能读取 MASAR、ROS2swarm、论文和其他参考仓库，但只有母工程目录可写。关键不是“用了 Docker”本身，而是写权限边界必须小于当前 MASAR 的挂载边界。

### 宿主机目录

```text
/home/zjc/workspace/rescue-task-allocation/
  grid_simulator/      # Explore-Bench 原有 Level-0 环境和 A*
  onpolicy/            # Explore-Bench 原有 MAPPO 训练/评测入口
  chapter4/            # 第四章新增模块、训练入口和消融脚本
  configs/             # 方法配置和冻结后的实验配置
  data/                # 母工程数据和导出数据索引
  checkpoints/         # checkpoint、日志、指标
  tests/               # 纯 Python 单元测试和契约测试
  docker/
    Dockerfile
    compose.yaml
```

建议将 Explore-Bench 母工程改名为 `/home/zjc/workspace/rescue-task-allocation`。整个 `/home/zjc/workspace` 可以挂载进容器，但必须是只读；母工程目录再以读写卷覆盖挂载。不要把整个 workspace 作为可写卷，也不要把 `/home/zjc/workspace/masar_ws` 单独作为可写卷挂进新容器。

### `rescue-task-allocation` 容器的安全边界

| 项目 | 研发阶段设置 | 原因 |
| --- | --- | --- |
| 整个 workspace | `/home/zjc/workspace:/root/workspace:ro` | 容器可读取所有项目和参考资料，但默认不能写入 |
| 母工程目录覆盖 | `/home/zjc/workspace/rescue-task-allocation:/root/workspace/rescue-task-allocation:rw` | 只有 Explore-Bench 母工程及第四章扩展可写 |
| `masar_ws` | 通过父目录只读可见，不单独可写挂载 | 可读接口和源码，避免误改 `build/devel` 和 ROS 包 |
| `ROS2swarm_w` | 通过父目录只读可见，不单独可写挂载 | 可读取第三章接口，不改变其运行时 |
| 网络 | `network_mode: host`，但 O1–O3 不启动 ROS、不 source ROS 环境、不设置 MASAR master | 兼容用户现有 GPU/display 启动方式；算法进程本身不会自动加入 MASAR 任务流 |
| GUI/X11 | 显式传入 `DISPLAY`，只读挂载宿主 Xauthority，挂载 `/tmp/.X11-unix` | 支持 CUDA/display smoke test 和后续可视化；不启动 Gazebo/RViz |
| GPU | `gpus: all`、`/dev/dri`、`NVIDIA_VISIBLE_DEVICES=all`、`NVIDIA_DRIVER_CAPABILITIES=all` | 训练和 CUDA smoke test 使用独立容器；不修改当前 MASAR 容器 |
| 用户权限 | 非 root 用户运行训练进程；研究目录 UID/GID 与宿主用户一致 | 避免容器产生 root-owned 文件 |
| 依赖 | 容器内独立 `requirements.lock`/镜像 tag | 不改变宿主 Python、ROS 或 MASAR 依赖 |

容器使用 host network 以兼容用户给出的启动方式，但不启动 ROS 节点、不设置 MASAR 的 `ROS_MASTER_URI`、也不 source `/opt/ros/noetic/setup.bash`。这样 host network 只是网络命名空间共享，不会自动加入 MASAR 任务流。如果后续只读取 MASAR 导出的 rosbag/JSON/Parquet，应以单独的只读卷挂载：

```text
/home/zjc/workspace/masar-data:/workspace/masar-data:ro
```

不要把 `/home/zjc/workspace` 作为一个总目录以 `rw` 挂载。当前平台的 ROS1/ROS2 业务容器正是把整个 workspace 以可写方式挂载到 `/root/workspace`；第四章容器可以复用“全 workspace 可见”的路径，但必须改成父目录 `ro`、研究目录 `rw` 的嵌套挂载。

### 推荐容器阶段

```text
rescue-task-allocation
                    GPU/display + host network, workspace ro + mother project rw
        |
        | 方法实现、训练、自测、独立测试集结果
        v
chapter4-baseline   可联网但不接 ROS，复现母工程和公平基线
        |
        | checkpoint/config/接口冻结
        v
masar-integration   独立 MASAR 副本 + adapter，最后才连仿真
```

`masar-integration` 不是当前 `masar_ros1_srv-g` 的替换，也不是在现有运行容器里直接编辑文件；它应使用 `masar_ws-rescue-task-allocation` 副本、独立 compose project name 和独立端口/ROS master。首次集成只做离线回放和 smoke test，成功后才做一次完整 Gazebo/Webots 场景。

### 1.3 贡献表述边界

可以形成的贡献是：

1. 面向未知搜救任务的异构候选图和生命周期建模；
2. 将关系类型感知的图效用编码接入现有可解释的可行任务分配器，并验证其相对 HIGH_OA 的增益；
3. 在开源母工程和 MASAR 外部场景中证明任务层指标、接口兼容性和回退安全性。

不要声称：

- 从零发明 HGT/GNN；
- 把现有 HIGH_OA 的顺序候选生成、重叠折扣和 Hungarian 重新包装成原创的全局求解器；
- 用一个网络替代 A*/RRT/控制器；
- benchmark 的 coverage 结果就是 MASAR 搜救 coverage 结果；
- 尚未运行的系统实验已经优于 HIGH_OA。

### 1.4 对当前“交互感知图匹配”设想的严格判断

| 判断项 | 结论 | 理由与处理 |
| --- | --- | --- |
| 研究问题是否与总题目一致 | **一致** | 它直接服务于搜索发现组，输出任务分配，并为第五章产生可执行的探索动作 |
| 是否有真实系统数据支撑 | **有条件一致** | MASAR 同时存在 frontier 与 coverage 语义，但必须先做 E0 统计其共存率、生命周期和 overlap；若共存率很低，应改成分阶段任务选择而非强行异构图 |
| 全新端到端图匹配是否适合当前周期 | **不建议** | 会同时改变候选生成、匹配、路径规划和状态机，难以归因，也会放大 ROS 接口风险 |
| “学习效用 + 现有匹配器”是否可实施 | **推荐** | 算法研发阶段在独立包中实现 scorer 和通用约束匹配；最终系统接入时才复用 HIGH_OA、A*、任务图和 `/task_allocation` |
| 是否足以形成章节贡献 | **可以，但需实验兑现** | 贡献来自搜救异构上下文、生命周期/可达性约束、低冗余效用和系统外部验证的组合；不能只靠换一个网络层名称 |
| 是否能支撑第五章 | **接口上可以** | 输出仍是 `task_allocation.msg`；但状态枚举、coverage 完成语义和第三章响应控制器适配必须先解决 |

因此建议把章节标题和方法叙述从“全局交互感知图匹配算法”收敛为：

> **交互感知的异构任务效用学习与约束分配方法。**

这个命名更准确地反映当前工程：交互项用于效用重评分，最终匹配沿用可解释的约束求解器；它也为消融实验提供了清晰的因果边界。

## 2. 相关工作、母工程和复用清单

### 2.1 主母工程：Explore-Bench Level-0

- 论文：*Explore-Bench: A Benchmark for Multi-Robot Exploration*（ICRA 2022），<https://arxiv.org/abs/2202.11931>。
- 代码：<https://github.com/efc-robot/Explore-Bench>。
- 本地检出目录：`/home/zjc/workspace/rescue-task-allocation`；固定来源 commit：`ce3b98a09bf593b0716b26555dc4cccbd9e83cd3`。
- 适用原因：包含网格未知环境、A*、frontier exploration、MAPPO/on-policy 入口、场景 blueprint 和统一指标，适合先得到可重复结果。
- 适配动作：隔离旧依赖和绝对路径；固定地图/种子；把其 coverage target 明确标为 benchmark coverage，不与 MASAR 的几何 coverage task 混名。
- 代码策略：保留母工程原始目录和入口；第四章新增内容集中在 `chapter4/`，必要时只对训练入口做最小显式调用，不在母工程核心环境中隐式改变原有语义。

### 2.2 直接角色基线：Role Selection

- 论文：*Role Selection for Multi-Agent Reinforcement Learning*（arXiv:2312.01747），<https://github.com/linaug/Role_selection>。
- 可复用：`adversarial_comms/environments/coverage.py` 的随机 coverage 环境、PPO trainer、角色决策接口。
- 风险：GPL-3.0；Ray/Python 依赖较旧，仓库规模小；必须先做 reproduction gate，不能假定开箱即用。

### 2.3 图表征参考：NeuralCoMapping

- 论文：*NeuralCoMapping*（CVPR 2022），<https://arxiv.org/abs/2203.16319>；代码：<https://github.com/siyandong/NeuralCoMapping>。
- 可复用思想：机器人节点、候选/frontier 节点、self/cross attention、历史信息和图匹配。
- 不可直接复用：iGibson 数据、网络权重、观测/action 定义和原论文数字。

### 2.4 理论支撑

- Heterogeneous Graph Transformer（WWW 2020，<https://github.com/acbull/pyHGT>）：关系/节点类型特定的注意力。
- Multi-Robot Coverage and Exploration using Spatial GNN（IROS 2021，<https://github.com/katetolstaya/graph_rl>）：空间图和 coverage 表征。
- ROMA/RODE：角色分解和角色条件策略的背景，不作为搜救任务的直接母工程。

### 2.5 MASAR 原生组件的使用边界

第四章**会使用 MASAR 的既有实现内容，但仅限接口参考、离线数据来源和最终系统接入**，不把当前正在运行的 MASAR 当作算法开发环境。最终接入时保留：

- frontier filter 与 info gain；
- `task_graph_getter` 返回的 coverage task id、位置、类型和 info gain；
- `CostCalculator.a_star_cost()` 的地图路径代价和可达性判断；
- 现有 RRT/coverage 重叠过滤；
- `lazy_traffic_agent.cpp` 的路径执行及 coverage 到达后的旋转/搜索状态机；
- `TA_HIGH_OA` 作为后续传统基线和运行时回退。当前实现准确地说是“顺序候选选择 + 重叠效用折扣 + Hungarian 按路径代价重排”，不是一个直接优化所有交互项的全局联合求解器。

研发阶段只冻结并重建一个最小的纯数据契约：机器人状态、frontier、coverage、路径代价、地图阶段和分配输出。算法包不导入 `rospy`，不 source `masar_ws/devel/setup.bash`，不写入 `masar_ws`，也不要求 MASAR 容器在线。

### 2.6 开源母工程与许可证闸门

1. **先做环境可执行性检查，不先做正式基线比较**：记录母工程 commit、Python/PyTorch/Ray 版本、地图 split、episode horizon 和 seed，只验证环境 `reset/step`、A*、指标和数据形状能运行。该 smoke test 不产生论文比较结果，也不用于调节本方法以迎合基线。
2. **许可证隔离**：Role Selection 的仓库包含 GPL-3.0 许可证，若复制代码或发布衍生代码，必须保留许可证并评估与论文/平台代码的兼容性；NeuralCoMapping 当前仓库为 MIT，可复用代码时仍需保留版权声明；Explore-Bench 若未在仓库顶层明确许可证，优先以外部依赖、配置和接口复用为主，不直接复制大段实现。
3. **方法冻结后再复现基线**：Explore-Bench、Role Selection 和 HIGH_OA 的正式性能复现放到本章方法验收并冻结之后。每个实验表格应注明是“论文复现”“仓库复现”“本地重实现”还是“系统适配”。

## 3. 问题形式化

在重规划时刻 `t`，定义：

```text
S_t = (M_t, R_t, T_t, L_t)
```

- `M_t`：当前局部/全局地图及覆盖率；
- `R_t`：机器人位置、活动状态、当前任务、控制器状态；
- `T_t = F_t ∪ C_t`：frontier 集合和未访问 coverage 集合；
- `L_t`：任务生命周期、年龄、已分配次数、估计服务时间和最近观测时间。

对每个机器人 `r_i` 和候选任务 `q_j`，输出效用 `u[i,j,t]` 和可行性 mask `m[i,j,t]`。最终分配矩阵 `X_t` 满足：

```text
X[i,j] ∈ {0,1}
X[i,j] <= m[i,j]
sum_j X[i,j] <= 1
sum_i X[i,j] <= 1
```

当候选为空、全部不可达、模型超时、输入版本过期或消息状态不可信时，返回 `WAIT` 或 HIGH_OA，而不是输出无效 goal。

### 3.1 候选节点

| 节点类型 | 最低字段 | 来源 | 生命周期 |
| --- | --- | --- | --- |
| Robot | `id, x, y, active, status, current_task, battery(optional)` | TF/odom、controller status、allocator state | 长期存在，active 可变 |
| Frontier | `id_ephemeral, x, y, info_gain, age, reachable` | `/frontier_filter/filtered_frontiers` | 地图更新时重建，不能跨更新假定 id 稳定 |
| Coverage | `id_stable, x, y, info_gain, visited, assigned, overlap` | `/robosar_task_generator/task_graph_getter`、RRT | 跨更新保留，直到 visited/失效 |

### 3.2 推荐特征

所有连续特征先在训练集统计量下标准化或裁剪到固定范围，禁止使用测试地图统计量：

```text
robot: [relative_dx, relative_dy, euclidean_distance,
        a_star_cost, reachable, active, controller_idle,
        current_task_type, battery(optional)]
frontier: [relative_dx, relative_dy, info_gain, age,
           local_unknown_ratio, candidate_density]
coverage: [relative_dx, relative_dy, info_gain, visited,
           assigned, estimated_service_time, overlap_score, age]
global: [coverage_ratio, frontier_count, coverage_count,
         active_robot_count, map_seq_delta, emergency_queue_size]
```

其中 `estimated_service_time` 在没有真实标注时可以由任务类型和历史到达/搜索时长的稳健均值初始化；若该特征噪声过大，先只做二值 `coverage`/`frontier` 类型和年龄消融。

## 4. 方法设计

### 4.1 CandidateGraph

构建异构图 `G_t = (V_t, E_t)`：

- 节点类型：`Robot`、`Frontier`、`Coverage`；
- `Robot -> Frontier/Coverage`：可达、路径代价、方向和当前占用关系；
- `Frontier/Coverage -> Frontier/Coverage`：空间邻近、footprint overlap、相互信息冗余/互补；
- `Robot -> Robot`（可选）：距离和当前目标冲突，用于抑制重复派遣；
- edge mask：不可达、已 visited、非 active 或状态冲突的边直接屏蔽。

候选数量过大时，只保留每个机器人按 A* cost 和几何距离的 top-`K` 候选，并保留全局高 info gain 候选，避免图规模随地图无限增长。

### 4.2 关系类型感知编码

不要求复刻完整 PyG/HGT 工程，可用一个小型、可审计的实现：

\[
h_v^{(0)}=\mathrm{MLP}_{type(v)}(x_v),
\]

\[
m_{u\to v}^{(l)}=\phi_{r(u,v)}(h_u^{(l)},e_{uv}),
\quad
h_v^{(l+1)}=\mathrm{GRU}(h_v^{(l)},\sum_{u\in\mathcal N(v)}\alpha_{uv}m_{u\to v}^{(l)}).
\]

关系类型参数 `r` 至少区分：`robot-task`、`task-task-overlap`、`robot-robot-conflict`。若实现成本需要进一步压缩，可把 GRU 换成 residual MLP，但必须保留 type-specific encoder 和 edge mask。

### 4.3 任务效用头（本章真正替换的模块）

对每个机器人—任务边输出：

\[
u_{ij}=w^T[ h_{r_i}\Vert h_{q_j}\Vert e_{ij}\Vert g_t ]+b,
\]

并加入显式的可解释项。若使用 MASAR 原生 HIGH_OA，建议保留其顺序候选和 overlap discount，只学习一个增量重评分项，而不是完全重写目标：

\[
\tilde u_{ij}=u^{HIGH\_OA}_{ij}+\Delta u^{graph}_{ij},
\]

其中 `u_HIGH_OA[i,j]` 保留信息收益、探索/利用权重、路径代价和已选任务重叠折扣；

\[
\Delta u^{graph}_{ij}=f_\theta(h_{r_i},h_{q_j},e_{ij},g_t)
-\lambda_s\,\mathrm{serviceTime}_j
-\lambda_a\,\mathrm{agePenalty}_j.
\]

在纯 benchmark 实验中也可以从零学习 `u[i,j]`，但 MASAR 外部验证应优先采用增量形式，以降低分布偏移和接入风险。

系数可以学习，也可以在训练前冻结一组可审计权重。实验必须区分“网络学到的效用”和“手工项”，否则无法解释提升来自哪里。

### 4.4 约束分配与回退

1. 对无效边设 `-inf`，对可行边使用 `-score[i,j]` 作为候选 cost；
2. 复用现有一对一 Hungarian/可行匹配器；若 HIGH_OA 的顺序候选阶段已先锁定候选，则只在该候选集合上重排，不声称求得全局最优；
3. 过滤低置信度、过期地图版本或超出最大等待时间的结果；
4. 若没有有效结果，调用 HIGH_OA；
5. 回退原因、模型版本、输入 `map_seq`、推理时延和最终任务 id 写入日志。

伪代码：

```python
def allocate(snapshot):
    graph = build_candidate_graph(snapshot)
    if graph.empty_tasks or graph.stale or graph.invalid:
        return high_oa_or_wait(snapshot, reason="invalid_snapshot")

    scores = policy(graph)                         # [n_robot, n_task]
    scores = mask_infeasible(scores, graph.edge_mask)
    pairs = hungarian(-scores)                     # one-to-one feasible pairs
    pairs = filter_confidence_and_version(pairs, snapshot)
    if not pairs:
        return high_oa_or_wait(snapshot, reason="no_feasible_pair")
    return make_assignment_decision(pairs, snapshot)
```

## 5. 与 MASAR 第五章的接口适配

### 5.1 现有输入契约

已核查的主要来源：

- `robosar_task_allocator/src/task_allocator/Environment.py`：frontier 每次更新重建，coverage task 以 `coverage_tasks_dict` 保存并维护 `visited`；`update_tasks()` 将两者合并到 `available_tasks`。
- `high_ta.py`：读取 `/sim_map` 计算总面积和覆盖率，读取 `/frontier_filter/filtered_frontiers`，通过 `/robosar_task_generator/task_graph_getter` 获取 coverage task，使用 `CostCalculator` 和现有 solver。
- `CostCalculator.py`：使用当前地图做 A* 路径代价，返回 cost 和 reachable 标志。
- `task_graph.cpp`：提供 coverage task id、坐标、类型和 info gain；task graph 会随地图更新。

建议在新 allocator 内部统一成以下数据类（不必新增 ROS msg）：

```python
@dataclass
class MASARSnapshot:
    map_msg: OccupancyGrid
    map_seq: int
    coverage_ratio: float
    robots: list[RobotState]
    tasks: list[TaskState]
    path_costs: np.ndarray
    timestamp: float

@dataclass
class AssignmentDecision:
    robot_id: str
    task_id: int
    task_type: int       # 1 frontier, 2 coverage in current adapter
    start_xy: tuple[float, float]
    goal_xy: tuple[float, float]
    score: float
    policy_version: str
    map_seq: int
    timestamp: float
    fallback_used: bool
    fallback_reason: str | None
```

### 5.2 输出契约

现有 `robosar_messages/msg/task_allocation.msg` 为：

```text
string[] id
float32[] startx
float32[] starty
float32[] goalx
float32[] goaly
int64[] goal_type
int64[] goal_id
```

发布时七个数组必须严格等长，且第 `k` 个元素代表同一个机器人任务。`mission_executive` 按机器人索引读取，因此不能只为成功匹配的机器人填充数组而忽略其余索引；推荐保留现有 `publish_visualize()`/消息构造逻辑，只把 `names, starts, goals, goal_types, goal_ids` 的产生函数替换为 adapter。

### 5.3 最终集成替换点（不是研发阶段操作）

方法冻结并完成独立实验后，才进入 MASAR 集成。按风险从低到高：

1. **推荐**：在 MASAR 副本中新增 `interaction_high_ta.py`，由启动配置选择 allocator；保留 `high_ta.py`、任务循环、路径规划、控制器和回退逻辑。
2. **可选**：在 MASAR 副本的 `high_ta.py` 中替换 solver 的评分函数，保留状态循环、发布和回退；该方案改动小，但不如新增模块容易回滚。
3. **不建议**：重写 task generator、controller 或新增自定义 ROS 消息；这会把第四章变成平台重构项目。

研发阶段的 `rescue-task-allocation` 代码不导入 `rospy`，不写入当前 `masar_ws`，不替换正在运行的 `high_ta.py`。集成提交应能通过“删除整个 MASAR 副本并从干净副本重建”的回滚检查。

### 5.4 第五章可适配性判断

| 接口 | 第四章输出是否能直接适配 | 需要的调整 |
| --- | --- | --- |
| `/frontier_filter/filtered_frontiers` | 是 | 只需转换为稳定的 episode-local frontier id |
| `task_graph_getter` | 是 | 保留 coverage id/type/info gain（当前 id 由 `vertex_id * 10000 + node_id` 构造），处理服务失败和空返回 |
| `CostCalculator` | 是 | 复用 A* cost/reachable，统一地图 frame 和时间戳 |
| `/task_allocation` | 是 | 保持七数组等长和当前 goal type 编码 |
| coverage controller state machine | 是 | 不改行为；分配器等待真正 SUCCEEDED/搜索结束 |
| `EmergencyTask` | 间接 | 第五章新增 discovery adapter，将发现事件转成响应组任务 |
| 第三章 actor | 间接 | 第五章为 EmergencyTask 做 observation/action adapter 和 checkpoint 选择 |

结论：第四章可以作为一个独立 Python adapter 先完成；第五章不需要重写消息定义，但必须修正状态语义、补充事件映射和时序测试。

第三章执行策略的接入不能被简化成“把 checkpoint 换进去”：ROS2swarm 当前使用 `/robot_N/cmd_vel`、`/global_info` 和 ROS2 观测；RoboSAR 响应组使用 `/tb_i/deform_cmd_vel`、`/tb_i/task_cmd_vel`、`/tb_i/cmd_vel` 和 ROS1 消息。真正替换响应执行器至少需要 observation/action adapter、动作缩放、任务模式门控和最终速度 mux 接入。第五章第一阶段应先用现有响应控制器验证发现到响应的系统闭环，再将第三章 actor 作为可替换执行模块。

## 6. 必须先完成的 P0/P1 风险与测试

### P0：状态枚举冲突

`controller_status.msg` 定义：

```text
IDLE=1, BUSY=2, SUCCEEDED=3
```

而 `task_listener_robosar_control.py` 内部常量写成 `BUSY=1, IDLE=2, GOAL_SENT=3`，并在 callback 中重新映射；`high_ta.py` 第 231 行附近使用 `status == 2` 判断 robot reached。必须二选一：

- 统一内部常量并显式比较 `controller_status.SUCCEEDED`；或
- 新 adapter 直接订阅原始消息，建立 `IDLE/BUSY/SUCCEEDED/FAILED` 状态机。

最小单元测试应覆盖：

```text
initial IDLE -> allocation -> BUSY -> controller SUCCEEDED -> exactly one reallocation
BUSY without success -> no duplicate reallocation
FAILED/timeout -> fallback or quarantine
```

### P1：地图、id 和候选语义

- `/sim_map` 的已知区域用于总面积/覆盖率；A* 主要使用 `/slam_toolbox/map` 或 `/map` 回退，实验报告必须分别记录。
- frontier 索引每次更新可能改变（当前由 `enumerate` 重新编号）；不能将其当成长期 id。
- `PointArray.infoGain` 经过 filter 的周期内归一化，随后又可能在 `high_ta.py` 中按当前最大值归一化；比较实验必须记录归一化阶段，不能把数值当作跨场景绝对信息增益。
- coverage id 来自 task graph（当前由 `vertex_id * 10000 + node_id` 构造），必须稳定维护 visited/assigned。
- 空 frontier、空 coverage、服务超时、地图版本过期和不可达候选必须有显式路径。
- `/task_allocation_image` 只用于可视化，不作为模型输入。

## 7. 训练与实现分阶段方案

### C0：容器和数据契约 smoke test（1–2 天）

在 `rescue-task-allocation` 容器内完成，不启动 ROS，不连接当前 MASAR；GPU/display 只用于 CUDA 和可视化 smoke test：

- 验证容器可重建、`nvidia-smi`/CUDA 可见、display 环境可用，且非 root 用户可写母工程目录；
- 验证 `reset/step`、候选图、mask、约束匹配和指标的形状；
- 用合成地图/固定小场景检查空候选、不可达和机器人掉线分支；
- 固定随机种子、Python/PyTorch 版本和数据格式。

这一阶段是环境检查，不是基线结果，也不应据此调整方法去追求相对优势。

### E0：无学习数据诊断（1–2 天）

先对合成数据和公开母工程可导出的数据统计；若需要 MASAR 数据，只读取单独导出的 rosbag/JSON，不连接或修改当前容器：

- frontier/coverage 同时存在的比例；
- info gain 的范围、长尾和归一化稳定性；
- coverage task 生命周期、服务时间和 visited 延迟；
- 候选 footprint overlap 与重复派遣比例；
- frontier-only、coverage-only 和混合策略的覆盖差距；
- A* 不可达比例、地图更新频率和控制器状态转移。

若 frontier 与 coverage 几乎不共存，异构图的研究问题就缺少数据支撑，应改成分阶段/单类型消融，而不是伪造异构收益。

### O1：先完成本方法并取得独立结果（约 2–3 周）

只实现 `Hetero-RS` 的最小闭环，不引入正式基线代码：

1. 完成 CandidateGraph、关系类型编码、效用头、可行性 mask、匹配器和 fallback；
2. 在固定训练地图上训练，在 held-out 地图/seed 上评测；
3. 保存 checkpoint、配置、训练曲线、失败 episode、推理延迟和有效分配率；
4. 完成方法自身的绝对验收：无效分配率为零或低于预设安全阈值、任务完成率达到预设阈值、held-out 性能不出现不可接受崩溃、推理延迟满足任务层周期；
5. 冻结模型结构、输入字段、归一化统计量、top-`K`、阈值、fallback 和随机种子列表。

这里不声称“优于某基线”；它回答的是“本方法自己能否正确运行并取得稳定结果”。

### O2：方法冻结后的正式基线比较（约 1 周）

此时才复现 Explore-Bench 母工程原有方法，并在完全相同的地图、候选、停止条件、计算预算和统计协议下运行。Role Selection 只作为可选的外部角色选择参考，不与主母工程同时引入为第二套训练框架：

1. Random feasible；
2. Greedy info gain；
3. Greedy info gain / A* cost；
4. fixed phase switch（先 frontier 后 coverage）；
5. MASAR HIGH_OA（只在后续系统外部验证中运行）。

基线代码和参数不得回写 `rescue-task-allocation` 的已冻结方法配置；比较结果只能新增到 `experiments/baselines/`。

### O3：冻结方法后的消融实验（约 1–2 周）

| 版本 | 新增内容 | 目的 |
| --- | --- | --- |
| Homo-L | 共享节点编码、无类型区分 | 学习母线 |
| Type-L | frontier/coverage 类型 embedding | 类型信息是否有用 |
| TypeLife-L | 类型 + age/service/global phase | 生命周期和阶段是否有用 |
| Hetero-RS | relation-specific encoder + edge mask，学习 HIGH_OA 增量效用 | 核心方法 |
| Hetero-RS − Interaction | 去掉 task-task overlap/conflict 边，但保留 HIGH_OA 原有顺序 overlap discount | 新增交互编码的因果贡献 |
| Hetero-RS − Cost | 去掉 A* cost，仅几何距离 | 路径可达性贡献 |
| Hetero-RS − Fallback | 禁用回退，仅保留模型 | 安全回退的代价与必要性 |

所有版本共享训练预算、候选 top-`K`、优化器和 early stopping 规则。

### O4：MASAR 副本外部验证（约 1–2 周）

先创建 `masar_ws-rescue-task-allocation` 副本和独立容器/Compose project；当前 `masar_ros1_srv-g`、`masar_ros2_bridge-w` 和平台 Run 不停止、不重启、不挂载新代码。先在离线 rosbag/回放中验证输入输出，再在副本 Gazebo/Webots 场景运行：

- HIGH_OA；
- learned adapter；
- learned adapter + forced fallback；
- 空候选、不可达、机器人掉线、服务超时和延迟注入。

完整系统再测试 discovery→`EmergencyTask`→response-group actor→completion，至少保存地图、任务批次、状态转移、策略版本和回退原因。外部验证只回答“接口和系统链是否能接入”，不回头修改已经冻结的方法参数。

## 8. 指标与统计协议

### 8.1 主要指标

- exploration ratio / time-to-90% known area；
- search coverage ratio 和 coverage-task 完成率；
- emergency detection time、首次发现时延和 discovery-to-response handoff time；
- duplicate observation / candidate overlap；
- total path cost、task completion time；
- collision、timeout、unreachable assignment、robot failure recovery；
- role switch count、assignment churn 和平均任务保持时间；
- 参数量、FLOPs、单次推理延迟、CPU/内存占用。

### 8.2 统计规则

- 训练至少 5 个独立 seed，建议 6 个；
- 测试地图和 seed 预先固定并与训练隔离；
- 报告 mean/median、每个 episode 原始点和 95% bootstrap CI；
- 以“策略×地图×seed 的完整 episode”为统计单元，不能把每个机器人时间步当独立样本；
- 预先冻结 top-`K`、置信阈值、回退条件和停止标准；
- 记录失败 episode，不只报告成功运行。

## 9. 最小实现目录与模块边界

建议在 Explore-Bench 母工程 `/home/zjc/workspace/rescue-task-allocation` 的 `chapter4/` 中建立扩展包，不直接改动 `ea_rl/` 的第三章训练代码，也不直接改动 `masar_ws`：

```text
/home/zjc/workspace/rescue-task-allocation/
  chapter4/
    interaction_aware_task_allocator/
    snapshot.py        # offline snapshot -> MASARSnapshot
    candidate_graph.py # node/edge construction and masks
    policy.py          # Homo-L, Type-L, Hetero-RS incremental scorer
    assignment.py      # Hungarian/feasibility/threshold
    fallback.py        # deterministic WAIT fallback in research stage
    adapter_ros.py     # final-stage only: AssignmentDecision -> ROS msg
    replay_or_env.py   # Explore-Bench/Role Selection interface
    metrics.py
  configs/
  data/
  checkpoints/
  experiments/
  tests/
  docker/
    Dockerfile
    compose.yaml
    requirements.lock
```

边界要求：

- `policy.py` 不导入 rospy，不负责发布消息；
- `adapter_ros.py` 不包含训练逻辑，且在 O1–O3 阶段不连接 ROS；
- `fallback.py` 对模型超时、输入过期和空候选提供确定行为；
- 每个决策输出可序列化 JSON，便于第五章平台和论文复盘。

### 容器启动约定（示意）

```yaml
services:
  rescue-task-allocation:
    build: ./docker
    working_dir: /root/workspace/rescue-task-allocation
    network_mode: host
    ipc: host
    gpus: all
    group_add: [video]
    devices:
      - /dev/dri:/dev/dri
    environment:
      DISPLAY: ${DISPLAY:-:0}
      XAUTHORITY: /tmp/.docker.xauth
      NVIDIA_DRIVER_CAPABILITIES: all
    user: "1000:1000"
    volumes:
      - /home/zjc/workspace:/root/workspace:ro
      - /home/zjc/workspace/rescue-task-allocation:/root/workspace/rescue-task-allocation:rw
      - ${HOST_XAUTHORITY:-/home/zjc/.Xauthority}:/tmp/.docker.xauth:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    command: ["bash"]
```

实际使用时应把 UID/GID、基础镜像 tag、PyTorch/CUDA 版本和依赖 hash 固定在 `docker/` 中。需要下载依赖时只在构建阶段启用网络，运行训练容器可保留 host network 以支持 X11，但不启动 ROS 节点、不 source ROS 环境；需要读取 MASAR 导出数据时增加 `:ro` 数据卷，不增加 `masar_ws` 可写卷。容器首次启动必须执行写权限探针：在母工程目录创建临时文件应成功，在 `/root/workspace/masar_ws` 和 `/root/workspace/ROS2swarm_w` 创建临时文件应失败；探针通过后才允许运行训练命令。

## 10. 可复现实验配置最小字段

```yaml
environment:
  name: explore_bench_level0  # or role_selection_coverage
  train_maps: [...]
  test_maps: [...]
  episode_horizon: 500
  seeds: [0, 1, 2, 3, 4, 5]
model:
  variant: hetero_rs
  hidden_dim: 128
  layers: 2
  top_k: 8
  use_task_interaction: true
  use_astar_cost: true
assignment:
  solver: hungarian
  confidence_threshold: 0.0  # freeze before test
  fallback: high_oa
system:
  map_topic: /slam_toolbox/map
  total_area_topic: /sim_map
  frontier_topic: /frontier_filter/filtered_frontiers
  allocation_topic: /task_allocation
```

## 11. 章节写作结构

1. **问题定义**：未知搜救任务层、两类候选、生命周期和约束。
2. **基线与开源母工程**：复现范围、许可证、改动清单和可重复性。
3. **异构任务图建模**：节点/边/特征/可行性 mask。
4. **交互感知效用与分配器**：编码、评分、Hungarian、回退和复杂度。
5. **实验设置**：环境、地图 split、seed、基线、指标和统计。
6. **结果与消融**：先母工程，再 MASAR 外部验证；分别讨论任务层和系统层指标。
7. **接口和系统验证**：消息契约、状态机、延迟/故障测试。
8. **局限性**：coverage 语义差异、仿真偏差、未做端到端联合训练、真实灾害现场未验证。

## 12. 完成判据（Definition of Done）

第四章只有同时满足下列条件才可写成“完成”：

- [ ] E0 证明异构候选和生命周期在数据中确实存在；
- [ ] 至少一个母工程基线能从干净环境复现；
- [ ] Random/Greedy/A*/HIGH_OA 等基线与学习模型使用相同候选和停止条件；
- [ ] Hetero-RS 的关键消融覆盖类型、交互、A* cost 和 fallback；
- [ ] 统计结果包含多 seed、held-out 地图和置信区间；
- [ ] adapter 保持七数组等长的 `/task_allocation` 契约；
- [ ] P0 状态枚举冲突已修正并有生命周期单测；
- [ ] 空候选、不可达、服务超时、地图过期和机器人掉线可安全处理；
- [ ] 至少一个 MASAR 场景完成从任务分配到 coverage/发现回报；
- [ ] 第五章完整搜救至少有一个 discovery→EmergencyTask→response 的可重复闭环，或明确将其列为未完成工作；
- [ ] 论文结论只引用已运行的日志、曲线和回放，不把预期收益写成事实。

## 13. 实施证据索引

为了避免第四章写成脱离平台的抽象算法，以下文件应作为实现和论文附录的固定参照：

| 模块 | 证据文件 | 写作/实现要点 |
| --- | --- | --- |
| 候选生命周期 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_allocator/src/task_allocator/Environment.py` | frontier 每轮重建；coverage 以稳定 id 和 `visited` 保存 |
| 现有分配骨架 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_allocator/src/high_ta.py`、`task_allocator/TA.py` | 保留执行循环和消息发布，只替换效用 scorer |
| 路径和可达性 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_allocator/src/task_allocator/CostCalculator.py` | 使用 A* cost/reachable，避免模型直接输出路径 |
| coverage 任务来源 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_generator/src/robosar_task_generator/task_graph.cpp`、`include/rrt_utils/RRT.h` | 保留 task id、info gain 和重叠过滤 |
| 执行状态机 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_controller/src/lazy_traffic_agent.cpp` | coverage 到点后要等待旋转/搜索完成 |
| 输出契约 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_messages/msg/task_allocation.msg` | 七数组等长；`goal_type=1/2` 分别为 frontier/coverage |
| 状态反馈 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_messages/msg/controller_status.msg`、`task_transmitter/task_listener_robosar_control.py` | 先修复 IDLE/BUSY/SUCCEEDED 语义冲突 |
| 平台消费 | `/home/zjc/workspace/masar-simulation-platform/frontend/src/ros/decoders.ts`、`docs/SPEC.md` | 前端严格校验 allocation 消息，图像只做展示 |
| 第三章响应策略 | `/home/zjc/workspace/ROS2swarm_w/src/ros2swarm/ros2swarm/` | 需 ROS1/ROS2 observation/action/mux adapter，不能零改动替换 |
