# 《未知救援环境下多机器人分层协同决策方法与系统研究》论文整体骨架

> 文档用途：作为硕士论文的总纲、章节边界和实施验收基线。
>
> 证据约定：`[已核查]` 表示已从本地代码、论文或仓库文件中确认；`[拟实现]` 表示建议的研究方案；`[待验证]` 表示不能在写作阶段预先宣称的实验结论。
>
> 本文档没有读取 `/tmp/thesis.md`、`/tmp/ch4.md` 等旧产物。本版本以 `ieeeconf-v2/root.tex`、MASAR 工作区、MASAR simulation platform、ROS2swarm 和外部开源仓库的当前内容为依据。

## 1. 先给结论

当前方向可以形成一篇逻辑完整、可落地的硕士论文，但必须把“分层协同”定义为**两个功能机器人组及其任务链之间的分层协同**，而不是把第三章和第四章强行解释成同一策略网络中的上下层控制器：

```text
未知救援环境
    |
    v
搜索发现组（任务层）
    - 建图、frontier/coverage 候选生成
    - 角色/任务选择和多机器人分配（第四章）
    - 发现应急对象并发布 EmergencyTask
    |
    v
任务移交与系统编排（第五章）
    - 记录对象、位置、状态和优先级
    - 选择可用响应机器人
    - 维护可追踪的任务生命周期
    |
    v
应急响应组（执行层）
    - 采用第三章学习得到的协同行为策略
    - 在局部观测和端侧算力受限条件下完成接近、运输、围捕/导航等子任务
    |
    v
闭环验证：环境覆盖、对象发现、任务完成、故障恢复和日志可追溯
```

因此推荐的总科学问题是：

> **在地图逐步揭示、通信和计算受限且任务会动态出现的未知救援环境中，如何让搜索发现组在任务层形成稳定、可解释、可约束的多机器人任务分配，并让响应组在执行层以紧凑的协同行为策略完成移交的应急子任务，从而实现可验证的跨组闭环协同？**

该问题同时包含三个可独立评价、又由系统链路贯通的子问题：

1. **执行层行为学习（第三章）**：局部观测下，多机器人怎样学习紧凑、鲁棒、可部署的协同行为策略？
2. **任务层角色/任务分配（第四章）**：搜索组怎样根据异构候选任务、机器人状态、信息收益、路径代价和任务生命周期进行可行分配？
3. **跨组系统闭环（第五章）**：怎样把发现、移交、响应、状态回报和故障恢复接成可观测的未知救援系统？

这里的“分层”是功能和决策时间尺度的分层：第四章在较慢的任务重规划周期选择“谁做什么”，第三章在较快的控制周期选择“如何运动/协同”。第五章负责跨组事件编排，而不是另外发明一个与前三章无关的算法。

## 2. 论文题目、对象和边界

### 2.1 推荐题目

**中文：**《未知救援环境下多机器人分层协同决策方法与系统研究》

**英文（可选）：** *Hierarchical Cooperative Decision-Making and System for Multi-Robots in Unknown Search-and-Rescue Environments*

题目中的关键词对应关系如下：

| 题目关键词 | 论文中必须兑现的内容 |
| --- | --- |
| 未知救援环境 | 地图、frontier、coverage 候选和应急对象均随时间揭示；不能只在静态已知图上做离线分配 |
| 多机器人 | 每一层内部都是多机器人协同；不能把一组机器人简化成单体 |
| 分层协同决策 | 搜索发现组负责任务层，响应组负责执行层；跨组有明确事件和数据契约 |
| 方法与系统 | 第三、四章给出可重复算法实验，第五章给出可运行平台闭环；系统展示不能代替算法证据 |

### 2.2 不应声称的内容

- 当前代码库中第三章已有论文和实验，但第四章算法尚未实现，不能写成“已完成并优于基线”。
- MASAR simulation platform 的 README/规范明确表明现阶段平台并未默认接入 `multi-agent-neat`；不能声称第三章策略已在第五章系统中完成统一部署。
- 第四章采用 Explore-Bench 或 Role Selection 作为母工程时，母工程的 coverage 语义与 MASAR 的几何 coverage task 不完全等价，不能直接拼接指标或复用原论文数值。
- HGT、Hungarian、PPO、GNN 等是理论/工程组件，不应包装成单独的原创贡献；原创性应放在未知搜救任务上下文、接口约束、动态生命周期和系统验证的组合设计上。

## 3. 章节骨架

### 第一章 绪论

1. 研究背景：未知环境、救援时效、多机器人资源异构和任务动态到达。
2. 问题痛点：单一层次方法无法同时处理“先发现、再响应”；仅靠手工启发式难以解释和迁移；端到端策略难以接入已有搜救平台。
3. 研究目标与科学问题：对应上面的三个子问题。
4. 主要工作和贡献（只列最终被实验支持的内容）。
5. 论文组织。

### 第二章 相关理论与技术基础

建议按问题链组织，而不是按算法名堆砌：

- 多机器人探索、frontier exploration、coverage planning 和信息增益；
- 角色选择、异构多智能体强化学习、图神经网络/异构图注意力；
- 多机器人任务分配、可行性约束、Hungarian/拍卖/路网代价；
- 第三章使用的 NEAT、精英回放、CEM + TD3+BC、循环策略与跨机器人记忆；
- ROS/ROS2、地图、任务消息和仿真到系统的接口设计。

每一节最后说明该技术在本文哪一章被复用，以及复用边界。

### 第三章 局部观测下的多机器人协同行为策略学习（执行层）

**章节问题：**给定任务目标和局部传感器观测，响应组如何学习高效、稳定且能部署到端侧的协同行为？

**已核查事实：** `ieeeconf-v2/root.tex` 描述了 NEAT recurrent topology search、function-preserving/zero-weight topology expansion、elite replay、critic pretraining、CEM + TD3+BC、SSRM（训练阶段的跨机器人和时间聚合）、recurrent sequence replay 与 burn-in；部署时共享 actor 参数、每台机器人保持私有 recurrent state，不交换 hidden state。实验包含 VMAS 的 Flocking、Transport、Navigation，并有 Webots/e-puck/TurtleBot3 部署证据。

**本章输出：**一个响应组行为能力库（actor checkpoint、观测/动作规范、任务适配器和部署限制），供第五章在 `EmergencyTask` 已经确定之后调用。

**边界：**本章不负责搜索 frontier 的全局任务排序，也不负责发现应急对象；它只证明“执行一个已经选定的子任务”时的协同行为能力。

### 第四章 未知搜索环境中的异构任务图角色/任务分配（任务层）

**章节问题：**在地图和候选集合不断变化的情况下，怎样利用 Robot–Frontier–Coverage 的异构关系、信息收益、路径可达性和任务生命周期，选择稳定且可执行的分配？

**建议章节标题：**《面向未知搜救环境的交互感知异构任务图角色分配方法》

**核心方案：**以开源轻量 benchmark 复现作为母线，将机器人节点、frontier 节点和 coverage 节点建成异构候选图；关系类型感知编码器提取“机器人—候选任务”和“候选—候选冗余/互补”信息；对不可达、已分配、过期或状态不合法的边做 mask；策略只负责学习每个机器人—候选任务的效用重评分，最终仍由可解释的约束分配器完成一对一分配；失败时回退到已有 HIGH_OA。这样不会把第四章扩大成路径规划、控制或全局联合优化项目。

**不能写成的版本：**“提出全新的 HGT 网络”“Hungarian 直接优化带交互项的全局目标”或“用一个端到端策略直接输出 ROS 控制”。当前 `TA_HIGH_OA` 已是“顺序候选选择 + 重叠效用折扣 + Hungarian 路程匹配”的启发式基线；本章应准确表述为：在其上学习交互感知的候选效用，并保持对已有导航执行链的兼容。

### 第五章 未知救援环境多机器人分层协同应用系统

本章实现完整闭环：

1. 搜索发现组接收地图和传感器，维护 frontier/coverage 候选；
2. 第四章分配器输出任务批次，沿用 MASAR 的路径规划和控制；
3. 发现应急对象后产生 `EmergencyTask`，由平台记录、展示和移交；
4. 响应组选择第三章能力库中匹配的策略，执行应急子任务；
5. 响应结果、任务状态、探索进度和故障事件回写平台；
6. 以重复场景、丢包/机器人失效和延迟注入验证闭环。

**当前事实：**平台的 exploration profiles、ROS 数据适配、界面和测试已存在；但“第三章策略 + 第四章分配器”统一接入仍是后续工程工作。第五章应明确采用“原有 HIGH_OA → 第四章 adapter → 回退 HIGH_OA”的渐进接入路线。

**执行层适配限制：**第三章 ROS2swarm wrapper 发布的是 `/robot_N/cmd_vel` 一类 ROS2 速度命令，而 RoboSAR 响应组当前使用 `/tb_i/deform_cmd_vel`、`/tb_i/task_cmd_vel` 经 mux 后进入 `/tb_i/cmd_vel`。因此第五章不能声称第三章 actor 可零改动替换响应控制器；至少需要观测适配、动作缩放、任务模式门控和速度 mux 适配。第五章第一阶段可以先用现有响应控制器验证跨组任务闭环，再把第三章 actor 作为可替换执行模块。

**研发环境隔离：**第四章不在当前 MASAR 容器或 `masar_ws` 中开发。推荐将 Explore-Bench 母工程检出并改名为 `/home/zjc/workspace/rescue-task-allocation`，使用 `rescue-task-allocation` Docker 容器；整个 `/home/zjc/workspace` 以只读方式挂载到容器，母工程目录再以 `rw` 覆盖挂载。这样可以读取 MASAR、ROS2swarm、论文和参考仓库，同时使用 `--gpus all`、`DISPLAY`、Xauthority、X11 socket、`--ipc=host`、`--network=host`、`/dev/dri` 和 `video` 组支持 GPU/display；只有母工程目录可写。容器不 source ROS、不启动 ROS 节点，因此 host network 不会自动加入 MASAR 任务流。当前平台的 ROS1/ROS2 业务容器会把整个 workspace 以可写方式挂载到 `/root/workspace`，第四章容器不得复制这个可写边界。方法冻结后，再建立 `masar_ws-rescue-task-allocation` 副本和独立 Compose project 做 adapter/系统验证；当前 `masar_ros1_srv-g`、`masar_ros2_bridge-w` 和现有平台 Run 不停止、不重启、不替换。

### 第六章 总结与展望

- 按三个科学问题逐项总结证据，不把系统可视化当成算法提升；
- 讨论跨组通信、真实机器人、动态障碍、异构传感器和安全约束；
- 给出尚未解决的全局联合训练、在线自适应和 sim-to-real 问题。

## 4. 端到端数据与决策链

```text
OccupancyGrid / robot pose / controller status
        |
        v
MASAR snapshot builder
  - map_seq, timestamp
  - active robots and current task
  - frontier points + info gain
  - stable coverage task id + visited state
  - A* reachability/cost
        |
        v
CandidateGraph (Chapter 4)
  - node types: Robot, Frontier, Coverage
  - edge types: can-reach, assigned-to, spatial-overlap, temporal-conflict
        |
        v
utility policy + feasibility mask + assignment solver
        |
        +--> AssignmentDecision (internal, auditable)
        |       robot_id, task_id, type, score, fallback, version, time
        |
        +--> existing /task_allocation (seven equal-length arrays)
                    |
                    v
             task generator -> RRT/A* -> controller
                    |
                    +--> coverage arrival -> ROTATION/SEARCHING
                    +--> emergency discovery -> EmergencyTask
                                      |
                                      v
                         response-group policy (Chapter 3)
```

关键原则是：第四章只替换“候选任务选择/分配”模块，不替换已有路径规划、coverage 到达后的搜索状态机和 ROS 消息契约；第五章只在上层增加事件编排和策略选择。

## 5. 研究工作之间的理论贯通

### 5.1 统一的状态、动作和约束

每个重规划时刻 `t` 的任务层状态可写为：

```text
S_t = (M_t, R_t, T_t, E_t)
```

其中 `M_t` 是当前可见地图，`R_t` 是机器人状态集合，`T_t` 是 frontier/coverage 候选及其生命周期，`E_t` 是已经发现但尚未完成的应急事件。第四章输出可行分配矩阵 `X_t`，满足：

```text
X[i,j] ∈ {0,1}
sum_j X[i,j] <= 1
sum_i X[i,j] <= 1
```

不可达、已占用、状态不匹配的边设为不可行。第三章接收第五章选定的 `EmergencyTask`、目标类型和局部观测，输出连续动作 `a[i,t]`。因此两章不是同一网络，但共享“任务状态—可行性—执行反馈”的事件链。

### 5.2 两个时间尺度

| 时间尺度 | 决策者 | 输入 | 输出 | 典型验证 |
| --- | --- | --- | --- | --- |
| 任务层（约 1–5 s，按地图/状态事件触发） | 第四章分配器 | 候选图、路径代价、生命周期 | robot→task 分配 | 覆盖率、发现时延、重复区域、重分配稳定性 |
| 执行层（控制周期） | 第三章 actor/控制器 | 局部观测、任务条件、私有 recurrent state | 速度/动作 | 成功率、碰撞、轨迹、响应完成时间 |

时间尺度不应在论文中写成固定常数；实际值由仿真频率、地图更新和控制器约束测量确定。

## 6. 高质量开源工作与复用边界

| 工作 | 论文/仓库 | 可复用内容 | 不可直接复用的部分 |
| --- | --- | --- | --- |
| Explore-Bench（ICRA 2022） | [论文](https://arxiv.org/abs/2202.11931)，[代码](https://github.com/efc-robot/Explore-Bench) | Level-0 网格环境、A*、frontier exploration、MAPPO/on-policy、场景 blueprint、指标框架 | 旧依赖/绝对路径需隔离；其 coverage target 不等于 MASAR 几何 coverage |
| NeuralCoMapping（CVPR 2022） | [论文](https://arxiv.org/abs/2203.16319)，[代码](https://github.com/siyandong/NeuralCoMapping) | Robot/frontier 图输入、self/cross attention、历史信息和图匹配思想 | iGibson 数据、权重和动作定义不能直接用于 MASAR |
| Role Selection | [论文](https://arxiv.org/abs/2312.01747)，[代码](https://github.com/linaug/Role_selection) | 随机 coverage 环境、PPO 角色选择母线、角色稳定性指标 | GPL-3.0；旧 Ray/Python 依赖和小规模代码需先过 reproduction gate |
| Multi-Robot Coverage/Exploration GNN（IROS 2021） | [代码](https://github.com/katetolstaya/graph_rl) | 空间图网络和 coverage/exploration 表征参考 | 仿真和任务定义不同，不能直接宣称复现原结果 |
| Heterogeneous Graph Transformer（WWW 2020） | [代码](https://github.com/acbull/pyHGT) | relation/type-specific attention 的理论依据 | HGT 不是本论文原创，也不应直接照搬其训练任务 |
| ROMA / RODE | [ROMA](https://github.com/TonghanWang/ROMA)，[RODE](https://github.com/TonghanWang/RODE) | 角色分解、角色条件策略和消融设计 | StarCraft 场景与搜救任务不匹配，仅作方法论背景 |
| MAexp | [代码](https://github.com/DuangZhu/MAexp) | 通用多智能体探索训练框架备用基线 | 不与 Explore-Bench、Role Selection 同时做主线，避免扩大复现范围 |
| MASAR 原生 HIGH/PGART 系列 | `arXiv:2303.02259` 及本地 `masar_ws` | frontier/coverage 语义、信息增益、A*、现有回退分配器 | 其启发式不是第四章学习方法，需单独作为系统基线 |

复用策略是“算法母线 + 数据适配 + 小规模可归因改动”，而不是把多个仓库拼成一个未经验证的大系统。

### 6.1 文献质量和许可证处理

- **主实验母线优先选 Explore-Bench**：ICRA 2022 论文和公开代码同时提供环境、算法入口和指标，适合作为可重复起点；仓库依赖较旧，必须在隔离环境中复现并记录补丁。其 README 未给出清晰的顶层许可证时，不应直接把代码复制进论文仓库，建议以外部依赖或仅复用接口/思想为主。
- **NeuralCoMapping 和空间 GNN 用作结构依据**：CVPR 2022/IROS 2021 的论文质量足以支撑 Robot–frontier 图表征，但数据集、动作空间和权重均与 MASAR 不同，只复用建模思想。
- **Role Selection 是角色决策基线而非主要理论依据**：当前可核查的是 arXiv 预印本和 GPL-3.0 代码。若使用其代码，需保留许可证和改动说明；若无法在目标 Python/Ray 环境复现，则只引用角色选择思想，不把它作为主结果来源。
- **HGT、ROMA、RODE 用于理论定位**：它们说明关系类型注意力或角色分解的合理性，但不应被写成搜救场景的直接实验基线，除非另行完成同等环境适配。

论文参考文献中应分别标注“正式会议论文”“预印本”“开源工程”和“本地系统实现”，避免把 GitHub star 数或可运行性当成学术质量证明。

## 7. 主要贡献的合格表述

最终论文至少应从以下候选中选择**有实验支持**的三项：

1. **问题建模贡献**：面向未知搜救任务链，统一表示 Robot–Frontier–Coverage 节点、任务生命周期、信息收益和可达性约束。
2. **方法贡献**：提出交互感知的异构任务图效用编码，并将其接入可解释的一对一可行分配器；核心不是重新发明 HGT，而是搜救任务上下文和约束集成。
3. **实验贡献**：在开源母工程上完成可复现基线、嵌套消融和跨地图测试，并在 MASAR 中验证外部有效性和回退安全性。
4. **系统贡献**：实现发现→移交→响应的两组机器人闭环，提供统一日志、任务状态和故障恢复证据。

如果某项没有完成相应实验，必须降级为“设计方案”或“工程实现”，不能列为已证实贡献。

## 8. 总体实验和证据矩阵

| 证据层 | 最低实验 | 回答的问题 | 不能回答的问题 |
| --- | --- | --- | --- |
| 第三章复核 | VMAS/Webots 原论文设置、部署 smoke test | 行为策略是否可复现/可执行 | 第四章任务分配是否有效 |
| 第四章独立容器 | C0、E0、O1 方法结果、固定 held-out maps | 本方法能否独立运行并取得稳定任务层结果 | 相对基线是否更优（后续 O2） |
| 第四章正式比较 | O2 基线、O3 嵌套消融、多 seed | 方法相对基线和各模块贡献 | MASAR ROS 接口是否无 bug |
| MASAR 外部验证 | HIGH_OA、learned adapter、fallback 三组，重复场景和故障注入 | 方案能否接入真实任务语义和消息链 | 真实灾害现场性能 |
| 完整系统 | discovery→EmergencyTask→response→completion | 两层是否真正闭环、日志是否完整 | 单个模块的因果提升（需看前两层） |

统计最低要求：训练至少 5 个独立 seed（建议 6）；测试使用固定 held-out 地图/种子；报告均值/中位数和 95% bootstrap CI；把“策略在地图上的一次完整 episode”作为统计单元；阈值和回退规则在看结果前冻结。

## 9. 里程碑与最小可毕业路径

| 阶段 | 交付物 | 通过条件 |
| --- | --- | --- |
| M0：数据和接口诊断 | E0 报告、字段映射、状态语义单测 | 能解释 frontier/coverage 共存率、info gain 尺度、地图语义和状态生命周期 |
| M1：rescue-task-allocation 容器 | Explore-Bench 母工程、全 workspace 只读可见、母工程读写、GPU/display 和锁定镜像 | 不启动 ROS、不写 MASAR、容器可重建 |
| M2：本方法独立结果 | Hetero-RS checkpoint、held-out 结果和日志 | 方法自身达到绝对验收阈值，结构/超参数冻结 |
| M3：正式基线 | Random、Greedy/A*、主母工程、HIGH_OA 配置 | 相同候选、停止条件、预算和统计协议 |
| M4：消融 | 类型、生命周期、交互、cost、fallback 消融 | 只改变一个因素，记录完整失败样本 |
| M5：MASAR 副本 adapter | 保持 `/task_allocation` 契约的替换模块 | 不触碰生产容器，离线回放和副本 smoke test 通过 |
| M6：第五章闭环 | EmergencyTask 移交和第三章策略占位/实装 | 至少一个可重复场景完成发现到响应；失败时能回退并记录 |
| M7：写作冻结 | 图表、配置、日志、commit/hash 和限制 | 结论只引用已完成实验，所有未完成内容列为局限 |

若时间紧，优先完成 M0–M4 和一个可靠的 M5 场景；不要为了追求“端到端联合训练”牺牲可复现性。

## 10. 总体风险清单

### P0：必须先解决

0. **环境隔离**：当前 `masar_ros1_srv-g`、`masar_ros2_bridge-w` 均处于 healthy 状态，并把 `/home/zjc/workspace` 以可写卷挂到 `/root/workspace`。第四章开发不得在这些容器内执行，不得直接编辑其挂载路径下的 `masar_ws`，不得通过现有平台 Compose 重建或重启业务容器；开发必须使用独立 `rescue-task-allocation` 容器。该容器将全 workspace 只读挂载，再将 `/home/zjc/workspace/rescue-task-allocation` 单独读写挂载，并用探针确认其他目录写入失败。
1. **控制器状态语义冲突**：`controller_status.msg` 定义 `IDLE=1, BUSY=2, SUCCEEDED=3`，而 `TaskListenerRobosarControl` 内部把 `BUSY=1, IDLE=2, GOAL_SENT=3`，`high_ta.py` 又以 `status == 2` 判断到达。接入前必须统一枚举，或新 adapter 直接判断消息中的 `SUCCEEDED`，并添加 IDLE→BUSY→SUCCEEDED 生命周期单测。
2. **批量消息数组一致性**：`task_allocation.msg` 的 `id/startx/starty/goalx/goaly/goal_type/goal_id` 必须长度完全一致；`mission_executive` 按机器人索引读取，不能只发布部分机器人。
3. **coverage 完成语义**：coverage 到点后不是立即结束，而是 `ROTATION → SEARCHING → ROTATION_COMPLETED`，分配器不能过早重派。

### P1：实验前必须锁定

- `/sim_map` 用于总面积/覆盖率，而主要路径代价来自 `/slam_toolbox/map`（带动态地图和 `/map` 回退），两者不能直接混算。
- coverage task id 来自 task graph（当前实现由 `vertex_id * 10000 + node_id` 构造），应跨地图更新维护 visited/assigned；frontier 每次更新会由 `enumerate` 重排，不能把 frontier 的临时索引当作长期 id。
- `PointArray.infoGain` 已在 frontier filter 中做过阈值/周期内归一化，`high_ta.py` 还会按当前最大值再次归一化；论文中只能把它当作周期内相对收益，不能解释成跨场景绝对信息增益。
- frontier 话题可能暂时为空；策略必须支持等待、空候选和 HIGH_OA 回退。
- `/task_allocation_image` 是低频/latched 可视化输出，不能作为算法输入；算法直接消费结构化任务、地图和状态。
- 母工程 coverage target 与 MASAR coverage 几何搜索候选不同，论文中应分别命名为“benchmark coverage”与“MASAR coverage task”。

## 11. 写作时的判定标准

每个结论都应能回答三句话：

1. **输入是什么？** 地图、候选任务、机器人状态、应急事件或局部观测。
2. **哪一层做了什么？** 第四章选任务，第三章执行行为，第五章编排和记录。
3. **证据在哪里？** 母工程指标、MASAR 日志/回放、部署 smoke test 或系统故障实验。

如果一句话同时混用“探索效率、控制成功率和界面显示”，说明章节边界或实验设计仍不清楚，应在定稿前拆开。

## 12. 核查证据索引

以下是本次分析实际使用的主要本地证据面，便于后续写作和复核：

| 证据 | 路径 | 用途 |
| --- | --- | --- |
| 第三章论文和算法说明 | `ieeeconf-v2/root.tex`、`ieeeconf-v2/references.bib` | 确认 NEAT、SSRM、CEM + TD3+BC、VMAS/Webots 和部署边界 |
| 第三章训练实现 | `ea_rl/`、`experiment/`、`pytorch_neat/`、`environments/vmas/` | 确认 checkpoint、recurrent actor 和训练/评测接口 |
| MASAR 任务状态 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_allocator/src/task_allocator/Environment.py`、`TA.py` | 确认 frontier 重建、coverage 生命周期和 allocator 抽象 |
| MASAR HIGH_OA | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_allocator/src/high_ta.py` | 确认地图、info gain、探索/利用权重、候选选择和输出数组 |
| MASAR 路径代价 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_allocator/src/task_allocator/CostCalculator.py` | 确认 A* cost、可达性和地图来源 |
| MASAR coverage/task graph | `/home/zjc/workspace/masar_ws/src/robosar/robosar_task_generator/src/robosar_task_generator/task_graph.cpp`、`include/rrt_utils/RRT.h` | 确认 coverage id、info gain 和重叠过滤 |
| MASAR 控制器状态机 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_controller/src/lazy_traffic_agent.cpp` | 确认 coverage 到达后的旋转/搜索语义 |
| ROS 消息契约 | `/home/zjc/workspace/masar_ws/src/robosar/robosar_messages/msg/task_allocation.msg`、`controller_status.msg` | 确认数组字段和状态枚举 |
| 平台规范 | `/home/zjc/workspace/masar-simulation-platform/docs/SPEC.md`、`scenarios/`、`frontend/src/ros/decoders.ts` | 确认平台 profile、数据展示和消息校验 |
| 容器与挂载事实 | `/home/zjc/workspace/masar_ws/README.md`、`/home/zjc/workspace/masar-simulation-platform/deploy/compose.yaml` | 确认 `talent18/masar_ros1_srv-g:v2`、workspace bind mount 和生产容器不可直接作为开发环境 |
| 响应组策略适配 | `/home/zjc/workspace/ROS2swarm_w/src/ros2swarm/ros2swarm/` | 确认 ROS2 actor 输入输出与 RoboSAR 响应控制器并非同一话题契约 |
