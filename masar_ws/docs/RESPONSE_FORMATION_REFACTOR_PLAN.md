# 响应组编队导航重构实施流程

本文只记录仍需维护的落地顺序、模块边界和阶段退出条件。原始需求分析与 1548 行完整实施计划作为历史冻结快照保留在 `docs/superpowers/`；真实改动与运行证据统一写入 `RESPONSE_FORMATION_REFACTOR_PROGRESS.md`。

## 1. 执行原则

1. 每阶段开始前先回看需求、当前实现说明和上一阶段遗留项。
2. 先用测试或运行数据固定根因，再修改最小的权威模块。
3. 新实现具备替代能力后，同阶段切换 launch 并删除冲突旧链，不长期保留 feature flag 双轨。
4. 阶段内完成模块回归；涉及状态切换、控制器或安全边界时必须再运行完整 Tepper。
5. 每阶段在进度日志记录“要改什么、完成什么、结果、未解决什么”。
6. 环境和模型差异进入地图/观测/物理配置，不进入场景坐标或逐案阈值。

## 2. 唯一运行链

```text
GUI mission command
-> emergency task manager / waypoint sequence
-> formation_navigation_node（共享中心规划与连续槽位）
-> response_cmd_mux_node（模式选择、运动门、最终安全）
-> navigation DONE
-> response_task_controller_node（coordinated surround / synchronized transport）
-> task result
-> next task
-> explicit response HOME
```

探索组保留原任务分配、导航和 lazy traffic controller；响应机器人只通过一个渐进的优先权投影影响探索速度。

## 3. 阶段划分与退出条件

### 阶段 1：控制语义和任务目标

- 实现默认关闭的最终运动门。
- STOP 保留任务状态，START 恢复，双组 HOME 独立。
- victim/fire 中心落目标物；box 中心落推送反侧 staging pose。

退出条件：START 前零速、暂停恢复、响应 HOME 和目标几何均有自动测试及系统探针证据。

### 阶段 2：统一编队导航器

- 用一个中心规划器替换四车独立 PlanManager。
- 从栅格净空生成通道中心倾向路径。
- 以连续纵/横尺度和零均值槽位偏移表达形变。
- 地图缺失报错；无直线兜底、比例枚举或第二到达状态机。

退出条件：demo 与 Tepper 都只启动一个导航器；规划、形变、到达、安全 gtest 通过；旧响应专用桥和协调器无引用并删除。

### 阶段 3：任务控制与跨组协同

- surround 使用四车协调循环追踪和自适应半径。
- transport 使用固定推送任务帧、前后排同步对位和实体接触链。
- 最终 mux 区分导航无碰撞与 transport 受控接触。
- 响应组优先，探索组执行非对称让行，消除双方互停。

退出条件：两类控制器 demo 实跑；环绕角进度、运输接触几何和跨组最小距离均可观测。

### 阶段 4：完整长跑闭环

- 干净启动 Tepper，START 前采样零速。
- 运行全部已发现任务，逐项记录 DONE/FAILED、最小距离和任务证据。
- 显式响应 HOME、探索 HOME、最终 STOP。

退出条件：全部任务 DONE、双 HOME 完成、九车最终持续零速，结果写入进度日志。

### 阶段 5：效率与 demo 一致性

- 动态交通消失后重新优化曾经形成的远绕路径。
- 独立 demo 明确打开自身 fixture gate，不等待不存在的外部机器人 TF。
- GUI 保留已分配任务的语义圆点。

退出条件：同一地图回放路径恢复短路线；生产控制器 demo 和全系统使用同一模块。

### 阶段 6：标准主站清洁启动

- Tepper 固定标准 ROS/Gazebo 端口契约。
- 默认启动清理旧 ROS 图和 `/clock`；共享模式只检查。
- 删除完整系统独立端口分支。

退出条件：模拟旧 `/clock` 可被预检清理，ROS1/ROS2 桥仍连接标准图，退出无本次进程残留。

### 阶段 7：探索效率、GUI 和仓库收口

- 保留已验证的 transport `ALIGN/PUSH -> DONE` 主链；推送后 contact release 试验不纳入生产链。
- transport 的 ALIGN 使用由箱体/机器人几何和既有 `robot_gap` 派生的非接触槽位；首次进入仍使用严格容差，进入后只在物理净空内保持位置就绪，PUSH 接触槽位不变。
- `TA_HIGH_OA` 在已过滤的全部 frontier 区域中逐台选择候选，并在选择下一台前扣减重叠信息效用；最后仍由 Hungarian 做机器人—目标匹配，普通 `TA_HIGH` 的局部策略不变。
- 响应机器人不再同时进入探索普通 RVO 和优先权投影；Gazebo 状态快照使用独立锁。
- 优先权约束从最小实体距离到让行半径连续放宽，半径外保持原探索命令。
- GUI 端点使用 frontier/coverage 圆形语义。
- 删除无引用旧链、废弃测试世界和生成缓存；已确认的历史规划文档保留。

退出条件：transport 既有回归、探索分配/RVO 单测、相关节点构建、完整 Tepper 长跑和引用扫描通过。

## 4. 每阶段标准验证

```bash
python3 -m pytest -q -p no:cacheprovider \
  src/robosar/robosar_emergency_response/test

catkin_make -DCATKIN_ENABLE_TESTING=ON \
  rvo_compute_velocity_test \
  clearance_grid_test dynamic_obstacle_planning_test \
  formation_planner_test formation_control_test velocity_safety_test

./run_response_deform_demo.sh --headless --dry-run
./run_response_controller_demo.sh roundflocking --headless --dry-run
./run_response_controller_demo.sh transport --headless --dry-run
./run_tepper_response_demo.sh --headless --no-vglrun --dry-run

git diff --check
```

涉及运行时控制的阶段还应使用验收监视器记录任务、HOME、碰撞和最终零速，不以“Gazebo 看起来能动”代替证据。

## 5. 后续变更判定

- 修复能放入现有模块边界时，扩展当前主链和测试。
- 若方案需要第二规划器、第二状态机、地图坐标特例或多个经验兜底，先回到需求评审，不直接合入。
- 更换底盘或环境时先更新物理标定和地图输入，再判断算法缺陷；不要用新场景阈值掩盖错误几何。
