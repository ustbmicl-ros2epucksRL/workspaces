# 实物实验指标记录规范

本文用于 Navigation、RoundFlocking 和 Transport 三种策略的 Pi-puck 实物实验。每个任务独立运行 **10--20 次**，建议 20 次；每次改变初始位置和朝向，但固定机器人数量、场地、checkpoint、YAML、代码版本和超时时间。

## 1. 每次运行记录

每次试验填写一行：

| task | trial | success (0/1) | time (s) | peak RSS max (MiB) | inference P95 max (ms) | notes |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Navigation / RoundFlocking / Transport | 1 |  |  |  |  |  |

`success=0` 的试验将 `time` 记为预设超时时间，并在 `notes` 中标记 `timeout`；统计成功试验的平均时间时不包含超时试验。

资源两列填写该次试验所有机器人观测值中的最大值；每台机器人自己的原始值另存。

任务成功和时间的判定沿用论文口径：

| 任务 | 成功条件 | 时间起止 |
| --- | --- | --- |
| RoundFlocking | 最终连续 5 s 内，平均 ring error `<= 0.12 m` 且 angular uniformity `>= 0.70` | START 到满足上述条件的持续区间开始 |
| Transport | 包裹与目标区域重叠（使用论文/当前 YAML 的距离阈值） | START 到首次满足重叠条件 |
| Navigation | 所有机器人均到达各自目标（使用当前 YAML 的目标阈值） | START 到首次全部到达 |

## 2. 两个资源指标

### Peak RSS（MiB）

**含义：** 策略进程运行期间的峰值常驻内存，表示该控制器在 Pi-puck 上实际需要的内存。数值包括 Python/NEAT（或 PyTorch）运行时，但不包括相机、雷达和底盘驱动进程。

**记录方法：** 在任务结束、策略进程仍未退出时，分别在每台 Pi-puck 上找到策略节点 PID，然后执行：

```bash
ps -eo pid,args | rg '[n]avigation_pattern|[r]oundflocking_pattern|[t]ransport_pattern'
```

```bash
PID=<策略节点PID>
awk '/VmHWM:/ {printf "%.2f MiB\n", $2/1024}' /proc/$PID/status
```

`VmHWM` 是 Linux 记录的 peak RSS。每台机器人、每次试验都记录一次；任务汇总取所有机器人和试验中的最大值（最坏情况）。
如果多个试验复用同一进程，`VmHWM` 不会清零；应重启策略节点以获得独立试验值，或将其明确作为整批试验的最大值。

### Actor inference latency P95（ms）

**含义：** 单次网络前向推理耗时的 95 百分位，表示计算负载和控制周期内的实时性。当前控制周期为 0.1 s，因此应检查 P95 是否小于 **100 ms**。

**记录方法：** 在策略节点中仅包围网络调用（`self.network.activate(...)` 或 `policy.act(...)`）计时，不包含观测构造和 ROS 发布：

```python
t0 = time.perf_counter_ns()
output = self.network.activate(observation)  # 或 policy.act(observation)
latency_ms = (time.perf_counter_ns() - t0) / 1e6
```

每次 START 后丢弃前 20 次 warm-up 样本，收集其余控制周期的 `latency_ms`，计算该机器人该次试验的 P95。任务汇总同样取所有机器人和试验中的最大 P95。
在 STOP 时用 `numpy.percentile(latencies_ms, 95)` 计算并保存该次试验的 P95。
循环策略只在每次 START 时 reset，不要在每次推理前 reset。

## 3. 论文汇总

对每个任务报告：

- **Success rate** = 成功次数 / 总试验次数；
- **Completion/formation time** = 成功试验的平均值 `mean +/- std`（与论文一致）；
- **Peak RSS** = 试验中所有 Pi-puck 的最大值；
- **Inference latency P95** = 试验中所有 Pi-puck 的最大 P95。

建议主表列名为：`Success (%)`、`Time (s)`、`Peak RSS (MiB)`、`Inference P95 (ms)`。同时保存每次试验的原始表格，便于复核。
