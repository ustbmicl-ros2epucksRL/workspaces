# G 局部图记忆策略部署

本目录提供独立控制器源码。部署包使用 `afor_local_g_v1` schema、版本1，只接受当前G检查点。为需要的队形明确指定训练后所选 actor，可只导出一种队形，也可重复 `--actor`：

```bash
python tools/export_local_deployment.py \
  --actor line=results/g_study/line/seed_11/actor.pt \
  --output models/deployment_example
python tools/validate_local_deployment.py \
  --bundle models/deployment_example \
  --actor line=results/g_study/line/seed_11/actor.pt \
  --output results/deployment_validation_example.json
```

输出目录必须为空。生成的包包含 `contract.json`、各队形的 `actor.ts`、本文件和 `local_controller.py`。复制整个包即可推理；只依赖 NumPy 和 PyTorch，无需项目包、训练器、critic 或环境。控制器检查 schema、版本、脚本元数据及权重哈希。验证工具逐步比较原 actor 与脚本输出，并在隔离解释器检查独立加载。任务成功率另行记录，导出一致性通过不代表任务性能通过。

## 单机器人接口

```python
from local_controller import LocalController, PeerMeasurement, sample_lidar

controller = LocalController(bundle_directory, formation="line", slot_id=0,
                             mission_yaw_world=0.0)
peers = [PeerMeasurement(slot_id=1,
                         relative_position_world=[0.5, 0.0],
                         relative_velocity_world=[0.0, 0.0], radius=0.13)]
command = controller.step(final_goal_world=assigned_goal_xy,
                          position_world=measured_position_xy,
                          velocity_world=measured_velocity_xy,
                          scan_mission=metric_ranges_32,
                          peers=peers, timestamp=monotonic_time_s)
```

`slot_id` 是固定最终槽位编号0–4，仅供适配器构造名义编队位移和边标记，编号本身不传入网络。目标、位置和速度使用相同世界/里程计轴；位置只用于求最终目标相对向量。邻居提供本机测得或单跳接收的相对位置、相对速度和半径，顺序任意，可为空。控制器仅保留范围内邻居，不接收邻居目标、记忆、地图、通道宽度、阶段或途中航点。

32束激光表示从机器人中心到首个表面的距离，包含其他机器人，单位米。射线在固定mission坐标下从+x起逆时针排列。mission的+y沿训练通道方向，启动时标定后保持不变。`sample_lidar`可将完整车体坐标扫描转到这些方向；非精确对齐时需要不大于1°的束间隔，取相邻束的较小值。合法无回波 `inf` 转量程，NaN和负值拒绝。

按 `contract.json` 的固定周期调用，训练周期为0.05s。加速度每mission轴限制±1m/s²，速度目标按 `clip(v_measured + dt*a, -1, 1)` 计算后转回世界轴。新任务或暂停重启时调用 `controller.reset()`。每台机器人独立维护64维GRU记忆；脚本直接调用为 `(ego, scan, neighbours, mask, state)`，输出 `(acceleration, next_state)`。

该控制器输出全向平面xy指令。高度、姿态、航向及底层跟踪由平台负责；导出一致性验证不代表真实噪声、延迟或三维动力学下的任务性能。
