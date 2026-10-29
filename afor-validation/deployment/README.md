# 原渐缩任务的局部策略部署

使用已训练好的三份 seed=2 最终策略，无需为每个开口重新训练。每种队形选一份 actor，同队五台机器人共享其权重；每台机器人维护自己的 GRU 状态。完整配方在 1.20 m 训练，固定权重在 1.10／1.20／1.30／1.40／1.50／1.60 m 各档、各队形均为 96/96，见[宽度测试](../docs/RESULTS.md)。

可复制整个 [deployment](.) 到目标电脑。它包含三份 `actor.ts`、`contract.json` 和 `local_controller.py`；推理只依赖 NumPy、PyTorch，不需要项目包、训练器、critic、仿真器或几何安全模块。原始 PyTorch 权重及配置仍在 [models](../models/manifest.json)。本次验证环境为 Python 3.11、PyTorch 2.10.0、CPU。

## 必需的任务与测量接口

| 输入 | 单位与含义 |
|---|---|
| formation、role | 启动时选 line／circle／wedge；固定机器人角色 0～4，不能在回合中随意交换 |
| 最终目标、本机位置 | 同一 ENU／里程计坐标系的米制 xy，仅将二者差值送给 actor；也可直接用底层 `pack_observation` 传最终目标相对向量 |
| 本机速度 | 同一世界坐标系下的 xy 速度，m/s |
| 激光 | 32 条从机器人中心到首个表面的距离，含墙和其他机器人，量程 2 m；不减自身半径、不传穿透遮挡后的全地图距离 |
| 单跳邻居 | 仅 2 m 内实际测得／接收到的 role、相对位置、相对速度、半径；邻居顺序任意，可为空 |
| mission_yaw_world | 启动前标定的固定坐标旋转，弧度；局部任务坐标 +y 沿原通道前进方向，+x 朝右 |

固定模板和编队边保存在 `contract.json`；控制器用邻居 role 查静态模板关系。每台机器人只需知道自己的最终槽位，不接收其他机器人的目标。最终目标、固定角色与方向标定属于任务输入；无需开口宽度、全局阶段、入口／出口航点、全地图或环境下发的途中队形。世界绝对位置只在接口侧用来求目标相对向量，不进入网络。

当前策略并未证明任意朝向不变性。固定 mission 坐标用于将同一物理任务的目标、速度、邻居向量和射线统一到训练时的轴向，不能每步把 +y 重新对准当前目标。传感器外参、激光原点和速度坐标均需一致。

## 单机器人控制调用

将这个目录加入 Python 模块路径后，每台机器人创建一个控制器。例如从项目根目录运行以下程序；复制到其他电脑时，将路径改为复制后目录：

```python
from pathlib import Path
import sys
import torch

bundle = Path("deployment").resolve()
sys.path.insert(0, str(bundle))
from local_controller import LocalController, PeerMeasurement, sample_lidar

torch.set_num_threads(1)
controller = LocalController(bundle, formation="line", role=0,
                             mission_yaw_world=0.0)

# 在 20 Hz 的传感／控制回调中运行；下面变量来自本机量测。
scan = sample_lidar(ranges_m, angle_min_rad, angle_increment_rad,
                   body_yaw_world_rad, controller.mission_yaw_world)
peers = [PeerMeasurement(role, relative_position_world_xy,
                         relative_velocity_world_xy, radius=0.13)
         for role, relative_position_world_xy, relative_velocity_world_xy
         in received_local_measurements]
command = controller.step(
    final_goal_world=assigned_final_goal_xy,
    position_world=measured_position_xy,
    velocity_world=measured_velocity_xy,
    scan_mission=scan,
    peers=peers,
    timestamp=monotonic_time_s,
)
# 将 command.velocity_target_world 接到仿真／机器人的平面速度跟踪器。
# 若低层直接接收平面加速度，可使用 command.acceleration_world。
```

传入的激光全周扫描须无重复的 2π 终点，角度从车体 +x 逆时针增加。32 条精确对齐的射线可直接传入 `step`；全周扫描通过 `sample_lidar` 转换到固定 mission 方向。非精确对齐时要求角分辨率不大于 1°，取两侧束最小值；合法无回波 `inf` 转量程，NaN／负距离拒绝。该适配器没有证明真实雷达薄障碍、丢失回波或遮挡误差的鲁棒性。

调用周期固定为 0.05 s（20 Hz）。训练模型使用半径 0.13 m 的圆盘，全向平面加速度各轴限幅 ±1 m/s²、速度各轴限幅 ±1 m/s。速度目标在 **mission 坐标轴**按 `clip(v_measured + 0.05*a, -1, 1)` 计算，再旋转回世界坐标；若平台接口是 body／NED，需在低层边界正确转换。新任务或暂停重启时调用 `controller.reset()`；不能跨回合共享记忆。开始状态和目标槽位应保持原任务分布，最终停稳验收由调用方监测，无需阶段指令。

接口输出没有追加 QP、安全过滤、解析避障或导航控制器。局部制动辅助项只存在于训练阶段。若目标平台实际制动、延迟或速度跟踪明显偏离训练动力学，这个导出并不能补偿这些偏差。

## 三维仿真／实物接入边界

这是一份平面 xy 编队策略。三维仿真可以使用相同渐缩几何和平面传感切片，接收它的 xy 速度目标；无人机的高度 z、姿态与航向由低层控制器维持，墙体与机器人感知需在工作高度对应训练的平面占据。该策略没有 z 动作，不能完成空间三维编队，也没有验证差速底盘直接跟踪这类全向指令的能力。

已验证的迁移工程部分为：三种 actor 的 TorchScript 导出在随机输入 batch=1／5／16 与原模型一致；局部量测适配在三种坐标旋转、平移及邻居重排下与原 actor 匹配；隔离进程只加载 bundle 即可推理。导出模型另在原渐缩环境的 1.10／1.20／1.60 m、每队形每档 16 个新回合运行完整任务，合计 144/144 成功，并逐步比较原 actor 的动作／记忆及独立连续碰撞检查。

这 144 回合是**导出回归**，不加入上面的 1728 回合主宽度统计。没有运行 Gazebo／Isaac／PX4 或真实硬件；尚未验证噪声、延迟和真实动力学下的闭环成功率。有限理想仿真测试也不构成实物安全保证。

## 复核命令

从项目根目录执行，不训练或改变原权重：

```bash
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python tools/validate_local_deployment.py --output runs/deployment_validation.json
python tools/export_local_deployment.py --output runs/deployment_bundle
```

需要先按项目 README 安装 `afor-local`；导出的独立包推理则只需要 NumPy、PyTorch。新导出写到空目录，不覆盖本目录的已交付模型。验证其他导出包时，传 `--bundle runs/deployment_bundle`。

权重 SHA256 在 `contract.json`，原完整闭环记录在 [results/deployment_validation.json](../results/deployment_validation.json)。复核命令会生成新的验证报告，不改写原证据。PyTorch 2.10 对 `torch.jit.load` 提示弃用，当前版本仍通过验证；此包没有验证跨 PyTorch 版本兼容性。
