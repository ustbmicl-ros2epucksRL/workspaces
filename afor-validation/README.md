# G：局部图记忆多机器人编队导航

本项目只实现一种策略 **G：局部图注意力＋GRU＋PPO**。五台机器人共享权重、各自维护记忆，利用本机激光和有限范围邻居状态，在渐缩通道中自主变形、通过窄口、到达各自目标槽位、恢复编队并停稳。

活动代码已移除角色条件化、行为锚定、相关消融与历史兼容分支。横排 `line`、圆形 `circle`、楔形 `wedge` 使用相同 G 架构和训练流程，分别从零训练。actor 为42,845参数；训练时使用独立局部 critic。本次精简与原 G 的初始化、动作、注意力和记忆均已逐位核对，记录见 `results/implementation/g_regression.json`。

**2026-11-05已完成：15次从零训练、每次500更新，以及11,040回合冻结策略独立测试。** 五种子主挑战成功率：横排99.10%±1.51%、圆形100.00%±0.00%、楔形99.79%±0.31%。全部165项实现测试通过，三队形TorchScript与Python动作及记忆误差均为0。完整数据、压力条件和失败记录见 [本轮结果](results/g_study/analysis/RESULTS.md)，候选模型见 [模型索引](models/README.md)。

## 目录

```text
src/afor_local/
  policy.py, critic.py     唯一 G actor 与独立 critic
  modules/graph.py         局部邻居注意力
  environment.py          渐缩通道、物理、奖励与终止
  observation.py          本机传感与邻居关系接口
  formations.py           三种固定任务模板
  train.py, ppo.py         循环 PPO、GAE 与精确续训
  barrier.py              训练用局部制动辅助项
  selection.py            仅用验证集选检查点
  evaluate.py             独立扫掠几何验收与过程指标
  protocols.py, study.py   三队形实验协议与可恢复调度
  checkpoints.py          G 专用权重格式与 TorchScript
  artifacts.py, reporting.py  产物身份与种子级统计
  __main__.py             统一命令行
configs/protocol.json      可机读的实验清单
docs/METHOD.md             方法、奖励、指标及边界
docs/EXPERIMENT_PROTOCOL.md 参考论文依据与本轮实验设计
deployment/               独立本机控制适配器
tests/                    物理、PPO、选型、评测与部署测试
tools/                    训练、实验、导出与绘图便捷入口
results/g_study/           本轮三队形训练和独立测试
models/                   按验证集选择的三队形部署模型
afor/                     原论文参考材料，不属于活动实现
```

## 训练与独立评测

本机 Python 环境：`/home/zjc/anaconda3/envs/PPO-ROBO/bin/python`。也可在 Python≥3.11 环境安装：

```bash
python -m pip install -e '.[dev]'
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python -m pytest -q

# 打印协议，不开始训练
python -m afor_local study

# 完整实验：三队形×五训练种子×500更新
python -m afor_local study --execute --workers 15 --output results/g_study

# 中断后重复同一命令，恢复未完成训练或测试
python tools/run_g_study.py --workers 15 --output results/g_study

# 单次训练；默认每25更新在验证集选best，结束后独立测试
python -m afor_local train --formation circle --seed 11 --updates 500 \
  --output results/g_training/circle/seed_11

# 显式评测一个权重；输出文件必须不存在
python -m afor_local evaluate results/g_study/line/seed_11/actor.pt \
  --formation line --split test --output results/line_recheck.json

python -m afor_local summarize results/g_study \
  --output results/g_study/analysis/report.json \
  --figure results/g_study/analysis/summary.png

# 从已完成实验生成论文表格、固定轨迹及动画
python tools/analyze_g_study.py results/g_study
```

完整实验每次保留 `actor.pt`（验证最优）、`final_actor.pt`（训练终点）、`metrics.jsonl`、`run.json` 和 `evaluation.json`。主研究只独立测试所选模型，不按测试分数倒选检查点或训练种子。训练中只滚动保存一个恢复点，正常完成自动清理。单次训练若需完成后继续扩展，首次加 `--keep-resume`，再以同配置加 `--resume`；扩展预算属于另一实验，不混入固定500更新统计。

## 实验设计与结果口径

参考本地 `afor/public/static/pdf/AFOR.pdf` 及 `supp.pdf` 中的三队形、渐缩通道与队形适应实验；本项目采用更严格的整队成功验收，指标和训练预算与原论文不同，不能称为原论文复现或与其数值直接比较。

| 项目 | 本轮冻结设置 |
|---|---|
| 独立训练 | 每队形5个种子：11、23、37、51、67；各500更新 |
| 训练采样 | 16队×128步/更新，约1.024M团队transition/模型 |
| 训练扰动 | 50%原初态，50%幅度≤0.10 m的独立纵向扰动 |
| 验证选型 | seed172001/172002，四档扰动，每条件24回合，每25更新一次 |
| 独立测试 | seed183001/183002/183003；每模型23条件、736回合 |
| 测试范围 | 主宽1.2 m五档扰动、五种宽度迁移、窄口与扰动联合变化 |
| 主指标 | 全体无碰撞通过、到位、恢复、停稳的团队成功率 |
| 效率与形变 | 失败惩罚耗时、里程、控制努力代理、过程/终态编队误差、恢复时间 |
| 统计单位 | 独立训练种子；报告全部五种子及样本标准差 |

15个所选模型总计11,040个独立测试评估回合；同队形的五模型使用固定配对初态。多个测试回合不增加独立训练种子数。过程固定模板误差不具旋转不变性，控制努力代理不等于实机能耗，仿真成功不提供硬安全保证。

结果与运行状态见 [结果索引](results/README.md)，方法见 [METHOD](docs/METHOD.md)，论文依据、完整矩阵及统计规则见 [实验协议](docs/EXPERIMENT_PROTOCOL.md)。只保留 G 后，这轮实验用于建立可靠的三队形基础结果，不据此预写新机制或优于外部方法的论文结论。

## 历史归档与部署

精简前的完整项目（含原结果、权重、源码、文档和暂存修改）已保存到项目外部：

`/home/zjc/workspace/afor-validation-archive-20261105-g-only/project-before.tar.gz`

归档 SHA-256：`a1dd17fe77fe779e84e5801e8ce4d8950eb8522280115e135a64ccadc1b49a94`。归档含独立的暂存/未暂存补丁；本轮没有改写 Git 暂存区。旧数据不与本轮结果混合。

部署显式选择每种队形的权重，不存在静默加载的旧默认模型：

```bash
python tools/export_local_deployment.py \
  --actor line=/path/to/line/actor.pt \
  --actor circle=/path/to/circle/actor.pt \
  --actor wedge=/path/to/wedge/actor.pt \
  --output /path/to/new_bundle
```

新权重及部署契约统一为 `afor_local_g_v1`。每台机器人仍需一致的任务坐标系、合法传感输入和独立64维记忆。部署接口和校验方法见 [deployment/README.md](deployment/README.md)。
