# G 三队形实验结果

本目录只保存 G 的实现核验和本轮重新训练的数据。旧结果已完整归档到项目外部，参见主README。

## 已完成的三队形实验（2026-11-05）

15次训练及11,040回合测试全部完成，源码/场景/权重/逐回合计数核验通过。每队形五个训练种子，每次500更新；只用验证集选择检查点，冻结全部模型后再开展独立测试。

| 队形 | 常规成功率 | 主挑战成功率（五seed均值±样本标准差） | 0.20 m压力 | 主挑战惩罚耗时/s |
|---|---:|---:|---:|---:|
| 横排 line | 100% | 99.10% ± 1.51% | 92.71% ± 7.55% | 12.12 ± 3.56 |
| 圆形 circle | 100% | 100.00% ± 0.00% | 100.00% ± 0.00% | 6.83 ± 0.06 |
| 楔形 wedge | 100% | 99.79% ± 0.31% | 98.33% ± 2.72% | 7.17 ± 0.15 |

主挑战是宽1.2 m、纵向扰动0.05/0.10/0.15 m；失败在惩罚耗时中按37.5 s计。主挑战成功数依次为1427/1440、1440/1440、1437/1440。无扰动宽度迁移三队形均800/800；宽1.1 m与扰动0.15 m联合变化依次460/480、478/480、471/480。

全部条件合计10,950成功，90失败（88机间碰撞、1墙撞、1超时）。不同测试家族分别报告，不用总成功率替代主指标；三队形几何不同，时间差不是同一任务下算法优劣的证明。

[完整逐seed表与指标解释](g_study/analysis/RESULTS.md) · [机器可读汇总](g_study/analysis/report.json) · [完整性核验](g_study/analysis/verification.json) · [训练与鲁棒性图](g_study/analysis/summary.png) · [固定轨迹](g_study/analysis/trajectories.png) · [固定动画](g_study/analysis/formation_navigation.gif)

## 产物索引

- `implementation/g_regression.json`：精简前后原G参数、初始化和连续输出的逐位比较。
- `g_study/study.json`：训练前登记、源码身份和各任务运行状态。
- `g_study/source_snapshot.zip`：本轮冻结源码、协议和方法说明。
- `g_study/cases/`：在训练前生成并登记哈希的三队形配对初态。
- `g_study/<formation>/seed_<seed>/`：验证所选及训练终点模型、逐更新日志、身份和原始独立评测。
- `g_study/analysis/report.json`：完整性审计与按独立训练种子统计的结果；`status=complete`才表示全轮完成。
- `g_study/analysis/summary.png`、`.pdf`：训练、扰动和耗时图。
- `g_study/analysis/RESULTS.md`：从原始结果生成的论文用表格和局限说明。
- `g_study/analysis/trajectories.png`、`.pdf`及`formation_navigation.gif`：固定分析场景，未根据成功与否换例。
- `g_study/analysis/verification.json`：全部训练预算、初始化、权重、场景与原始计数的核验。
- `g_study/recommended_models.json`：每种队形仅依开发验证分数选择的部署候选。

实现回归165项通过，记录于`implementation/verification.json`；独立控制包的连续Python/TorchScript输出误差均为0，记录于`implementation/deployment_validation.json`。原G与精简G在相同种子前三次真实PPO更新后的全部actor权重逐位一致。论文实验依据及结论边界见 [实验协议](../docs/EXPERIMENT_PROTOCOL.md)。
