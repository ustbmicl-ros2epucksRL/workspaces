# G 模型

原始训练权重仅保存在 `results/g_study/<formation>/seed_<seed>/`，避免复制多份默认模型。

每次训练的 `actor.pt` 为开发验证最优权重，`final_actor.pt` 是固定500更新终点。已完成三队形全部15次训练及11,040回合独立测试；下列候选及其哈希在独立测试前依据开发验证冻结。

| 队形 | 训练seed | 所选更新 | 原始权重 | 独立TorchScript |
|---|---:|---:|---|---|
| 横排 | 67 | 275 | [actor.pt](../results/g_study/line/seed_67/actor.pt) | [line/actor.ts](deployment/line/actor.ts) |
| 圆形 | 11 | 450 | [actor.pt](../results/g_study/circle/seed_11/actor.pt) | [circle/actor.ts](deployment/circle/actor.ts) |
| 楔形 | 67 | 500 | [actor.pt](../results/g_study/wedge/seed_67/actor.pt) | [wedge/actor.ts](deployment/wedge/actor.ts) |

三份单模型主挑战均288/288成功，平均耗时依次10.10/6.76/7.01 s；横排候选0.20 m压力89/96、联合变化95/96成功，圆形与楔形候选均为全部736/736成功。这是预选单网络结果，不替代每队形五训练seed统计。

[模型身份和测试记录](selected_policies.json)保留所选更新、源文件与导出哈希。完整验证推荐依据位于`results/g_study/recommended_models.json`，测试前文件哈希登记于`study.json`，测试没有更改推荐。

三队形独立TorchScript及控制契约位于 [deployment/](deployment/README.md)。已验证Python/脚本连续动作与记忆误差均为0，隔离解释器可在不导入项目包的情况下推理。当前仅有G，权重和契约schema为 `afor_local_g_v1`；旧模型保存在项目外部归档中。此处是固定五机器人的仿真与导出证据，尚无实机验证。
