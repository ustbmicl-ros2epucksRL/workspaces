# 第四章：交互感知异构任务效用学习

本目录是在 Explore-Bench Level-0 母工程上的研究扩展。母工程的网格未知环境、A*、frontier exploration、MAPPO 训练入口和指标代码保留；第四章只在此处增加 Robot–Frontier–Coverage 候选表示、任务生命周期特征、交互感知效用重评分和约束分配接口。

## 研究文档

- [硕士论文整体骨架](/home/zjc/workspace/thesis_overall_framework.md)
- [第四章研究实施方案](./chapter4_interaction_aware_graph_matching_research_plan.md)

## 研发边界

- 研发先在独立 Docker 容器中完成，不连接 ROS，不修改 `masar_ws` 或 `ROS2swarm_w`。
- 方法先独立训练并在 held-out 地图上取得绝对结果，然后冻结结构、配置和 checkpoint。
- 方法冻结后才运行母工程基线和嵌套消融；基线结果不回写方法配置。
- 最后才将冻结的分配器通过 adapter 接入 `masar_ws-rescue-task-allocation` 副本。

## 推荐新增模块

```text
chapter4/
  data_contract.py       # 母工程状态 -> Robot/Frontier/Coverage snapshot
  candidate_graph.py     # 节点、关系、edge mask、top-K
  utility_policy.py      # Type-L / TypeLife-L / Hetero-RS
  assignment.py          # 可行性过滤和 Hungarian/匹配器
  fallback.py            # WAIT/确定性回退
  train_method.py        # O1：只训练本方法
  evaluate_method.py     # O1：held-out 绝对验收
  evaluate_baselines.py  # O2：方法冻结后才运行
  ablations.py           # O3：方法冻结后才运行
  tests/
```

不要在这里导入 `rospy`。MASAR 的 ROS 输入输出只在最终 adapter 中实现，算法模型不得直接输出速度或修改导航控制。

## 母工程来源

- 项目：Explore-Bench
- 论文：ICRA 2022，<https://arxiv.org/abs/2202.11931>
- 来源仓库：<https://github.com/efc-robot/Explore-Bench>
- 固定来源 commit：`ce3b98a09bf593b0716b26555dc4cccbd9e83cd3`
- 当前研究分支：`chapter4/rescue-task-allocation`

保留母工程原始文件，新增和修改内容集中在 `chapter4/` 与明确的训练入口；每次实验记录母工程 commit、代码 diff、容器镜像 tag 和配置 hash。

## 容器启动

容器使用 GPU、X11/display、host IPC、host network、`/dev/dri` 和 `video` 组；workspace 父目录只读，母工程目录读写覆盖。构建和启动：

```bash
docker compose -f /home/zjc/workspace/rescue-task-allocation/docker/compose.yaml build
docker compose -f /home/zjc/workspace/rescue-task-allocation/docker/compose.yaml up -d
docker exec -it rescue-task-allocation bash
cd /root/workspace/rescue-task-allocation
./docker/probe_mounts.sh
```

也可以直接使用与现有 MASAR 启动方式相近的命令封装：

```bash
/home/zjc/workspace/rescue-task-allocation/docker/run.sh
```

该脚本使用 `--gpus all`、`--ipc=host`、`--network=host`、`DISPLAY`、Xauthority、X11 socket、`--group-add video`、`--device=/dev/dri`、`NVIDIA_VISIBLE_DEVICES=all` 和 `NVIDIA_DRIVER_CAPABILITIES=all`，并保留父 workspace 只读、母工程目录读写覆盖。`QT_X11_NO_MITSHM=1` 用于规避容器 GUI 常见的 MIT-SHM 兼容问题。

显示环境按实际会话显式传入。例如本地图形桌面：

```bash
DISPLAY=:0 HOST_XAUTHORITY="$HOME/.Xauthority" \
  /home/zjc/workspace/rescue-task-allocation/docker/run.sh
```

如果使用 TurboVNC/noVNC 对应的宿主 X server，则把 `DISPLAY` 设置为实际的 `:1`、`:101` 等，并把 `HOST_XAUTHORITY` 设置为宿主机上包含该 display cookie 的文件（例如 `/tmp/.docker.xauth`）。当前 shell 没有 `DISPLAY` 时，脚本默认使用 `:0`，但这只代表默认值，不保证该 display 实际存在。

首次启动只验证 GPU/display 和挂载权限，不启动 ROS 节点、不 source `masar_ws/devel/setup.bash`，也不修改当前 MASAR 容器。
