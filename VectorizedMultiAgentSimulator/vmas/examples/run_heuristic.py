#  Copyright (c) ProrokLab.
#  版权所有 (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  此源代码的许可证可在根目录的 LICENSE 文件中找到
#  LICENSE file in the root directory of this source tree.

"""
运行启发式策略的示例脚本

这个脚本演示如何：
1. 创建VMAS环境
2. 使用启发式策略控制智能体
3. 收集并统计奖励
4. 可选地渲染和保存视频
"""

import time  # 用于计时
from typing import Type  # 类型提示

import torch  # PyTorch库

from vmas import make_env  # VMAS环境创建函数
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy, RandomPolicy  # 启发式策略类
from vmas.simulator.utils import save_video  # 视频保存工具


def run_heuristic(
    scenario_name: str,  # 场景名称，例如 "transport", "navigation"
    heuristic: Type[BaseHeuristicPolicy] = RandomPolicy,  # 启发式策略类（不是实例）
    n_steps: int = 200,  # 运行的步数
    n_envs: int = 32,  # 并行环境的数量
    env_kwargs: dict = None,  # 传递给场景的额外参数
    render: bool = False,  # 是否渲染
    save_render: bool = False,  # 是否保存渲染视频
    device: str = "cpu",  # 计算设备："cpu" 或 "cuda"
):
    """
    使用启发式策略运行VMAS场景
    
    参数:
        scenario_name: 场景名称字符串
        heuristic: 启发式策略类（例如 RandomPolicy 或自定义策略）
        n_steps: 仿真步数
        n_envs: 并行仿真的环境数量（向量化批处理）
        env_kwargs: 传递给场景的字典参数（例如 {"n_agents": 4}）
        render: 是否显示渲染画面
        save_render: 是否保存渲染为视频文件
        device: PyTorch计算设备
    
    功能:
        1. 创建向量化环境
        2. 实例化启发式策略
        3. 在多个并行环境中运行策略
        4. 收集性能统计信息
        5. 可选地保存渲染视频
    """
    # 参数验证：如果要保存视频，必须先渲染
    assert not (save_render and not render), "要保存视频，必须启用渲染（render=True）"
    
    # 如果没有提供环境参数，初始化为空字典
    if env_kwargs is None:
        env_kwargs = {}
        """
        env_kwargs = {"n_agents": 8,
                      "n_packages": 1,
                      "package_width": 0.15,
                      "package_length": 0.15,
                      "package_mass": 50}
        """

    # ========== 步骤1：实例化启发式策略 ==========
    # 创建策略对象，设置为连续动作模式
    policy = heuristic(continuous_action=True)

    # ========== 步骤2：创建环境 ==========
    env = make_env(
        scenario=scenario_name,  # 场景名称
        num_envs=n_envs,  # 并行环境数量
        device=device,  # 计算设备
        continuous_actions=True,  # 使用连续动作空间（而非离散）
        wrapper=None,  # 不使用额外的环境包装器
        # 将所有额外参数传递给场景
        **env_kwargs,
    )

    # ========== 步骤3：准备渲染和统计 ==========
    frame_list = []  # 存储渲染帧，用于创建GIF或视频
    init_time = time.time()  # 记录开始时间
    step = 0  # 步数计数器
    
    # 重置环境，获取初始观测
    # obs是一个列表，每个元素对应一个智能体的观测
    # 每个观测的形状: (n_envs, obs_dim)
    obs = env.reset()
    #print(f"Initial observation: {obs}")
    
    total_reward = 0  # 累计总奖励

    # ========== 步骤4：主循环 ==========
    for _ in range(n_steps):
        step += 1
        
        # ===== 4.1 为每个智能体计算动作 =====
        # 创建动作列表，初始化为None
        actions = [None] * len(obs)
        
        # 遍历所有智能体的观测
        for i in range(len(obs)):
            # 使用策略计算动作
            # obs[i]: 第i个智能体在所有环境中的观测 (n_envs, obs_dim)
            # u_range: 动作的允许范围（例如 [-1, 1]）
            # 返回: (n_envs, action_dim) 的动作张量
            actions[i] = policy.compute_action(
                obs[i], 
                u_range=env.agents[i].u_range
            )
        
        # ===== 4.2 执行动作，获取下一步状态 =====
        # step()返回四个值：
        # - obs: 新观测列表
        # - rews: 奖励列表，每个元素形状 (n_envs,)
        # - dones: 终止标志，形状 (n_envs,)
        # - info: 额外信息字典列表
        obs, rews, dones, info = env.step(actions)
        #print(f"Step {step} - Rewards: {rews}")#Step 200 - Rewards: [tensor([0.1047, 0.1030]), tensor([0.1047, 0.1030]), tensor([0.1047, 0.1030]), tensor([0.1047, 0.1030])]           Tensor代表该 agent 在每个并行环境实例上的奖励
        
        # ===== 4.3 计算和累积奖励 =====
        # 将奖励列表堆叠成张量
        # rewards 形状: (n_envs, n_agents)
        rewards = torch.stack(rews, dim=1)
        #print(f"Step {step} - Stacked Rewards: {rewards}")#Step 200 - Stacked Rewards: tensor([[0.1047, 0.1047, 0.1047, 0.1047],[0.1030, 0.1030, 0.1030, 0.1030]])
        
        # 计算每个环境的全局奖励（所有智能体的平均）
        # global_reward 形状: (n_envs,)
        global_reward = rewards.mean(dim=1)
        #print(f"Step {step} - Global Rewards: {global_reward}")#Step 200 - Global Rewards: tensor([0.1047, 0.1030])
        # 计算所有环境的平均奖励（标量）
        mean_global_reward = global_reward.mean(dim=0)
        
        # 累积奖励
        total_reward += mean_global_reward
        
        # ===== 4.4 渲染（如果启用） =====
        if render:
            # 渲染当前帧
            # mode="rgb_array": 返回numpy数组而不是显示窗口
            # agent_index_focus=None: 相机不跟随特定智能体
            # visualize_when_rgb=True: 在RGB模式下显示可视化信息
            frame_list.append(
                env.render(
                    mode="rgb_array",
                    agent_index_focus=None,
                    visualize_when_rgb=True,
                )
            )

    # ========== 步骤5：统计和报告 ==========
    # 计算总耗时
    total_time = time.time() - init_time
    
    # 如果需要保存视频
    if render and save_render:
        # fps = 1 / dt，dt是仿真时间步长
        save_video(scenario_name, frame_list, 1 / env.scenario.world.dt)

    # 打印性能统计
    print(
        f"耗时: {total_time}秒，运行了 {n_steps} 步，"
        f"共 {n_envs} 个并行环境，设备: {device}\n"
        f"平均总奖励: {total_reward}"
    )
    """
    print(
        f"It took: {total_time}s for {n_steps} steps of {n_envs} parallel environments on device {device}\n"
        f"The average total reward was {total_reward}"
    )
    """


# ========== 主程序入口 ==========
if __name__ == "__main__":
    # 导入transport场景的启发式策略
    from vmas.scenarios.transport import HeuristicPolicy as TransportHeuristic

    # 运行示例
    run_heuristic(
        scenario_name="transport",  # 使用transport场景
        heuristic=TransportHeuristic,  # 使用transport的专用策略
        n_envs=300,  # 300个并行环境（大规模向量化）
        n_steps=1200,  # 运行200步
        render=True,  # 显示渲染
        save_render=False,  # 不保存视频（节省磁盘空间）
    )