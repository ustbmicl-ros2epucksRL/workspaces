#  Copyright (c) ProrokLab.
#  版权所有 (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  此源代码的许可证可在根目录的 LICENSE 文件中找到
#  LICENSE file in the root directory of this source tree.

"""
VMAS环境使用示例

这个脚本演示了VMAS环境的基本用法：
1. 如何创建环境
2. 如何生成动作（确定性或随机）
3. 如何执行环境步骤
4. 如何处理观测、奖励和终止标志
5. 如何渲染和保存视频
"""

import random  # Python标准随机库
import time  # 计时

import torch  # PyTorch

from vmas import make_env  # VMAS环境创建函数
from vmas.simulator.core import Agent  # Agent类
from vmas.simulator.utils import save_video  # 视频保存工具


def _get_deterministic_action(agent: Agent, continuous: bool, env):
    """
    获取确定性动作（用于测试）
    
    这个辅助函数生成一个固定的动作，用于演示和测试。
    对于连续动作，返回最小允许动作；对于离散动作，返回动作1。
    
    参数:
        agent: Agent对象，包含动作空间信息
        continuous: 布尔值，指示是连续还是离散动作空间
        env: 环境对象，用于获取batch_dim和device
    
    返回:
        action: 形状为 (batch_dim, action_size) 的动作张量
    """
    if continuous:
        # 连续动作：返回负的最大动作值
        # agent.action.u_range_tensor: 动作范围张量，例如 [1.0, 1.0] 表示每个维度的范围
        # expand(): 扩展到所有环境
        # 结果：所有环境执行相同的动作（例如：全部向左下移动）
        action = -agent.action.u_range_tensor.expand(env.batch_dim, agent.action_size)
    else:
        # 离散动作：返回动作索引1
        # 对于3-way离散化（0:保持, 1:减小, 2:增大），这通常表示"向下"或"减小"
        action = (
            torch.tensor([1], device=env.device, dtype=torch.long)  # 动作索引1
            .unsqueeze(-1)  # 添加维度: [1] -> [[1]]
            .expand(env.batch_dim, 1)  # 扩展到所有环境: (batch_dim, 1)
        )
    
    # clone()确保返回一个独立的副本
    return action.clone()


def use_vmas_env(
    render: bool = False,  # 是否渲染
    save_render: bool = False,  # 是否保存视频
    num_envs: int = 32,  # 并行环境数量
    n_steps: int = 100,  # 运行步数
    random_action: bool = False,  # 是否使用随机动作
    device: str = "cpu",  # 计算设备
    scenario_name: str = "waterfall",  # 场景名称
    continuous_actions: bool = True,  # 是否使用连续动作
    visualize_render: bool = True,  # 渲染时是否显示可视化信息
    dict_spaces: bool = True,  # 是否使用字典形式的观测/奖励
    **kwargs,  # 传递给场景的额外参数
):
    """
    使用VMAS环境的示例函数
    
    这个函数展示了如何：
    - 创建和配置VMAS环境
    - 生成不同类型的动作
    - 执行环境步骤
    - 处理返回值
    - 渲染和保存视频
    
    参数:
        render: 是否渲染场景
        save_render: 是否将渲染保存为视频
        num_envs: 向量化环境的数量
        n_steps: 运行的步数
        random_action: 使用随机动作还是确定性动作
        device: Torch设备（"cpu" 或 "cuda"）
        scenario_name: 场景名称
        continuous_actions: 智能体是否有连续动作
        visualize_render: 是否在渲染时显示可视化（例如：传感器读数）
        dict_spaces: 是否将观测、奖励和信息作为字典返回（默认是列表）
        **kwargs: 传递给场景的关键字参数
    
    返回:
        None（打印性能统计）
    """
    # 参数验证
    assert not (save_render and not render), "要保存视频必须先渲染"

    # ========== 步骤1：创建环境 ==========
    env = make_env(
        scenario=scenario_name,  # 场景名称
        num_envs=num_envs,  # 并行环境数量
        device=device,  # 计算设备
        continuous_actions=continuous_actions,  # 动作类型
        dict_spaces=dict_spaces,  # 返回格式（字典或列表）
        wrapper=None,  # 不使用环境包装器
        seed=None,  # 不设置随机种子（每次运行结果不同）
        # 将所有额外参数传递给场景
        **kwargs,
    )

    # ========== 步骤2：准备 ==========
    frame_list = []  # 存储渲染帧
    init_time = time.time()  # 开始计时
    step = 0  # 步数计数器

    # ========== 步骤3：主循环 ==========
    for _ in range(n_steps):
        step += 1
        print(f"步骤 {step}")

        # ===== 3.1 动作格式选择 =====
        # VMAS支持两种动作输入格式：
        # 1. 列表: [agent0_action, agent1_action, ...]
        # 2. 字典: {"agent_0": action0, "agent_1": action1, ...}
        # 
        # 这里随机选择一种格式，展示两种都可以使用
        dict_actions = random.choice([True, False])

        # 根据选择初始化动作容器
        actions = {} if dict_actions else []
        
        # ===== 3.2 为每个智能体生成动作 =====
        for agent in env.agents:
            if not random_action:
                # 使用确定性动作（用于测试和演示）
                action = _get_deterministic_action(agent, continuous_actions, env)
            else:
                # 使用随机动作
                # get_random_action()根据动作空间类型自动生成合适的随机动作
                action = env.get_random_action(agent)
            
            # 将动作添加到容器中
            if dict_actions:
                # 字典格式：使用智能体名称作为键
                actions.update({agent.name: action})
            else:
                # 列表格式：按顺序添加
                actions.append(action)

        # ===== 3.3 执行环境步骤 =====
        # step()方法接受动作并返回四个值
        obs, rews, dones, info = env.step(actions)
        
        # 返回值说明（取决于dict_spaces设置）：
        #
        # 如果 dict_spaces=True（字典格式）：
        #   obs:  {"agent_0": tensor(batch_dim, obs_dim), ...}
        #   rews: {"agent_0": tensor(batch_dim,), ...}
        #   info: {"agent_0": {...}, ...}
        #
        # 如果 dict_spaces=False（列表格式）：
        #   obs:  [tensor(batch_dim, obs_dim), ...]
        #   rews: [tensor(batch_dim,), ...]
        #   info: [{...}, ...]
        #
        # dones: tensor(batch_dim,) - 总是张量，表示每个环境是否终止

        # ===== 3.4 渲染（如果启用） =====
        if render:
            # 渲染当前状态
            frame = env.render(
                mode="rgb_array",  # 返回RGB数组
                agent_index_focus=None,  # 相机不跟随特定智能体（可以设置为0, 1等）
                visualize_when_rgb=visualize_render,  # 是否显示额外的可视化信息
            )
            
            # 如果需要保存，添加到帧列表
            if save_render:
                frame_list.append(frame)

    # ========== 步骤4：统计和保存 ==========
    # 计算总耗时
    total_time = time.time() - init_time
    
    # 打印性能统计
    print(
        f"耗时: {total_time}秒，"
        f"运行了 {n_steps} 步，"
        f"共 {num_envs} 个并行环境，"
        f"设备: {device}，"
        f"场景: {scenario_name}"
    )
    """
    print(
        f"It took: {total_time}s for {n_steps} steps of {num_envs} parallel environments on device {device} "
        f"for {scenario_name} scenario."
    )
    """

    # 保存视频（如果启用）
    if render and save_render:
        # fps = 1 / dt，其中dt是仿真时间步长
        save_video(scenario_name, frame_list, fps=1 / env.scenario.world.dt)


# ========== 主程序入口 ==========
if __name__ == "__main__":
    # 运行示例
    use_vmas_env(
        scenario_name="waterfall",  # waterfall场景（智能体避开下落的物体）
        render=True,  # 显示渲染
        save_render=False,  # 不保存视频
        random_action=False,  # 使用确定性动作（所有智能体向下移动）
        continuous_actions=False,  # 使用离散动作空间
        # 场景特定参数
        n_agents=4,  # 4个智能体
    )


"""
常见场景:
- "waterfall": 智能体避开下落物体
- "transport": 协作运输重物
- "navigation": 导航到目标
- "balance": 平衡任务
- "wind_flocking": 在风中保持队形
"""

