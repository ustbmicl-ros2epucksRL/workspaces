#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.

import torch

from vmas import render_interactively
from vmas.simulator.core import Agent, Landmark, Sphere, World
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.utils import Color, ScenarioUtils


# 多智能体收集食物的场景定义
class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        # 读取场景超参数，如果未提供则使用默认值
        n_agents = kwargs.pop("n_agents", 4)
        self.share_reward = kwargs.pop("share_reward", False)       # share_reward: True 时所有智能体共享同一奖励信号，False 时奖励分配给触碰食物的个体
        self.penalise_by_time = kwargs.pop("penalise_by_time", False)   # penalise_by_time: True 时在每个时间步对未得分的环境施加-0.01惩罚，鼓励更快收集
        self.food_radius = kwargs.pop("food_radius", 0.05)      # 食物半径，决定触碰判定范围
        self.pos_range = kwargs.pop("pos_range", 1.0)       # 世界坐标的半边长，实体初始位置与移动边界限定在 [-pos_range, pos_range]
        n_food = kwargs.pop("n_food", n_agents)     # 食物数量，默认与智能体数量相等，可用于控制稀缺程度
        ScenarioUtils.check_kwargs_consumed(kwargs)         # 确保没有未使用的额外参数

        # 构建方形世界，世界中心在(0,0)，边长为2*pos_range
        world = World(
            batch_dim,  # 并行环境个数，影响所有张量的第1维
            device,  # 计算设备（CPU/GPU）
            x_semidim=self.pos_range,  # x轴半径，限制实体x坐标范围
            y_semidim=self.pos_range,  # y轴半径，限制实体y坐标范围
        )
        # 添加指定数量的智能体
        for i in range(n_agents):
            agent = Agent(
                name=f"agent_{i}",  # 唯一名称
                collide=False,  # 不与其他实体碰撞
                shape=Sphere(radius=0.035),  # 使用固定半径的球形
            )
            world.add_agent(agent)  # 将智能体注册到世界
        # 添加食物（地标）
        for i in range(n_food):
            food = Landmark(
                name=f"food_{i}",  # 唯一名称
                collide=False,  # 不产生碰撞
                shape=Sphere(radius=self.food_radius),  # 半径可配
                color=Color.GREEN,  # 渲染颜色为绿色
            )
            world.add_landmark(food)  # 将食物注册到世界

        return world  # 返回构建好的世界

    def reset_world_at(self, env_index: int = None):
        # 将所有智能体位置重置到原点（每个环境独立）
        for agent in self.world.agents:
            agent.set_pos(
                torch.zeros(
                    self.world.dim_p,  # 位置维度(2)
                    device=self.world.device,
                    dtype=torch.float32,
                ),
                batch_index=env_index,  # 指定环境索引（None表示全部环境）
            )
        # 随机初始化所有食物的位置并重置状态
        for landmark in self.world.landmarks:
            landmark.set_pos(
                torch.zeros(
                    (
                        (1, self.world.dim_p)
                        if env_index is not None
                        else (self.world.batch_dim, self.world.dim_p)
                    ),  # 按是否指定env_index决定生成单个或批量位置
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -self.pos_range,  # x/y下界
                    self.pos_range,  # x/y上界
                ),
                batch_index=env_index,  # 仅更新指定环境
            )
            if env_index is None:
                landmark.eaten = torch.full(
                    (self.world.batch_dim,), False, device=self.world.device
                )  # 标记每个环境中食物是否已被吃
                landmark.just_eaten = torch.full(
                    (self.world.batch_dim,), False, device=self.world.device
                )  # 标记当前步是否刚被吃
                landmark.reset_render()  # 重新启用渲染
            else:
                landmark.eaten[env_index] = False  # 单个环境重置为未被吃
                landmark.just_eaten[env_index] = False  # 当前步未被吃
                landmark.is_rendering[env_index] = True  # 重新渲染该环境中的食物

    def reward(self, agent: Agent):
        """
        计算智能体的奖励
        
        场景目标: 多个智能体协作收集食物（landmarks）
        奖励机制: 当智能体接触到未被吃掉的食物时获得奖励
        
        参数:
            agent (Agent): 要计算奖励的智能体
            
        返回:
            torch.Tensor: 奖励值，形状 (batch_dim,)
        """
        is_first = agent == self.world.agents[0]  # 第一个智能体用于聚合计数
        is_last = agent == self.world.agents[-1]  # 最后一个智能体负责更新状态

        rews = torch.zeros(self.world.batch_dim, device=self.world.device)  # 初始化奖励

        # 遍历每个食物，计算当前智能体的贡献
        for landmark in self.world.landmarks:
            if is_first:
                landmark.how_many_on_food = torch.stack(
                    [
                        torch.linalg.vector_norm(
                            a.state.pos - landmark.state.pos, dim=1
                        )
                        < a.shape.radius + landmark.shape.radius  # 判断是否发生接触
                        for a in self.world.agents
                    ],
                    dim=1,
                ).sum(-1)  # 统计每个环境中在该食物上的智能体数量

                landmark.anyone_on_food = landmark.how_many_on_food > 0  # 是否有人触碰
                landmark.just_eaten[landmark.anyone_on_food] = True  # 标记本步被吃

            assert (landmark.how_many_on_food <= len(self.world.agents)).all()  # 安全检查

            if self.share_reward:
                rews[landmark.just_eaten * ~landmark.eaten] += 1  # 被首次吃掉奖励全员
            else:
                on_food = (
                    torch.linalg.vector_norm(
                        agent.state.pos - landmark.state.pos, dim=1
                    )
                    < agent.shape.radius + landmark.shape.radius
                )  # 当前智能体是否触碰该食物

                eating_rew = landmark.how_many_on_food.reciprocal().nan_to_num(
                    posinf=0, neginf=0
                )  # 按人数平分奖励，防止除0
                rews[on_food * ~landmark.eaten] += eating_rew[
                    on_food * ~landmark.eaten
                ]  # 未被吃过且正在触碰时给奖励

            if is_last:
                landmark.eaten += landmark.just_eaten  # 更新累积已吃标志
                landmark.just_eaten[:] = False  # 清空当前步标志
                landmark.is_rendering[landmark.eaten] = False  # 隐藏已吃掉的食物

        if self.penalise_by_time:
            rews[rews == 0] = -0.01  # 未获奖励的环境给时间惩罚
        
        return rews  # 返回每个环境的奖励

    def observation(self, agent: Agent):
        """
        计算智能体的观测
        
        观测内容:
        1. 智能体自身位置 (2维)
        2. 智能体自身速度 (2维)
        3. 每个食物（landmark）的信息 (3维):
           - 食物到智能体的相对位置 (2维)
           - 食物是否已被吃掉 (1维，布尔转整数)
        
        参数:
            agent (Agent): 要计算观测的智能体
            
        返回:
            torch.Tensor: 观测向量，形状 (batch_dim, 4 + 3*n_landmarks)
        """
        obs = []  # 存放每个食物的局部观测
        for landmark in self.world.landmarks:
            obs.append(
                torch.cat(
                    [
                        landmark.state.pos - agent.state.pos,  # 食物相对位置
                        landmark.eaten.to(torch.int).unsqueeze(-1),  # 是否已被吃
                    ],
                    dim=-1,
                )
            )
        
        return torch.cat(
            [
                agent.state.pos,  # 自身位置
                agent.state.vel,  # 自身速度
                *obs,  # 所有食物信息
            ],
            dim=-1,
        )  # 形状: (batch_dim, 4 + 3*n_landmarks)

    def done(self):
        return torch.all(
            torch.stack(
                [landmark.eaten for landmark in self.world.landmarks],  # 收集每个食物的已吃标志
                dim=1,
            ),
            dim=-1,  # 若所有食物都被吃则环境结束
        )


if __name__ == "__main__":
    render_interactively(
        __file__,  # 当前脚本路径
        control_two_agents=True,  # 可用键盘控制两个智能体
        n_agents=4,  # 默认四个智能体
        share_reward=False,  # 默认个体奖励
        penalise_by_tim=False,  # 是否按时间惩罚（拼写与接口保持一致）
    )
