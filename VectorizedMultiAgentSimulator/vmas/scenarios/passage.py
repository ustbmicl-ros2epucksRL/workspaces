#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.

import torch

from vmas import render_interactively
from vmas.simulator.core import Agent, Box, Landmark, Line, Sphere, World
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.utils import Color, ScenarioUtils


class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        self.n_passages = kwargs.pop("n_passages", 1)  # 需要留空的通道数量（不碰撞），决定可通过的缺口个数
        self.shared_reward = kwargs.pop("shared_reward", False)  # True: 全员共享 shaping 奖励；False: 各自独立计算奖励
        ScenarioUtils.check_kwargs_consumed(kwargs)

        assert self.n_passages >= 1 and self.n_passages <= 20

        self.shaping_factor = 100  # potential-based shaping 的缩放系数

        self.n_agents = 5  # 固定智能体数量
        self.agent_radius = 0.03333  # 智能体半径
        self.agent_spacing = 0.1  # 初始化时智能体/目标之间的间距
        self.passage_width = 0.2  # 通道宽度
        self.passage_length = 0.103  # 通道块长度

        self.visualize_semidims = False  # 可视化开关（当前未使用）

        # 构建方形世界，坐标范围 [-1, 1] x [-1, 1]
        world = World(
            batch_dim,  # 并行环境个数
            device,  # 计算设备
            x_semidim=1,  # x 轴半边长
            y_semidim=1,  # y 轴半边长
        )
        # 添加智能体及其目标点
        for i in range(self.n_agents):
            agent = Agent(
                name=f"agent_{i}",  # 唯一名称
                shape=Sphere(self.agent_radius),  # 半径
                u_multiplier=0.7,  # 运动控制增益
            )
            world.add_agent(agent)
            goal = Landmark(
                name=f"goal {i}",  # 目标标记
                collide=False,  # 不参与碰撞
                shape=Sphere(radius=self.agent_radius),  # 半径与智能体一致
                color=Color.LIGHT_GREEN,  # 目标渲染颜色
            )
            agent.goal = goal  # 将目标挂到对应智能体
            world.add_landmark(goal)  # 注册目标
        # 添加通道方块（红色砖块）
        for i in range(
            int((2 * world.x_semidim + 2 * self.agent_radius) // self.passage_length)
        ):
            removed = i < self.n_passages  # True: 该块被“移除”形成缺口且不碰撞；False: 保留为红色障碍并参与碰撞
            passage = Landmark(
                name=f"passage {i}",  # 唯一名称
                collide=not removed,  # True: 作为障碍会碰撞；False: 缺口，不碰撞
                movable=False,  # True: 可移动；False: 固定不动（此处固定）
                shape=Box(length=self.passage_length, width=self.passage_width),  # 方块尺寸
                color=Color.RED,  # 渲染颜色
                collision_filter=lambda e: not isinstance(e.shape, Box),  # 返回 True 时允许碰撞；此处仅与非 Box 形状碰撞
            )
            world.add_landmark(passage)

        return world

    def reset_world_at(self, env_index: int = None):
        # 随机生成中心智能体位置（位于下方，避开通道）
        central_agent_pos = torch.cat(
            [
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -1 + (3 * self.agent_radius + self.agent_spacing),  # x 下界
                    1 - (3 * self.agent_radius + self.agent_spacing),  # x 上界
                ),
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -1 + (3 * self.agent_radius + self.agent_spacing),  # y 下界
                    -(3 * self.agent_radius + self.agent_spacing)
                    - self.passage_width / 2,  # y 上界（位于通道下方）
                ),
            ],
            dim=1,
        )
        # 随机生成中心目标位置（位于上方，穿越通道）
        central_goal_pos = torch.cat(
            [
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -1 + (3 * self.agent_radius + self.agent_spacing),  # x 下界
                    1 - (3 * self.agent_radius + self.agent_spacing),  # x 上界
                ),
                torch.zeros(
                    (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    (3 * self.agent_radius + self.agent_spacing)
                    + self.passage_width / 2,  # y 下界（通道上方）
                    1 - (3 * self.agent_radius + self.agent_spacing),  # y 上界
                ),
            ],
            dim=1,
        )

        # 随机排列智能体与目标，防止策略依赖固定顺序
        order = torch.randperm(self.n_agents).tolist()
        agents = [self.world.agents[i] for i in order]
        goals = [self.world.landmarks[i] for i in order]
        for i, goal in enumerate(goals):
            if i == self.n_agents - 1:
                goal.set_pos(
                    central_goal_pos,
                    batch_index=env_index,
                )
            else:
                goal.set_pos(
                    central_goal_pos  # 其余目标在中心周围按棋盘式偏移
                    + torch.tensor(
                        [
                            [
                                (
                                    0.0
                                    if i % 2
                                    else (
                                        self.agent_spacing
                                        if i == 0
                                        else -self.agent_spacing
                                    )
                                ),
                                (
                                    0.0
                                    if not i % 2
                                    else (
                                        self.agent_spacing
                                        if i == 1
                                        else -self.agent_spacing
                                    )
                                ),
                            ],
                        ],
                        device=self.world.device,
                    ),
                    batch_index=env_index,
                )
        for i, agent in enumerate(agents):
            if i == self.n_agents - 1:
                agent.set_pos(
                    central_agent_pos,
                    batch_index=env_index,
                )
            else:
                agent.set_pos(
                    central_agent_pos
                    + torch.tensor(
                        [
                            [
                                (
                                    0.0
                                    if i % 2
                                    else (
                                        self.agent_spacing
                                        if i == 0
                                        else -self.agent_spacing
                                    )
                                ),
                                (
                                    0.0
                                    if not i % 2
                                    else (
                                        self.agent_spacing
                                        if i == 1
                                        else -self.agent_spacing
                                    )
                                ),
                            ],
                        ],
                        device=self.world.device,
                    ),
                    batch_index=env_index,
                )
            if env_index is None:
                agent.global_shaping = (
                    torch.linalg.vector_norm(
                        agent.state.pos - agent.goal.state.pos, dim=1
                    )
                    * self.shaping_factor  # 记录初始 shaping 距离
                )
            else:
                agent.global_shaping[env_index] = (
                    torch.linalg.vector_norm(
                        agent.state.pos[env_index] - agent.goal.state.pos[env_index]
                    )
                    * self.shaping_factor
                )

        # 随机排列通道块，并为“移除”的通道关闭渲染
        order = torch.randperm(len(self.world.landmarks[self.n_agents :])).tolist()
        passages = [self.world.landmarks[self.n_agents :][i] for i in order]
        for i, passage in enumerate(passages):
            if not passage.collide:
                passage.is_rendering[:] = False
            passage.set_pos(
                torch.tensor(
                    [
                        -1
                        - self.agent_radius
                        + self.passage_length / 2
                        + self.passage_length * i,
                        0.0,
                    ],
                    dtype=torch.float32,
                    device=self.world.device,
                ),
                batch_index=env_index,
            )

    def reward(self, agent: Agent):
        is_first = agent == self.world.agents[0]  # 仅第一个智能体聚合共享奖励

        if self.shared_reward:
            if is_first:
                self.rew = torch.zeros(
                    self.world.batch_dim,
                    device=self.world.device,
                    dtype=torch.float32,
                )
                for a in self.world.agents:
                    dist_to_goal = torch.linalg.vector_norm(
                        a.state.pos - a.goal.state.pos, dim=1
                    )
                    agent_shaping = dist_to_goal * self.shaping_factor  # 当前 shaping
                    self.rew += a.global_shaping - agent_shaping  # shaping 差分为奖励
                    a.global_shaping = agent_shaping  # 更新 shaping 记录
        else:
            self.rew = torch.zeros(
                self.world.batch_dim,
                device=self.world.device,
                dtype=torch.float32,
            )
            dist_to_goal = torch.linalg.vector_norm(
                agent.state.pos - agent.goal.state.pos, dim=1
            )
            agent_shaping = dist_to_goal * self.shaping_factor  # 当前 shaping
            self.rew += agent.global_shaping - agent_shaping  # shaping 差分奖励
            agent.global_shaping = agent_shaping  # 更新记录

        if agent.collide:  # True: 计算碰撞惩罚；False: 跳过碰撞惩罚
            for a in self.world.agents:
                if a != agent:
                    self.rew[self.world.is_overlapping(a, agent)] -= 10  # 与其他智能体碰撞惩罚
            for landmark in self.world.landmarks[self.n_agents :]:
                if landmark.collide:  # True: 障碍参与碰撞惩罚；False: 缺口不罚
                    self.rew[self.world.is_overlapping(agent, landmark)] -= 10  # 撞到通道惩罚

        return self.rew

    def observation(self, agent: Agent):
        # 获取智能体视角下的观测
        passage_obs = []
        passages = self.world.landmarks[self.n_agents :]
        for passage in passages:
            if not passage.collide:
                passage_obs.append(passage.state.pos - agent.state.pos)  # 仅记录缺口位置
        return torch.cat(
            [
                agent.state.pos,  # 自身位置
                agent.state.vel,  # 自身速度
                agent.goal.state.pos - agent.state.pos,  # 目标相对位置
                *passage_obs,
            ],
            dim=-1,
        )

    def done(self):
        return torch.all(
            torch.stack(
                [
                    torch.linalg.vector_norm(a.state.pos - a.goal.state.pos, dim=1)
                    <= a.shape.radius / 2
                    for a in self.world.agents
                ],
                dim=1,
            ),
            dim=1,
        )

    def extra_render(self, env_index: int = 0):
        from vmas.simulator import rendering

        geoms = []
        for i in range(4):
            geom = Line(length=2 + self.agent_radius * 2).get_geometry()
            xform = rendering.Transform()
            geom.add_attr(xform)

            xform.set_translation(
                (
                    0.0
                    if i % 2
                    else (
                        self.world.x_semidim + self.agent_radius
                        if i == 0
                        else -self.world.x_semidim - self.agent_radius
                    )
                ),
                (
                    0.0
                    if not i % 2
                    else (
                        self.world.x_semidim + self.agent_radius
                        if i == 1
                        else -self.world.x_semidim - self.agent_radius
                    )
                ),
            )
            xform.set_rotation(torch.pi / 2 if not i % 2 else 0.0)
            color = Color.BLACK.value
            if isinstance(color, torch.Tensor) and len(color.shape) > 1:
                color = color[env_index]
            geom.set_color(*color)
            geoms.append(geom)
        return geoms


if __name__ == "__main__":
    render_interactively(
        __file__, control_two_agents=True, n_passages=1, shared_reward=False
    )
