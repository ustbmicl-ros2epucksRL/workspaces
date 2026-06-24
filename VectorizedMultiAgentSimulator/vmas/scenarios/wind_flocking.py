#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.
import typing
from typing import Dict, List

import torch
from torch import Tensor

from vmas import render_interactively
from vmas.simulator.controllers.velocity_controller import VelocityController
from vmas.simulator.core import Agent, Sphere, World
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.utils import Color, ScenarioUtils, X, Y

if typing.TYPE_CHECKING:
    from vmas.simulator.rendering import Geom


def angle_to_vector(angle):
    return torch.cat([torch.cos(angle), torch.sin(angle)], dim=1)


def get_line_angle_0_90(rot: Tensor):
    angle = torch.abs(rot) % torch.pi
    other_angle = torch.pi - angle
    return torch.minimum(angle, other_angle)


def get_line_angle_0_180(rot):
    angle = rot % torch.pi
    return angle


def get_line_angle_dist_0_360(angle, goal):
    angle = angle_to_vector(angle)
    goal = angle_to_vector(goal)
    return -torch.einsum("bs,bs->b", angle, goal)


def get_line_angle_dist_0_180(angle, goal):
    angle = get_line_angle_0_180(angle)
    goal = get_line_angle_0_180(goal)
    return torch.minimum(
        (angle - goal).abs(),
        torch.minimum(
            (angle - (goal - torch.pi)).abs(),
            ((angle - torch.pi) - goal).abs(),
        ),
    ).squeeze(-1)


class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        self.plot_grid = True
        self.viewer_zoom = 2

        # Reward 奖励权重
        self.vel_shaping_factor = kwargs.pop("vel_shaping_factor", 1)  # 速度贴近期望速度的权重
        self.dist_shaping_factor = kwargs.pop("dist_shaping_factor", 1)  # 维持两机期望间距的权重
        self.wind_shaping_factor = kwargs.pop("wind_shaping_factor", 1)  # 抵消风/重力的权重

        self.pos_shaping_factor = kwargs.pop("pos_shaping_factor", 0)  # 朝目标高度推进的权重
        self.rot_shaping_factor = kwargs.pop("rot_shaping_factor", 0)  # 线段朝向的权重
        self.energy_shaping_factor = kwargs.pop("energy_shaping_factor", 0)  # 控制能耗惩罚权重

        # Observation 观测开关
        self.observe_rel_pos = kwargs.pop("observe_rel_pos", False)  # 是否加入他人相对位置
        self.observe_rel_vel = kwargs.pop("observe_rel_vel", False)  # 是否加入他人相对速度
        self.observe_pos = kwargs.pop("observe_pos", True)  # 是否包含自身绝对位置

        # Controller 控制与动力学
        self.use_controller = kwargs.pop("use_controller", True)  # 是否使用内置速度控制器（否则直接力控制）
        self.wind = torch.tensor(
            [0, -kwargs.pop("wind", 2)], device=device, dtype=torch.float32
        ).expand(batch_dim, 2)  # 向下的风（作为重力）
        self.v_range = kwargs.pop("v_range", 0.5)  # 速度控制模式下的速度上限
        self.desired_vel = kwargs.pop("desired_vel", self.v_range)  # 期望速度模长，方向默认+Y
        self.f_range = kwargs.pop("f_range", 100)  # 力控制模式下的力上限

        controller_params = [1.5, 0.6, 0.002]  # 速度控制器的 PID 参数
        self.u_range = self.v_range if self.use_controller else self.f_range  # 暴露给策略的动作幅度

        # Other 其他场景参数
        self.cover_angle_tolerance = kwargs.pop("cover_angle_tolerance", 1)  # 风缩放的角度容差
        self.horizon = kwargs.pop("horizon", 200)  # 单轮最大步数
        ScenarioUtils.check_kwargs_consumed(kwargs)

        self.desired_distance = 1
        self.grid_spacing = self.desired_distance

        # Make world
        world = World(batch_dim, device, drag=0, linear_friction=0.1)

        self.desired_vel = torch.tensor(
            [0.0, self.desired_vel], device=device, dtype=torch.float32
        )
        self.max_pos = (self.horizon * world.dt) * self.desired_vel[Y]
        self.desired_pos = 10.0
        self.n_agents = 2

        # Add agents
        self.big_agent = Agent(
            name="agent_0",
            render_action=True,
            shape=Sphere(radius=0.05),
            u_range=self.u_range,
            v_range=self.v_range,
            f_range=self.f_range,
            gravity=self.wind,
        )
        self.big_agent.controller = VelocityController(
            self.big_agent, world, controller_params, "standard"
        )
        world.add_agent(self.big_agent)

        self.small_agent = Agent(
            name="agent_1",
            render_action=True,
            shape=Sphere(radius=0.03),
            u_range=self.u_range,
            v_range=self.v_range,
            f_range=self.f_range,
            gravity=self.wind,
        )
        self.small_agent.controller = VelocityController(
            self.small_agent, world, controller_params, "standard"
        )
        world.add_agent(self.small_agent)

        for agent in world.agents:
            agent.wind_rew = torch.zeros(batch_dim, device=device)
            agent.vel_rew = agent.wind_rew.clone()
            agent.energy_rew = agent.wind_rew.clone()

        self.dist_rew = torch.zeros(batch_dim, device=device)
        self.rot_rew = self.dist_rew.clone()
        self.vel_reward = self.dist_rew.clone()
        self.pos_rew = self.dist_rew.clone()
        self.t = self.dist_rew.clone()

        return world

    def set_wind(self, wind):
        self.wind = torch.tensor(
            [0, -wind], device=self.world.device, dtype=torch.float32
        ).expand(self.world.batch_dim, self.world.dim_p)
        self.big_agent.gravity = self.wind
        self.small_agent.gravity = self.wind

    def reset_world_at(self, env_index: int = None):
        start_angle = torch.zeros(
            (1, 1) if env_index is not None else (self.world.batch_dim, 1),
            device=self.world.device,
            dtype=torch.float32,
        ).uniform_(
            -torch.pi / 8,
            torch.pi / 8,
        )

        start_delta_x = (self.desired_distance / 2) * torch.cos(start_angle)
        start_delta_y = (self.desired_distance / 2) * torch.sin(start_angle)

        order = torch.randperm(self.n_agents).tolist()
        agents = [self.world.agents[i] for i in order]
        for i, agent in enumerate(agents):
            agent.controller.reset(env_index)
            if i == 0:
                agent.set_pos(
                    -torch.cat([start_delta_x, start_delta_y], dim=1),
                    batch_index=env_index,
                )
            else:
                agent.set_pos(
                    torch.cat([start_delta_x, start_delta_y], dim=1),
                    batch_index=env_index,
                )

            if env_index is None:
                agent.vel_shaping = (
                    torch.linalg.vector_norm(agent.state.vel - self.desired_vel, dim=-1)
                    * self.vel_shaping_factor
                )
                agent.energy_shaping = torch.zeros(
                    self.world.batch_dim,
                    device=self.world.device,
                    dtype=torch.float32,
                )
                agent.wind_shaping = (
                    torch.linalg.vector_norm(agent.gravity, dim=-1)
                    * self.wind_shaping_factor
                )

            else:
                agent.vel_shaping[env_index] = (
                    torch.linalg.vector_norm(
                        agent.state.vel[env_index] - self.desired_vel
                    )
                    * self.vel_shaping_factor
                )
                agent.energy_shaping[env_index] = 0
                agent.wind_shaping[env_index] = (
                    torch.linalg.vector_norm(agent.gravity[env_index])
                    * self.wind_shaping_factor
                )

        if env_index is None:
            self.t = torch.zeros(
                self.world.batch_dim, device=self.world.device, dtype=torch.int
            )
            self.distance_shaping = (
                torch.linalg.vector_norm(
                    self.small_agent.state.pos - self.big_agent.state.pos,
                    dim=-1,
                )
                - self.desired_distance
            ).abs() * self.dist_shaping_factor

            self.pos_shaping = (
                (
                    torch.maximum(
                        self.big_agent.state.pos[:, Y],
                        self.small_agent.state.pos[:, Y],
                    )
                    - self.desired_pos
                ).abs()
            ) * self.pos_shaping_factor

            self.rot_shaping = (
                get_line_angle_dist_0_180(self.get_agents_angle(), 0)
                * self.rot_shaping_factor
            )

        else:
            self.t[env_index] = 0
            self.distance_shaping[env_index] = (
                torch.linalg.vector_norm(
                    self.small_agent.state.pos[env_index]
                    - self.big_agent.state.pos[env_index]
                )
                - self.desired_distance
            ).abs() * self.dist_shaping_factor

            self.pos_shaping[env_index] = (
                (
                    torch.maximum(
                        self.big_agent.state.pos[env_index, Y],
                        self.small_agent.state.pos[env_index, Y],
                    )
                    - self.desired_pos
                ).abs()
            ) * self.pos_shaping_factor

            self.rot_shaping[env_index] = (
                get_line_angle_dist_0_180(self.get_agents_angle()[env_index], 0)
                * self.rot_shaping_factor
            )

    def process_action(self, agent: Agent):
        if self.use_controller:
            agent.controller.process_force()

    def reward(self, agent: Agent):
        is_first = agent == self.world.agents[0]  # 只在第一个智能体上做全局奖励计算

        if is_first:
            self.t += 1  # 步数计数
            self.set_friction()  # 根据朝向调整风（重力）

            # Dist reward：保持两机间距接近期望值
            distance_shaping = (
                torch.linalg.vector_norm(
                    self.small_agent.state.pos - self.big_agent.state.pos,
                    dim=-1,
                )
                - self.desired_distance
            ).abs() * self.dist_shaping_factor  # 当前势能
            self.dist_rew = self.distance_shaping - distance_shaping  # 势能下降即正奖励
            self.distance_shaping = distance_shaping  # 更新势能

            # Rot shaping：连线朝向接近目标方向
            rot_shaping = (
                get_line_angle_dist_0_180(
                    self.get_agents_angle(),
                    0,
                )
                * self.rot_shaping_factor
            )
            self.rot_rew = self.rot_shaping - rot_shaping  # 角度偏差减小为正
            self.rot_shaping = rot_shaping

            # Pos reward：高度推进到期望高度
            pos_shaping = (
                (
                    torch.maximum(
                        self.big_agent.state.pos[:, Y],
                        self.small_agent.state.pos[:, Y],
                    )
                    - self.desired_pos
                ).abs()
            ) * self.pos_shaping_factor
            self.pos_rew = self.pos_shaping - pos_shaping  # 高度偏差减小为正
            self.pos_shaping = pos_shaping

            # Vel reward：速度贴近期望速度
            for a in self.world.agents:
                vel_shaping = (
                    torch.linalg.vector_norm(a.state.vel - self.desired_vel, dim=-1)
                    * self.vel_shaping_factor
                )
                a.vel_rew = a.vel_shaping - vel_shaping  # 速度误差下降为正
                a.vel_shaping = vel_shaping
            self.vel_reward = torch.stack(
                [a.vel_rew for a in self.world.agents],
                dim=1,
            ).mean(-1)  # 两机平均

            # Energy reward：控制能耗惩罚
            for a in self.world.agents:
                energy_shaping = (
                    torch.linalg.vector_norm(a.action.u, dim=-1)
                    * self.energy_shaping_factor
                )
                a.energy_rew = a.energy_shaping - energy_shaping  # 能耗上升给负值
                a.energy_rew[self.t < 10] = 0  # 前 10 步不计惩罚
                a.energy_shaping = energy_shaping

            self.energy_rew = torch.stack(
                [a.energy_rew for a in self.world.agents],
                dim=1,
            ).mean(-1)

            # Wind reward：抵消风/重力大小
            for a in self.world.agents:
                wind_shaping = (
                    torch.linalg.vector_norm(a.gravity, dim=-1)
                    * self.wind_shaping_factor
                )
                a.wind_rew = a.wind_shaping - wind_shaping  # 重力越小越好
                a.wind_rew[self.t < 5] = 0  # 前 5 步不计
                a.wind_shaping = wind_shaping

            self.wind_rew = torch.stack(
                [a.wind_rew for a in self.world.agents],
                dim=1,
            ).mean(-1)

        return (
            self.dist_rew  # 间距奖励
            + self.vel_reward  # 速度奖励
            + self.rot_rew  # 朝向奖励
            + self.energy_rew  # 能耗惩罚（为负）
            + self.wind_rew  # 风力惩罚（为负）
            + self.pos_rew  # 高度推进奖励
        )

    def set_friction(self):
        dist_to_goal_angle = (
            get_line_angle_dist_0_360(
                self.get_agents_angle(),
                torch.tensor([-torch.pi / 2], device=self.world.device).expand(
                    self.world.batch_dim, 1
                ),
            )
            + 1
        ).clamp(max=self.cover_angle_tolerance).unsqueeze(-1) + (
            1 - self.cover_angle_tolerance
        )  # Between 1 and 1 - tolerance
        dist_to_goal_angle = (dist_to_goal_angle - 1 + self.cover_angle_tolerance) / (
            self.cover_angle_tolerance
        )  # Between 1 and 0
        self.big_agent.gravity = self.wind * dist_to_goal_angle

    def observation(self, agent: Agent):
        observations = []  # 按开关收集各项观测
        if self.observe_pos:
            observations.append(agent.state.pos)  # 自身位置
        observations.append(agent.state.vel)  # 自身速度（始终加入）
        if self.observe_rel_pos:
            for a in self.world.agents:
                if a != agent:
                    observations.append(a.state.pos - agent.state.pos)  # 其他智能体相对位置
        if self.observe_rel_vel:
            for a in self.world.agents:
                if a != agent:
                    observations.append(a.state.vel - agent.state.vel)  # 其他智能体相对速度

        return torch.cat(
            observations,
            dim=-1,
        )  # 拼接为单一向量

    def get_agents_angle(self):
        return torch.atan2(
            self.big_agent.state.pos[:, Y] - self.small_agent.state.pos[:, Y],
            self.big_agent.state.pos[:, X] - self.small_agent.state.pos[:, X],
        ).unsqueeze(-1)

    def info(self, agent: Agent) -> Dict[str, Tensor]:
        return {
            "dist_rew": self.dist_rew,
            "rot_rew": self.rot_rew,
            "pos_rew": self.pos_rew,
            "agent_wind_rew": agent.wind_rew,
            "agent_vel_rew": agent.vel_rew,
            "agent_energy_rew": agent.energy_rew,
            "delta_vel_to_goal": torch.linalg.vector_norm(
                agent.state.vel - self.desired_vel, dim=-1
            ),
        }

    def extra_render(self, env_index: int = 0) -> "List[Geom]":
        from vmas.simulator import rendering

        geoms = []
        # Trajectory vel
        color = Color.BLACK.value
        circle = rendering.Line(
            (-self.desired_distance / 2, 0),
            (self.desired_distance / 2, 0),
            width=1,
        )
        xform = rendering.Transform()
        xform.set_translation(
            *(
                (
                    self.big_agent.state.pos[env_index]
                    + self.small_agent.state.pos[env_index]
                )
                / 2
            )
        )
        xform.set_rotation(self.get_agents_angle()[env_index])
        circle.add_attr(xform)
        circle.set_color(*color)
        geoms.append(circle)

        # Y line
        color = Color.RED.value
        circle = rendering.Line(
            (-self.desired_distance / 2, 0),
            (self.desired_distance / 2, 0),
            width=1,
        )
        xform = rendering.Transform()
        xform.set_translation(0.0, self.max_pos)
        circle.add_attr(xform)
        circle.set_color(*color)
        geoms.append(circle)

        return geoms


if __name__ == "__main__":
    render_interactively(__file__, control_two_agents=True)
