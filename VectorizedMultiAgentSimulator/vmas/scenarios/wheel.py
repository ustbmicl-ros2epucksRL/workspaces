#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.

import torch

from vmas import render_interactively
from vmas.simulator.core import Agent, Landmark, Line, Sphere, World
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy
from vmas.simulator.scenario import BaseScenario
from vmas.simulator.utils import Color, ScenarioUtils, TorchUtils


class Scenario(BaseScenario):
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        n_agents = kwargs.pop("n_agents", 4)
        self.line_length = kwargs.pop("line_length", 2)
        line_mass = kwargs.pop("line_mass", 30)
        self.desired_velocity = kwargs.pop("desired_velocity", 0.05)
        ScenarioUtils.check_kwargs_consumed(kwargs)

        # Make world
        world = World(batch_dim, device)
        # Add agents
        for i in range(n_agents):
            # Constraint: all agents have same action range and multiplier
            agent = Agent(name=f"agent_{i}", u_multiplier=0.6, shape=Sphere(0.03))
            world.add_agent(agent)
        # Add landmarks
        self.line = Landmark(
            name="line",
            collide=True,
            rotatable=True,
            shape=Line(length=self.line_length),
            mass=line_mass,
            color=Color.BLACK,
        )
        world.add_landmark(self.line)
        center = Landmark(
            name="center",
            shape=Sphere(radius=0.02),
            collide=False,
            color=Color.BLACK,
        )
        world.add_landmark(center)

        return world

    def reset_world_at(self, env_index: int = None):
        for agent in self.world.agents:
            # Random pos between -1 and 1
            agent.set_pos(
                torch.zeros(
                    (
                        (1, self.world.dim_p)
                        if env_index is not None
                        else (self.world.batch_dim, self.world.dim_p)
                    ),
                    device=self.world.device,
                    dtype=torch.float32,
                ).uniform_(
                    -1.0,
                    1.0,
                ),
                batch_index=env_index,
            )

        self.line.set_rot(
            torch.zeros(
                (1, 1) if env_index is not None else (self.world.batch_dim, 1),
                device=self.world.device,
                dtype=torch.float32,
            ).uniform_(
                -torch.pi / 2,
                torch.pi / 2,
            ),
            batch_index=env_index,
        )

    def reward(self, agent: Agent):
        is_first = agent == self.world.agents[0]  # 只在第一个智能体上计算全局奖励

        if is_first:
            # 角速度误差：|当前角速度| 与 期望角速度 的差的绝对值
            self.rew = (self.line.state.ang_vel.abs() - self.desired_velocity).abs()

        # 奖励是负的误差（误差越小越好，即整体越大越好）
        return -self.rew

    def observation(self, agent: Agent):
        # 计算线段两端的绝对位置（基于中心和当前旋转角）
        line_end_1 = torch.cat(
            [
                (self.line_length / 2) * torch.cos(self.line.state.rot),
                (self.line_length / 2) * torch.sin(self.line.state.rot),
            ],
            dim=1,
        )
        line_end_2 = -line_end_1  # 另一端与中心对称

        # 拼接观测向量
        return torch.cat(
            [
                agent.state.pos,  # 自身位置
                agent.state.vel,  # 自身速度
                self.line.state.pos - agent.state.pos,  # 到线段中心的相对位置
                line_end_1 - agent.state.pos,  # 到线端点1的相对位置
                line_end_2 - agent.state.pos,  # 到线端点2的相对位置
                self.line.state.rot % torch.pi,  # 线段角度（模 pi，因为 180° 对称）
                self.line.state.ang_vel.abs(),  # 线段角速度幅值
                (self.line.state.ang_vel.abs() - self.desired_velocity).abs(),  # 角速度误差
            ],
            dim=-1,
        )


class HeuristicPolicy(BaseHeuristicPolicy):
    def compute_action(self, observation: torch.Tensor, u_range: float) -> torch.Tensor:
        assert self.continuous_actions is True, "Heuristic for continuous actions only"

        index_line_extrema = 6

        pos_agent = observation[:, :2]
        pos_end2_agent = observation[:, index_line_extrema + 2 : index_line_extrema + 4]

        pos_end2 = pos_end2_agent + pos_agent

        pos_end2_shifted = TorchUtils.rotate_vector(
            pos_end2,
            torch.tensor(torch.pi / 4, device=observation.device).expand(
                pos_end2.shape[0]
            ),
        )

        pos_end2_shifted_agent = pos_end2_shifted - pos_agent

        action_agent = torch.clamp(
            pos_end2_shifted_agent,
            min=-u_range,
            max=u_range,
        )

        return action_agent


if __name__ == "__main__":
    render_interactively(
        __file__,
        control_two_agents=True,
        desired_velocity=0.05,
        n_agents=4,
        line_length=2,
        line_mass=30,
    )
