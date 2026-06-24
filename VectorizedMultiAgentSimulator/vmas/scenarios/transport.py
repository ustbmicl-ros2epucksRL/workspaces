#  Copyright (c) ProrokLab.
#  版权所有 (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  此源代码的许可证可在根目录的 LICENSE 文件中找到
#  LICENSE file in the root directory of this source tree.

"""
Transport场景 - 多智能体协作运输任务

场景描述:
    多个智能体需要协作将一个或多个重物（包裹）推到目标位置。
    包裹质量很大，单个智能体无法独自推动，需要多个智能体从不同方向协调推动。

任务特点:
    - 协作性: 必须多智能体合作
    - 物理性: 真实的力和质量
    - 挑战性: 需要规划推动方向和位置

应用场景:
    - 仓库机器人协作搬运
    - 无人机协作运输
    - 多机器人物流系统
"""

import torch  # PyTorch库

from vmas import render_interactively  # 交互式渲染工具
from vmas.simulator.core import Agent, Box, Landmark, Sphere, World  # 核心类
from vmas.simulator.heuristic_policy import BaseHeuristicPolicy  # 策略基类
from vmas.simulator.scenario import BaseScenario  # 场景基类
from vmas.simulator.utils import Color, ScenarioUtils  # 工具类
#from vmas.simulator.dynamics.holonomic import Holonomic  # 全向移动动力学
#from vmas.simulator.dynamics.diff_drive import DiffDrive  # 差速驱动动力学


class Scenario(BaseScenario):
    """
    Transport场景实现
    
    这个类定义了运输任务的所有方面：
    - 世界结构（智能体、包裹、目标）
    - 初始化逻辑
    - 观测计算
    - 奖励函数
    - 终止条件
    """
    
    def make_world(self, batch_dim: int, device: torch.device, **kwargs):
        """
        创建运输场景的仿真世界
        
        参数:
            batch_dim (int): 批次维度，即并行仿真的环境数量
            device (torch.device): 计算设备（CPU或GPU）
            **kwargs: 额外的场景参数
                - n_agents (int): 智能体数量，默认4
                - n_packages (int): 包裹数量，默认1
                - package_width (float): 包裹宽度，默认0.15
                - package_length (float): 包裹长度，默认0.15
                - package_mass (float): 包裹质量，默认50（非常重！）
        
        返回:
            World: 配置好的仿真世界对象
        """
        # ========== 参数解析 ==========
        # 从kwargs中提取参数，使用pop()确保参数被"消费"
        n_agents = kwargs.pop("n_agents", 4)  # 智能体数量，默认4
        self.n_packages = kwargs.pop("n_packages", 1)  # 包裹数量，默认1
        self.package_width = kwargs.pop("package_width", 0.15)  # 包裹宽度
        self.package_length = kwargs.pop("package_length", 0.15)  # 包裹长度
        self.package_mass = kwargs.pop("package_mass", 50)  # 包裹质量
        
        # 检查是否有未使用的参数（帮助发现拼写错误）
        ScenarioUtils.check_kwargs_consumed(kwargs)

        # ========== 场景常量 ==========
        self.shaping_factor = 100  # Shaping reward的缩放因子（奖励=距离*100）
        self.world_semidim = 1  # 世界的半径（世界范围：[-1,1] x [-1,1]）
        self.agent_radius = 0.03  # 智能体半径

        # ========== 创建世界 ==========
        # 世界边界稍大于world_semidim，为智能体和包裹留出空间
        world = World(
            batch_dim,  # 批次维度
            device,  # 计算设备
            # x方向半径 = 基础半径 + 智能体直径 + 包裹最大尺寸
            x_semidim=self.world_semidim
            + 2 * self.agent_radius
            + max(self.package_length, self.package_width),
            # y方向半径（同x）
            y_semidim=self.world_semidim
            + 2 * self.agent_radius
            + max(self.package_length, self.package_width),
        )
        
        # ========== 添加智能体 ==========
        for i in range(n_agents):
            agent = Agent(
                name=f"agent_{i}",  # 智能体名称
                shape=Sphere(self.agent_radius),  # 球形，半径0.03
                u_multiplier=0.6,  # 动作力的倍数（0.6表示力较小，难以单独推动包裹）
            )
            world.add_agent(agent)  # 将智能体添加到世界
        
        # ========== 添加目标地标 ==========
        goal = Landmark(
            name="goal",  # 地标名称
            collide=False,  # 不碰撞（智能体和包裹可以穿过）
            shape=Sphere(radius=0.15),  # 球形目标区域
            color=Color.LIGHT_GREEN,  # 浅绿色
        )
        world.add_landmark(goal)  # 添加到世界
        
        # ========== 添加包裹 ==========
        self.packages = []  # 存储所有包裹的列表
        for i in range(self.n_packages):
            package = Landmark(
                name=f"package {i}",  # 包裹名称
                collide=True,  # 可碰撞（会与智能体和其他物体产生碰撞力）
                movable=True,  # 可移动（受力会移动）
                mass=self.package_mass,  # 质量很大！默认50
                shape=Box(length=self.package_length, width=self.package_width),  # 矩形包裹
                color=Color.RED,  # 红色（表示未到达目标）
            )
            package.goal = goal  # 将目标绑定到包裹上（方便后续引用）
            self.packages.append(package)  # 添加到包裹列表
            world.add_landmark(package)  # 添加到世界

        return world  # 返回创建的世界

    def reset_world_at(self, env_index: int = None):
        """
        重置世界状态（初始化位置、速度等）
        
        参数:
            env_index (int, optional): 要重置的环境索引
                - None: 批量重置所有环境（向量化操作）
                - int: 只重置指定索引的单个环境
        
        重置策略:
            1. 随机放置智能体
            2. 随机放置目标和包裹（避开智能体）
            3. 初始化包裹的shaping状态
        """
        # ========== 步骤1：随机放置智能体 ==========
        ScenarioUtils.spawn_entities_randomly(
            self.world.agents,  # 要放置的实体列表
            self.world,  # 世界对象
            env_index,  # 环境索引
            min_dist_between_entities=self.agent_radius * 2,  # 最小间距（避免重叠）
            x_bounds=(
                -self.world_semidim,  # x范围：[-1, 1]
                self.world_semidim,
            ),
            y_bounds=(
                -self.world_semidim,  # y范围：[-1, 1]
                self.world_semidim,
            ),
        )
        
        # ========== 步骤2：记录智能体占据的位置 ==========
        # 收集所有智能体的位置，用于后续避让
        # 形状: (batch_dim, n_agents, 2)
        agent_occupied_positions = torch.stack(
            [agent.state.pos for agent in self.world.agents], dim=1
        )
        # 如果只重置单个环境，只保留该环境的数据
        if env_index is not None:
            agent_occupied_positions = agent_occupied_positions[env_index].unsqueeze(0)

        # ========== 步骤3：随机放置目标和包裹（避开智能体） ==========
        goal = self.world.landmarks[0]  # 获取目标地标
        ScenarioUtils.spawn_entities_randomly(
            [goal] + self.packages,  # 要放置的实体：目标 + 所有包裹
            self.world,
            env_index,
            # 确保包裹和目标不会太近（否则任务太简单）
            # 使用包裹外接圆半径 + 目标半径 + 额外间距
            min_dist_between_entities=max(
                package.shape.circumscribed_radius() + goal.shape.radius + 0.01
                for package in self.packages
            ),
            x_bounds=(
                -self.world_semidim,
                self.world_semidim,
            ),
            y_bounds=(
                -self.world_semidim,
                self.world_semidim,
            ),
            occupied_positions=agent_occupied_positions,  # 已占据的位置（智能体）
        )

        # ========== 步骤4：初始化包裹状态 ==========
        for package in self.packages:
            # 检查包裹是否一开始就在目标上（很少发生，但需要处理）
            package.on_goal = self.world.is_overlapping(package, package.goal)

            # 初始化global_shaping变量（用于计算shaping reward）
            # global_shaping保存上一步的"距离 * shaping_factor"
            # 用于计算reward时的距离变化
            if env_index is None:
                # 批量初始化所有环境
                # 计算：包裹到目标的距离 * shaping_factor
                package.global_shaping = (
                    torch.linalg.vector_norm(
                        package.state.pos - package.goal.state.pos, dim=1
                    )
                    * self.shaping_factor
                )
            else:
                # 只初始化指定环境
                package.global_shaping[env_index] = (
                    torch.linalg.vector_norm(
                        package.state.pos[env_index] - package.goal.state.pos[env_index]
                    )
                    * self.shaping_factor
                )

    def reward(self, agent: Agent):
        """
        计算智能体的奖励
        
        使用Shaping Reward策略：
        - 包裹接近目标时获得正奖励
        - 包裹远离目标时获得负奖励
        - 到达目标后不再给予shaping reward
        
        参数:
            agent (Agent): 要计算奖励的智能体
        
        返回:
            torch.Tensor: 奖励张量，形状 (batch_dim,)
        
        设计思路:
            - 所有智能体共享相同的奖励（协作任务）
            - 只用第一个智能体计算奖励（避免重复计算）
            - 使用距离变化而非绝对距离（更好的学习信号）
        """
        # 检查是否是第一个智能体（用于避免重复计算）
        is_first = agent == self.world.agents[0]

        if is_first:
            # ========== 初始化奖励张量 ==========
            # 创建全零张量，形状 (batch_dim,)
            self.rew = torch.zeros(
                self.world.batch_dim,
                device=self.world.device,
                dtype=torch.float32,
            )

            # ========== 遍历所有包裹计算奖励 ==========
            for package in self.packages:
                # ----- 计算当前状态 -----
                # 计算包裹到目标的距离
                # 形状: (batch_dim,)
                package.dist_to_goal = torch.linalg.vector_norm(
                    package.state.pos - package.goal.state.pos, dim=1
                )
                
                # 检查包裹是否在目标上
                package.on_goal = self.world.is_overlapping(package, package.goal)
                
                # ----- 更新包裹颜色 -----
                # 根据是否到达目标改变颜色（视觉反馈）
                # 创建颜色张量，形状 (batch_dim, 3)，初始为红色
                package.color = torch.tensor(
                    Color.RED.value,  # RGB: (0.75, 0.25, 0.25)
                    device=self.world.device,
                    dtype=torch.float32,
                ).repeat(self.world.batch_dim, 1)
                
                # 到达目标的环境中，将包裹颜色改为绿色
                package.color[package.on_goal] = torch.tensor(
                    Color.GREEN.value,  # RGB: (0.25, 0.75, 0.25)
                    device=self.world.device,
                    dtype=torch.float32,
                )

                # ----- 计算Shaping Reward -----
                # 当前的shaping值 = 距离 * 缩放因子
                package_shaping = package.dist_to_goal * self.shaping_factor
                
                # Shaping reward = 上一步的shaping - 当前shaping
                # 如果距离减小（靠近目标），reward为正
                # 如果距离增大（远离目标），reward为负
                # 只对未到达目标的环境计算reward
                self.rew[~package.on_goal] += (
                    package.global_shaping[~package.on_goal]  # 上一步的值
                    - package_shaping[~package.on_goal]  # 当前的值
                )
                
                # 更新global_shaping为当前值（用于下一步计算）
                package.global_shaping = package_shaping

        # 返回奖励（所有智能体共享相同奖励）
        return self.rew

    def observation(self, agent: Agent):
        """
        计算智能体的观测
        
        观测内容:
            1. 智能体自身状态（4维）:
               - 位置 (2维)
               - 速度 (2维)
            
            2. 每个包裹的信息（7维）:
               - 包裹到目标的相对位置 (2维)
               - 包裹到智能体的相对位置 (2维)
               - 包裹的速度 (2维)
               - 包裹是否在目标上 (1维，布尔转浮点)
        
        参数:
            agent (Agent): 要计算观测的智能体
        
        返回:
            torch.Tensor: 观测张量，形状 (batch_dim, obs_dim)
                         obs_dim = 4 + 7 * n_packages
        
        设计思路:
            - 使用相对位置（而非绝对位置）
            - 提供速度信息（预测运动）
            - 提供目标达成信息（任务进度）
        """
        # ========== 收集所有包裹的观测 ==========
        package_obs = []  # 存储每个包裹的观测向量
        
        for package in self.packages:
            # 1. 包裹到目标的相对位置 (2维)
            # 告诉智能体"包裹应该往哪个方向移动"
            package_obs.append(package.state.pos - package.goal.state.pos)
            
            # 2. 包裹到智能体的相对位置 (2维)
            # 告诉智能体"包裹在我的哪个方向"
            package_obs.append(package.state.pos - agent.state.pos)
            
            # 3. 包裹的速度 (2维)
            # 告诉智能体"包裹正在往哪个方向移动"
            package_obs.append(package.state.vel)
            
            # 4. 包裹是否在目标上 (1维)
            # 告诉智能体"任务是否已完成"
            # unsqueeze(-1)将形状从(batch_dim,)变为(batch_dim, 1)
            package_obs.append(package.on_goal.unsqueeze(-1))

        # ========== 拼接所有观测 ==========
        # 沿最后一维拼接所有观测向量
        return torch.cat(
            [
                agent.state.pos,   # 智能体位置 (batch_dim, 2)
                agent.state.vel,   # 智能体速度 (batch_dim, 2)
                *package_obs,      # 所有包裹的信息 (batch_dim, 7*n_packages)
            ],
            dim=-1,  # 沿最后一维拼接
        )
        # 结果形状: (batch_dim, 4 + 7*n_packages)

    def done(self):
        """
        计算终止条件
        
        终止条件:
            当所有包裹都到达目标时，任务完成
        
        返回:
            torch.Tensor: 终止标志张量，形状 (batch_dim,)，dtype=torch.bool
                         True表示该环境应该终止
        
        实现:
            使用逻辑AND操作检查所有包裹是否都在目标上
        """
        # 将所有包裹的on_goal状态堆叠成张量
        # 形状: (batch_dim, n_packages)
        all_packages_status = torch.stack(
            [package.on_goal for package in self.packages],
            dim=1,
        )
        
        # 检查每个环境中是否所有包裹都在目标上
        # torch.all(dim=-1)沿包裹维度进行AND操作
        # 返回形状: (batch_dim,)
        return torch.all(all_packages_status, dim=-1)


class HeuristicPolicy(BaseHeuristicPolicy):
    """
    Transport任务的启发式策略
    
    策略核心思想：Dribble（带球）
    类似足球运动员带球前进：
    1. 识别目标方向
    2. 移动到球（包裹）的后方
    3. 向前推动
    
    技术实现：
    使用Hermite样条曲线生成平滑轨迹
    
    Hermite样条：
    一种三次多项式插值方法，通过指定起点、终点及其导数（速度）来生成平滑曲线。
    优点：轨迹平滑、速度可控、避免突然转向
    """
    
    def __init__(self, *args, **kwargs):
        """
        初始化启发式策略
        
        设置各种控制参数，这些参数决定了智能体的运动特性。
        """
        super().__init__(*args, **kwargs)
        
        # ===== Hermite样条参数 =====
        self.lookahead = 0.0  
        # 在样条曲线上的评估位置（0=起点, 1=终点）
        # 0表示当前位置，可以设置为0.1-0.5来"提前看"
        
        # ===== 起始速度计算参数 =====
        self.start_vel_dist_from_target_ratio = 0.5  
        # 起始速度指向的点距离目标的比例（0.5=中间）
        
        self.start_vel_behind_ratio = 0.5  
        # 起始速度的后向分量比例
        # 0.5表示50%向后，50%向侧面
        
        self.start_vel_mag = 1.0  
        # 起始速度的大小（控制整体轨迹速度）
        
        # ===== 击打参数 =====
        self.hit_vel_mag = 1.0  
        # 击打包裹时的期望速度
        
        # ===== 几何参数 =====
        self.package_radius = 0.15 / 2  
        # 包裹半径（假设包裹是正方形，用外接圆半径）
        
        self.agent_radius = -0.02  
        # 智能体半径（负值表示可以更靠近包裹）
        
        # ===== 其他参数 =====
        self.dribble_slowdown_dist = 0.0  
        # 接近目标时的减速距离（0表示不减速）
        
        self.speed = 0.95  
        # 整体速度缩放因子（0-1之间）

    def compute_action(self, observation: torch.Tensor, u_range: float) -> torch.Tensor:
        """
        从观测计算动作
        
        参数:
            observation (torch.Tensor): 智能体的观测
                形状: (n_envs, obs_dim)
                内容: [agent_pos(2), agent_vel(2), package_to_goal(2), 
                      package_to_agent(2), package_vel(2), on_goal(1)]
            u_range (float): 动作范围限制
        
        返回:
            torch.Tensor: 动作向量，形状 (n_envs, 2)
        
        流程:
            1. 从观测中提取位置信息
            2. 使用dribble策略计算控制指令
            3. 缩放并限制在允许范围内
        """
        # ===== 保存环境信息 =====
        self.n_env = observation.shape[0]  # 环境数量
        self.device = observation.device  # 计算设备
        
        # ===== 从观测中提取位置 =====
        agent_pos = observation[:, :2]  # 智能体位置 (n_envs, 2)
        # agent_vel = observation[:, 2:4]  # 智能体速度（未使用）
        
        # 从相对位置恢复绝对位置
        package_pos = observation[:, 6:8] + agent_pos  # 包裹位置
        goal_pos = -observation[:, 4:6] + package_pos  # 目标位置
        
        # ===== 计算控制指令 =====
        # 使用dribble策略（智能体绕到包裹后方并推动）
        control = self.dribble(agent_pos, package_pos, goal_pos)
        
        # ===== 缩放和限制 =====
        # 乘以速度因子和动作范围
        control *= self.speed * u_range
        # 限制在[-u_range, u_range]范围内
        return torch.clamp(control, -u_range, u_range)

    def dribble(self, agent_pos, package_pos, goal_pos, agent_vel=None):
        """
        Dribble策略：推动包裹向目标移动
        
        类比足球：
        - package_pos是球的位置
        - goal_pos是球门
        - agent_pos是运动员位置
        - 策略：绕到球后方，向球门方向推球
        
        参数:
            agent_pos: 智能体位置 (n_envs, 2)
            package_pos: 包裹位置 (n_envs, 2)
            goal_pos: 目标位置 (n_envs, 2)
            agent_vel: 智能体速度（可选）
        
        返回:
            control: 控制指令 (n_envs, 2)
        
        步骤:
            1. 计算包裹到目标的方向
            2. 确定击打点（包裹后方）
            3. 计算击打速度（朝向目标）
            4. 生成从当前位置到击打点的轨迹
        """
        # ===== 步骤1：计算包裹到目标的方向 =====
        package_disp = goal_pos - package_pos  # 位移向量 (n_envs, 2)
        ball_dist = package_disp.norm(dim=-1)  # 距离 (n_envs,)
        direction = package_disp / ball_dist[:, None]  # 单位方向向量 (n_envs, 2)
        
        # ===== 步骤2：计算击打点 =====
        # 击打点在包裹的后方（相对于目标方向）
        # hit_pos = package_pos - direction * (包裹半径 + 智能体半径)
        hit_pos = package_pos - direction * (self.package_radius + self.agent_radius)
        
        # ===== 步骤3：计算击打速度 =====
        # 击打速度朝向目标
        hit_vel = direction * self.hit_vel_mag
        
        # ===== 步骤4：计算起始速度 =====
        # 起始速度决定了轨迹的形状
        start_vel = self.get_start_vel(
            hit_pos,  # 目标位置（击打点）
            hit_vel,  # 目标速度
            agent_pos,  # 起始位置
            self.start_vel_mag * 2  # 起始速度大小
        )
        
        # ===== 步骤5：减速处理（可选） =====
        # 如果包裹非常接近目标，降低速度
        slowdown_mask = ball_dist <= self.dribble_slowdown_dist
        hit_vel[slowdown_mask, :] *= (
            ball_dist[slowdown_mask, None] / self.dribble_slowdown_dist
        )
        
        # ===== 步骤6：生成控制指令 =====
        # 使用Hermite样条从当前位置到击打点生成平滑轨迹
        return self.get_action(
            target_pos=hit_pos,      # 目标位置（击打点）
            target_vel=hit_vel,      # 目标速度（击打速度）
            curr_pos=agent_pos,      # 当前位置
            curr_vel=agent_vel,      # 当前速度
            start_vel=start_vel,     # 起始速度
        )

    def hermite(self, p0, p1, p0dot, p1dot, u=0.0, deriv=0):
        """
        Hermite样条插值
        
        Hermite样条是一种三次多项式，通过指定起点、终点及其导数来定义。
        
        数学公式:
            H(u) = [2u³-3u²+1]p0 + [-2u³+3u²]p1 + [u³-2u²+u]p0' + [u³-u²]p1'
        
        其中 u ∈ [0, 1] 是插值参数:
            - u=0: 起点
            - u=1: 终点
            - 0<u<1: 中间点
        
        参数:
            p0: 起点位置 (n_envs, 2)
            p1: 终点位置 (n_envs, 2)
            p0dot: 起点导数（速度） (n_envs, 2)
            p1dot: 终点导数（速度） (n_envs, 2)
            u: 插值参数，0到1（标量或向量）
            deriv: 导数阶数（0=位置, 1=速度, 2=加速度）
        
        返回:
            ans: 插值结果 (n_envs, 2)
        """
        # ===== 格式化参数u =====
        u = u.reshape((-1,))  # 确保u是一维向量
        
        # ===== 计算参数向量U =====
        # U = [u³, u², u, 1]（考虑导数阶数）
        # nPr(n, r)计算排列数，用于导数的系数
        U = torch.stack(
            [
                self.nPr(3, deriv) * (u ** max(0, 3 - deriv)),  # u³项
                self.nPr(2, deriv) * (u ** max(0, 2 - deriv)),  # u²项
                self.nPr(1, deriv) * (u ** max(0, 1 - deriv)),  # u项
                self.nPr(0, deriv) * (u**0),                     # 常数项
            ],
            dim=1,
        ).float()
        
        # ===== Hermite基函数矩阵A =====
        # 这个矩阵将[u³, u², u, 1]转换为Hermite基函数
        A = torch.tensor(
            [
                [2.0, -2.0, 1.0, 1.0],    # H0, H1基函数
                [-3.0, 3.0, -2.0, -1.0],  # 一阶导数系数
                [0.0, 0.0, 1.0, 0.0],      # H2基函数
                [1.0, 0.0, 0.0, 0.0],      # H3基函数
            ],
            device=U.device,
        )
        
        # ===== 控制点矩阵P =====
        # P = [p0, p1, p0dot, p1dot]
        P = torch.stack([p0, p1, p0dot, p1dot], dim=1)
        
        # ===== 计算样条值 =====
        # H(u) = U @ A @ P
        # 形状: (n_envs, 1, 4) @ (1, 4, 4) @ (n_envs, 4, 2)
        #     = (n_envs, 1, 2)
        ans = U[:, None, :] @ A[None, :, :] @ P
        
        # 去除中间维度
        ans = ans.squeeze(1)  # (n_envs, 2)
        return ans

    def nPr(self, n, r):
        """
        计算排列数 P(n, r) = n!/(n-r)!
        
        用于计算导数时的系数。
        
        例如:
            - nPr(3, 0) = 1     (位置，u³的0阶导数)
            - nPr(3, 1) = 3     (速度，u³的1阶导数是3u²)
            - nPr(3, 2) = 6     (加速度，u³的2阶导数是6u)
        
        参数:
            n: 总数
            r: 选择数
        
        返回:
            排列数（整数）
        """
        if r > n:
            return 0
        ans = 1
        for k in range(n, max(1, n - r), -1):
            ans = ans * k
        return ans

    def get_start_vel(self, pos, vel, start_pos, start_vel_mag):
        """
        计算起始速度
        
        目的:
            起始速度不应该直接指向目标，而应该指向目标附近的某个点，
            以确保轨迹平滑并避免直线冲撞。
        
        策略:
            起始速度指向目标后方和侧方的某个位置
        
        参数:
            pos: 目标位置 (n_envs, 2)
            vel: 目标速度 (n_envs, 2)
            start_pos: 起始位置 (n_envs, 2)
            start_vel_mag: 起始速度的大小
        
        返回:
            start_vel: 起始速度 (n_envs, 2)
        """
        # ===== 格式化起始速度大小 =====
        start_vel_mag = torch.as_tensor(start_vel_mag, device=self.device).view(-1)
        
        # ===== 计算到目标的方向 =====
        goal_disp = pos - start_pos  # 位移向量
        goal_dist = goal_disp.norm(dim=-1)  # 距离
        
        # ===== 计算目标速度的方向 =====
        vel_mag = vel.norm(dim=-1)  # 目标速度大小
        vel_dir = vel.clone()  # 复制避免修改原数据
        # 归一化（只对非零速度）
        vel_dir[vel_mag > 0] /= vel_mag[vel_mag > 0, None]
        
        # ===== 归一化到目标的方向 =====
        goal_dir = goal_disp / goal_dist[:, None]

        # ===== 计算目标速度方向的法向量（垂直方向） =====
        # 2D向量的法向量：(x, y) -> (-y, x)
        vel_dir_normal = torch.stack([-vel_dir[:, 1], vel_dir[:, 0]], dim=1)
        
        # ===== 确保法向量指向正确方向 =====
        # 法向量应该指向目标附近（与目标方向夹角<90度）
        dot_prod = (goal_dir * vel_dir_normal).sum(dim=1)
        vel_dir_normal[dot_prod > 0, :] *= -1  # 反转不正确的法向量

        # ===== 计算目标后方的距离 =====
        dist_behind_target = self.start_vel_dist_from_target_ratio * goal_dist
        
        # ===== 计算起始速度方向 =====
        # 组合后向分量和侧向分量
        point_dir = -vel_dir * self.start_vel_behind_ratio + vel_dir_normal * (
            1 - self.start_vel_behind_ratio
        )

        # ===== 计算起始速度应指向的目标点 =====
        target_pos = pos + point_dir * dist_behind_target[:, None]
        target_disp = target_pos - start_pos
        target_dist = target_disp.norm(dim=1)
        
        # ===== 归一化并乘以速度大小 =====
        start_vel_aug_dir = target_disp
        start_vel_aug_dir[target_dist > 0] /= target_dist[target_dist > 0, None]
        start_vel = start_vel_aug_dir * start_vel_mag[:, None]
        
        return start_vel

    def get_action(
        self,
        target_pos,
        target_vel=None,
        start_pos=None,
        start_vel=None,
        curr_pos=None,
        curr_vel=None,
    ):
        """
        使用Hermite样条计算控制动作
        
        这是一个PD控制器（比例-微分控制器）：
        control = Kp * (期望位置 - 当前位置) + Kd * (期望速度 - 当前速度)
        这里 Kp = Kd = 0.5
        
        参数:
            target_pos: 目标位置
            target_vel: 目标速度（默认为0）
            start_pos: 轨迹起点（默认为当前位置）
            start_vel: 起始速度（默认自动计算）
            curr_pos: 当前位置（默认为0）
            curr_vel: 当前速度（默认为0）
        
        返回:
            control: 控制指令 (n_envs, 2)
        """
        # ===== 处理默认值 =====
        if curr_pos is None:  
            # 如果未提供，假设target_pos是相对位置
            curr_pos = torch.zeros(target_pos.shape, device=self.device)
        if curr_vel is None:
            curr_vel = torch.zeros(target_pos.shape, device=self.device)
        if start_pos is None:
            # 起点默认为当前位置
            start_pos = curr_pos
        if target_vel is None:
            target_vel = torch.zeros(target_pos.shape, device=self.device)
        if start_vel is None:
            # 自动计算起始速度
            start_vel = self.get_start_vel(
                target_pos, target_vel, start_pos, self.start_vel_mag * 2
            )

        # ===== 创建评估参数u =====
        # u = lookahead，通常为0（当前位置）
        u_start = torch.ones(curr_pos.shape[0], device=self.device) * self.lookahead
        
        # ===== 使用Hermite样条计算期望的位置 =====
        des_curr_pos = self.hermite(
            start_pos,    # 起点
            target_pos,   # 终点
            start_vel,    # 起点速度
            target_vel,   # 终点速度
            u=u_start,    # 评估参数（0=起点）
            deriv=0,      # 0阶导数（位置）
        )
        
        # ===== 使用Hermite样条计算期望的速度 =====
        des_curr_vel = self.hermite(
            start_pos,
            target_pos,
            start_vel,
            target_vel,
            u=u_start,
            deriv=1,      # 1阶导数（速度）
        )
        
        # ===== 确保是张量 =====
        des_curr_pos = torch.as_tensor(des_curr_pos, device=self.device)
        des_curr_vel = torch.as_tensor(des_curr_vel, device=self.device)
        
        # ===== 计算控制指令（PD控制） =====
        # control = 0.5 * 位置误差 + 0.5 * 速度误差
        control = 0.5 * (des_curr_pos - curr_pos) + 0.5 * (des_curr_vel - curr_vel)
        
        return control


# ========== 主程序入口 ==========
if __name__ == "__main__":
    # 交互式渲染
    # 允许用户使用键盘控制两个智能体，观察场景行为
    render_interactively(__file__, control_two_agents=True)

