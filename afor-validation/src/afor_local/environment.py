"""The original narrowing task; physics and final goals without a route planner."""
from dataclasses import dataclass, asdict
import torch
from .formations import make_formation
from .swept_geometry import segment_polygon_distance
from .observation import SensorConfig, lidar_scan, pack_observation
from .config import ScenarioConfig


def _perturbation_seed(seed):
    # This stream never consumes the production environment's endpoint RNG.
    return (int(seed) + 0x4252475632) % (2**63 - 1)


def _draw_valid_y_jitter(env, ids, amplitudes, generator, max_rejections):
    """Perturb selected teams, rejecting complete invalid team proposals.

    The base pose, goals and sampled amplitude are fixed during rejection.
    Other teams and the production endpoint RNG are untouched. Returns actual
    offsets and the number of proposals per team, indexed by environment.
    """
    base = env.pos.clone()
    offsets = torch.zeros_like(env.pos[..., 1])
    attempts = torch.zeros(env.config.num_envs, dtype=torch.long, device=env.device)
    pending = ids.clone()
    for _ in range(max_rejections):
        if not pending.numel():
            break
        draw = (2 * torch.rand((len(pending), env.n_agents), generator=generator,
                               device=env.device) - 1) * amplitudes[pending, None]
        candidate = base.clone()
        candidate[pending, :, 1] += draw
        valid = (env.clearances(candidate, candidate)[pending] > 0).all(-1)
        attempts[pending] += 1
        accepted = pending[valid]
        env.pos[accepted] = candidate[accepted]
        offsets[accepted] = draw[valid]
        pending = pending[~valid]
    if pending.numel():
        raise RuntimeError(
            f'Could not sample collision-free initial states after {max_rejections} '
            f'proposals for environment ids {pending.cpu().tolist()}')
    return offsets, attempts


@dataclass(frozen=True)
class TaskConfig:
    num_envs: int = 16
    horizon: int = 750
    formation: str = 'line'
    scenario: str = 'narrowing'
    passage_width: float = 1.2
    dt: float = .05
    radius: float = .13
    formation_weight: float = .15
    goal_distance_weight: float = 1.

    def __post_init__(self):
        if self.num_envs < 1 or self.horizon < 1 or self.dt <= 0 or not 0 < self.radius < .2:
            raise ValueError('Invalid physical/task configuration')
        if self.formation not in ('line', 'circle', 'wedge'):
            raise ValueError('Unknown formation')
        if self.scenario != 'narrowing':
            raise ValueError('Unknown scenario')
        if not 1.1 <= self.passage_width <= 1.6:
            raise ValueError('Passage width must be in [1.1, 1.6] m')
        if min(self.formation_weight,self.goal_distance_weight) < 0:
            raise ValueError('Reward weights must be nonnegative')


class LocalFormationEnv:
    """Five disk agents; original tapering corridor, strict swept failure.

    There is no automatic reset. A terminal proposal ends the entire team
    episode, leaving the last valid state for logging; it can never earn later
    progress through rollback. Call reset(done_ids) before stepping again.
    Goals are final assigned slots in a FIXED mission formation. Both are known
    before seeing a map; no compression, passage slots, or phase is computed.
    """
    n_agents = 5
    names = ('narrowing',)
    state_fields = ('pos', 'vel', 'goal', 'template', 'adjacency', 'vertices',
                    'obstacle_mask', 'kind', 'shape', 'steps', 'hold', 'ended',
                    'episode_return')

    def __init__(self, config=TaskConfig(), sensors=SensorConfig(), device='cpu', seed=0,
                 scenario=ScenarioConfig()):
        self.scenario_config = scenario
        self.perturb_rng = torch.Generator(device=torch.device(device))
        self.perturb_rng.manual_seed(_perturbation_seed(seed))
        self.jitter_amplitude = torch.zeros(config.num_envs, device=device)
        self.initial_jitter = torch.zeros(config.num_envs, self.n_agents, device=device)
        self.reset_attempts = torch.zeros(config.num_envs, dtype=torch.long, device=device)
        self.perturbation_counters = {'resets': 0, 'perturbed_resets': 0, 'rejected_proposals': 0}
        self.config, self.sensors = config, sensors
        self.device = torch.device(device)
        self.rng = torch.Generator(device=self.device).manual_seed(seed)
        self.dt, self.radius = config.dt, config.radius
        # Physical wall bounds; clearances/lidar account for robot radius.
        self.bounds = torch.tensor([2., 4.], device=self.device)
        e, n = config.num_envs, self.n_agents
        zeros = lambda *shape: torch.zeros(*shape, device=self.device)
        self.pos, self.vel, self.goal, self.template = (zeros(e,n,2) for _ in range(4))
        self.adjacency = zeros(e,n,n)
        self.vertices = zeros(e,4,4,2)
        self.obstacle_mask = torch.zeros(e,4,device=self.device,dtype=torch.bool)
        self.kind, self.shape, self.steps, self.hold = (
            torch.zeros(e,device=self.device,dtype=torch.long) for _ in range(4))
        self.ended = torch.zeros(e,device=self.device,dtype=torch.bool)
        self.episode_return = zeros(e)
        self.templates, self.graphs = [], []
        for name in ('line', 'circle', 'wedge'):
            f = make_formation(name, self.device)
            self.templates.append(f.template)
            a = zeros(n,n)
            for i,j in f.edges:
                a[i,j] = a[j,i] = 1
            self.graphs.append(a)
        self.templates, self.graphs = torch.stack(self.templates), torch.stack(self.graphs)
        self.reset()

    def reset(self, ids=None):
        if ids is None:
            ids = torch.arange(self.config.num_envs, device=self.device)
        ids = torch.as_tensor(ids, dtype=torch.long, device=self.device).flatten()
        if not len(ids):
            return
        shape = torch.full((len(ids),), ('line','circle','wedge').index(self.config.formation),
                           device=self.device, dtype=torch.long)
        self._reset_narrowing(ids, shape)
        self.jitter_amplitude[ids] = 0
        self.initial_jitter[ids] = 0
        self.reset_attempts[ids] = 0
        self.perturbation_counters['resets'] += len(ids)
        cfg = self.scenario_config
        if cfg.max_jitter == 0 or cfg.perturb_probability == 0:
            return
        selected = ids[torch.rand(len(ids), generator=self.perturb_rng,
                                 device=self.device) < cfg.perturb_probability]
        if not selected.numel():
            return
        self.jitter_amplitude[selected] = torch.rand(
            len(selected), generator=self.perturb_rng, device=self.device) * cfg.max_jitter
        offsets, attempts = _draw_valid_y_jitter(
            self, selected, self.jitter_amplitude, self.perturb_rng, cfg.max_rejections)
        self.initial_jitter[selected] = offsets[selected]
        self.reset_attempts[selected] = attempts[selected]
        self.perturbation_counters['perturbed_resets'] += len(selected)
        self.perturbation_counters['rejected_proposals'] += int(
            (attempts[selected] - 1).sum().item())

    def _reset_narrowing(self, ids, shape):
        """Only the historical physical scene and task endpoints, no planner.

        Fixed seeds preserve the historical physical task and endpoint draws.
        No intermediate slots, stages, or route geometry are constructed.
        """
        noise = torch.rand((len(ids),4),generator=self.rng,device=self.device)
        self.template[ids] = self.templates[shape]
        self.adjacency[ids] = self.graphs[shape]
        start = torch.stack(((noise[:,0]-.5)*.1, -2.8+noise[:,1]*.2),-1)
        goal = torch.stack(((noise[:,2]-.5)*.1, 2.5+noise[:,3]*.2),-1)
        self.pos[ids] = self.template[ids]+start[:,None]
        self.goal[ids] = self.template[ids]+goal[:,None]
        self.vel[ids] = 0
        half = self.config.passage_width/2
        walls = torch.tensor([
            [[-1.55,-1.6],[-1.30,-1.6],[-half,1.3],[-half-.25,1.3]],
            [[1.30,-1.6],[1.55,-1.6],[half+.25,1.3],[half,1.3]]],device=self.device)
        self.vertices[ids] = 0
        self.vertices[ids,:2] = walls
        self.obstacle_mask[ids] = False
        self.obstacle_mask[ids,:2] = True
        self.kind[ids], self.shape[ids] = self.names.index('narrowing'), shape
        self.steps[ids] = self.hold[ids] = 0
        self.ended[ids] = False
        self.episode_return[ids] = 0

    def observe(self):
        # Each i receives only measured states of j in communication range.
        delta = self.pos[:,None]-self.pos[:,:,None]
        relative_velocity = self.vel[:,None]-self.vel[:,:,None]
        desired = self.template[:,None]-self.template[:,:,None]
        mask = (delta.norm(dim=-1)<self.sensors.neighbour_range)
        mask &= ~torch.eye(self.n_agents,device=self.device,dtype=torch.bool)[None]
        peer = torch.cat((delta,relative_velocity,desired,self.adjacency[...,None],
                          torch.full_like(self.adjacency[...,None],self.radius)),-1)
        scan = lidar_scan(self.pos,self.radius,self.vertices,self.obstacle_mask,self.bounds,self.sensors)
        obs = pack_observation(self.goal-self.pos,self.vel,self.radius,scan,peer,mask,self.sensors)
        return obs

    def clearances(self, start, end):
        # Double precision, same physical disk radius for all three constraints.
        start, end = start.double(), end.double()
        i,j = torch.triu_indices(self.n_agents,self.n_agents,1,device=self.device)
        difference = start[:,i]-start[:,j]
        change = end[:,i]-end[:,j]-difference
        t = (-(difference*change).sum(-1)/change.square().sum(-1).clamp_min(1e-20)).clamp(0,1)
        pair = (difference+t[...,None]*change).norm(dim=-1).amin(-1)-2*self.radius
        distance = segment_polygon_distance(start,end,self.vertices.double()[:,None])
        wall = distance.masked_fill(~self.obstacle_mask[:,None],float('inf')).amin((1,2))-self.radius
        boundary = torch.minimum(self.bounds-start.abs(),self.bounds-end.abs()).amin((1,2))-self.radius
        return torch.stack((pair,wall,boundary),-1)

    def formation_error(self):
        delta = self.pos[:,None]-self.pos[:,:,None]
        desired = self.template[:,None]-self.template[:,:,None]
        error = (delta-desired).norm(dim=-1)*self.adjacency
        return error.sum(-1)/self.adjacency.sum(-1).clamp_min(1)

    def final_formation_ok(self):
        """Original three final shape criteria, independent of mission phase."""
        delta = self.pos[:,None]-self.pos[:,:,None]
        desired = self.template[:,None]-self.template[:,:,None]
        edge_error = ((delta.norm(dim=-1)-desired.norm(dim=-1)).abs()*self.adjacency).amax((1,2))
        centered = self.pos-self.pos.mean(1,keepdim=True)
        target = self.template-self.template.mean(1,keepdim=True)
        template_error = (centered-target).norm(dim=-1).amax(-1)
        y_span = self.pos[...,1].amax(-1)-self.pos[...,1].amin(-1)
        return (edge_error<.1)&torch.where(self.shape==0,y_span<.15,template_error<.1)

    def step(self, acceleration):
        if self.ended.any():
            raise RuntimeError('Reset every ended team before stepping')
        if acceleration.shape != self.vel.shape or not torch.isfinite(acceleration).all():
            raise ValueError('Expected finite normalized acceleration [E,5,2]')
        old_distance = (self.goal-self.pos).norm(dim=-1)
        velocity = (self.vel+self.dt*acceleration.clamp(-1,1)).clamp(-1,1)
        candidate = self.pos+self.dt*velocity
        clearance = self.clearances(self.pos,candidate)
        collision = (clearance<=0).any(-1)
        # Failed proposals are recorded and terminal. No post-impact learning.
        self.pos = torch.where(collision[:,None,None],self.pos,candidate)
        self.vel = torch.where(collision[:,None,None],self.vel,velocity)
        self.steps += 1
        distance = (self.goal-self.pos).norm(dim=-1)
        error = self.formation_error()
        # Same soft formation preference everywhere. No map-derived target.
        reward = 4*(old_distance-distance)
        reward -= self.dt*(.05+self.config.formation_weight*error.clamp_max(2))
        # Dense terminal precision without a task phase or a dwell bonus that
        # could make delaying team success more profitable than terminating.
        reward -= self.dt*self.config.goal_distance_weight*distance.clamp_max(1.)
        # Braking near a final goal is task shaping, with no discrete phase.
        reward -= self.dt*.3*torch.exp(-distance/.4)*self.vel.square().sum(-1)
        settled = ((distance<.3)&(self.vel.norm(dim=-1)<.15)).all(-1)&self.final_formation_ok()&~collision
        self.hold = torch.where(settled,self.hold+1,0)
        success = (self.hold>=10)&~collision
        reward += 5*success[:,None]-10*collision[:,None]
        terminated = collision|success
        truncated = (self.steps>=self.config.horizon)&~terminated
        self.ended = terminated|truncated
        self.episode_return += reward.mean(-1)
        info = {'success':success,'collision':collision,'clearance':clearance,
                'goal_distance':distance.mean(-1),'formation_error':error.mean(-1),
                'episode_return':self.episode_return.clone(),'steps':self.steps.clone()}
        return reward, terminated, truncated, info

    def state_dict(self):
        state = {'config':asdict(self.config),'sensors':asdict(self.sensors),
                'rng':self.rng.get_state(),
                'tensors':{k:getattr(self,k).clone() for k in self.state_fields}}
        state.update({
            'scenario_config': asdict(self.scenario_config),
            'perturb_rng': self.perturb_rng.get_state().clone(),
            'jitter_amplitude': self.jitter_amplitude.clone(),
            'initial_jitter': self.initial_jitter.clone(),
            'reset_attempts': self.reset_attempts.clone(),
            'perturbation_counters': dict(self.perturbation_counters),
        })
        return state

    def load_state_dict(self, state):
        if state.get('scenario_config') != asdict(self.scenario_config):
            raise ValueError('Initial-state perturbation resume configuration differs')
        if state['config'] != asdict(self.config) or state['sensors'] != asdict(self.sensors):
            raise ValueError('Environment resume configuration differs')
        for key in self.state_fields:
            setattr(self,key,state['tensors'][key].to(self.device).clone())
        self.rng.set_state(state['rng'].cpu())
        self.perturb_rng.set_state(state['perturb_rng'].cpu())
        for name in ('jitter_amplitude', 'initial_jitter', 'reset_attempts'):
            setattr(self, name, state[name].to(self.device).clone())
        self.perturbation_counters = dict(state['perturbation_counters'])
