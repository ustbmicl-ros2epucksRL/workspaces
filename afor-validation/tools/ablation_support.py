"""Research-only controls reusing the current policy, PPO and physical task.

No production defaults are changed. The neighbour control removes information
from the deployed actor while keeping the training critic and braking teacher fixed.
"""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import torch
from torch import nn
from afor_local.policy import LocalActor, LocalFeatures, PolicyConfig, ActorCritic, SCHEMA
from afor_local.environment import TaskConfig, LocalFormationEnv
from afor_local.observation import SensorConfig
from afor_local.train import PPOConfig, collect, update, source_hash
from afor_local.evaluate import independent_clearances

VARIANTS = {
    'full': {},
    'no_neighbours': {'neighbours': False},
    'no_braking': {'barrier_weight': 0.},
    'shared_value': {'shared_value': True},
    'no_goal_cost': {'goal_distance_weight': 0.},
}
LABELS = {
    'full': 'Full', 'no_neighbours': 'No neighbour input',
    'no_braking': 'No braking loss', 'shared_value': 'Shared value encoder',
    'no_goal_cost': 'No goal-distance cost',
}
CN_LABELS = {
    'full': '完整方案', 'no_neighbours': '去邻居输入',
    'no_braking': '去制动损失', 'shared_value': '共享价值编码器',
    'no_goal_cost': '去目标距离代价',
}


def network_observation(obs, control):
    """Do not mutate raw measurements used by the common geometric teacher."""
    if not control.get('neighbours', True):
        return {**obs, 'mask': torch.zeros_like(obs['mask']),
                'neighbours': torch.zeros_like(obs['neighbours'])}
    return obs


class ControlledActor(LocalActor):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig(), control=None):
        super().__init__(config, sensors)
        self.control = dict(control or {})

    def features(self, obs, state, reset=None):
        return super().features(network_observation(obs, self.control), state, reset)


class ControlledModel(ActorCritic):
    def __init__(self, control):
        # Construct in exactly the production RNG order, then replace classes
        # without new initialization or parameter draws.
        super().__init__()
        self.actor.__class__ = ControlledActor
        self.actor.control = dict(control)
        self.control = dict(control)
        if control.get('shared_value'):
            self.critic = nn.Identity()

    def initial_state(self, batch_shape=()):
        if self.control.get('shared_value'):
            memory = self.actor.initial_state(batch_shape)
            return torch.cat((memory, memory.clone()), -1)
        return super().initial_state(batch_shape)

    def features(self, obs, state, reset=None):
        if self.control.get('shared_value'):
            features, memory = self.actor.features(obs, state[..., :64], reset)
            return features, features, torch.cat((memory, memory), -1)
        return super().features(obs, state, reset)


def research_hash():
    root = Path(__file__).resolve().parent
    digest = hashlib.sha256(source_hash().encode())
    for name in ('ablation_support.py', 'run_ablations.py'):
        digest.update(name.encode())
        digest.update((root/name).read_bytes())
    return digest.hexdigest()


def save_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    tmp.replace(path)


def export_control(model, path, dt, variant):
    torch.save({'schema': SCHEMA, 'kind': 'ablation_actor', 'variant': variant,
                'control': model.control, 'policy': asdict(model.actor.config),
                'sensors': asdict(model.actor.sensors), 'dt': dt,
                'state_dict': model.actor.state_dict()}, path)


def load_control(path):
    saved = torch.load(path, map_location='cpu', weights_only=False)
    if saved.get('kind') != 'ablation_actor':
        raise ValueError('Expected an explicitly labelled ablation actor')
    actor = ControlledActor(PolicyConfig(**saved['policy']), SensorConfig(**saved['sensors']), saved['control'])
    actor.load_state_dict(saved['state_dict'])
    return actor.eval(), saved['dt']


@torch.no_grad()
def evaluate_control(actor, formation, width, seed, episodes=32, trajectory=False):
    """Production acceptance, with all-episode motion/shape diagnostics added."""
    env = LocalFormationEnv(TaskConfig(num_envs=episodes, formation=formation, passage_width=width),
                            actor.sensors, seed=seed)
    dt = env.dt
    memory = actor.initial_state((episodes, 5))
    finished = torch.zeros(episodes, dtype=torch.bool)
    crossed = torch.zeros(episodes, 5, dtype=torch.bool)
    minima = np.full((episodes, 3), np.inf)
    totals = {k: np.zeros(episodes) for k in ('path_m', 'shape_integral_m_s', 'acceleration_square_integral', 'jerk_square_integral')}
    peak_shape = np.zeros(episodes)
    previous_action = torch.zeros_like(env.vel)
    rows = [None]*episodes
    trace = None
    if trajectory:
        trace = {'task': asdict(env.config), 'sensors': asdict(actor.sensors), 'seed': seed,
                 'bounds': env.bounds.clone(), 'goal': env.goal[:4].clone(),
                 'vertices': env.vertices[:4].clone(), 'obstacle_mask': env.obstacle_mask[:4].clone(),
                 'template': env.template[:4].clone(), 'position': [env.pos[:4].clone()],
                 'velocity': [env.vel[:4].clone()], 'proposal': [], 'active': []}
    for _ in range(env.config.horizon):
        action, memory = actor(env.observe(), memory)
        velocity = (env.vel+dt*action).clamp(-1, 1)
        candidate = env.pos+dt*velocity
        crossing = (env.pos[..., 1]<=1.3)&(candidate[..., 1]>1.3)
        fraction = ((1.3-env.pos[..., 1])/(candidate[..., 1]-env.pos[..., 1]).clamp_min(1e-12)).clamp(0, 1)
        crossing_x = env.pos[..., 0]+fraction*(candidate[..., 0]-env.pos[..., 0])
        at_opening = crossing_x.abs()<width/2-env.radius
        oracle = independent_clearances(env, env.pos, candidate)
        active = ~finished
        active_np = active.numpy()
        start = env.pos.clone()
        previous_velocity = env.vel.clone()
        _, term, trunc, info = env.step(action)
        np.testing.assert_allclose(info['clearance'].numpy(), oracle, atol=1e-8, rtol=1e-7)
        if ((velocity-previous_velocity).abs()>dt+1e-6).any() or (velocity.abs()>1+1e-6).any():
            raise AssertionError('Actuator limit mismatch')
        minima[active_np] = np.minimum(minima[active_np], oracle[active_np])
        crossed |= crossing&at_opening&active[:, None]&~info['collision'][:, None]
        shape = info['formation_error'].numpy()
        quantities = {
            'path_m': (env.pos-start).norm(dim=-1).mean(-1).numpy(),
            'shape_integral_m_s': dt*shape,
            'acceleration_square_integral': dt*action.square().sum(-1).mean(-1).numpy(),
            'jerk_square_integral': dt*((action-previous_action)/dt).square().sum(-1).mean(-1).numpy(),
        }
        for key, value in quantities.items(): totals[key][active_np] += value[active_np]
        peak_shape[active_np] = np.maximum(peak_shape[active_np], shape[active_np])
        previous_action = action.clone()
        if trace is not None:
            trace['position'].append(env.pos[:4].clone())
            trace['velocity'].append(env.vel[:4].clone())
            trace['proposal'].append(candidate[:4].clone())
            trace['active'].append(active[:4].clone())
        done = term|trunc
        for index in (done&active).nonzero().flatten().tolist():
            duration = int(info['steps'][index])*dt
            passed = bool(crossed[index].all())
            collision = bool(info['collision'][index])
            distance = (env.goal[index]-env.pos[index]).norm(dim=-1)
            speed = env.vel[index].norm(dim=-1)
            shape_ok = bool(env.final_formation_ok()[index])
            rows[index] = {
                'episode': index, 'success': bool(info['success'][index]) and passed,
                'goal_success': bool(info['success'][index]), 'corridor_passed': passed,
                'collision': collision, 'timeout': bool(trunc[index]),
                'steps': int(info['steps'][index]), 'duration_s': duration,
                'mean_goal_distance_m': float(distance.mean()), 'max_goal_distance_m': float(distance.max()),
                'max_speed_m_s': float(speed.max()), 'final_shape_ok': shape_ok,
                'final_formation_error_m': float(info['formation_error'][index]),
                'mean_formation_error_m': float(totals['shape_integral_m_s'][index]/duration),
                'peak_formation_error_m': float(peak_shape[index]),
                'mean_robot_path_m': float(totals['path_m'][index]),
                'rms_command_acceleration_m_s2': float(np.sqrt(totals['acceleration_square_integral'][index]/duration)),
                'rms_command_jerk_m_s3': float(np.sqrt(totals['jerk_square_integral'][index]/duration)),
                'min_clearance_m': minima[index].tolist(),
                'collision_causes': {k: bool(collision and minima[index, j]<=0) for j, k in enumerate(('pair','wall','boundary'))},
                'timeout_causes': {'distance': bool(trunc[index] and (distance>=.3).any()),
                                   'speed': bool(trunc[index] and (speed>=.15).any()),
                                   'shape': bool(trunc[index] and not shape_ok)},
            }
        finished |= done
        if finished.all(): break
        ids = done.nonzero().flatten()
        env.reset(ids)
        memory = torch.where(done[:, None, None], 0., memory)
        previous_action = torch.where(done[:, None, None], 0., previous_action)
    if not all(row is not None for row in rows): raise AssertionError('Missing episode')
    if trace is not None:
        for key in ('position', 'velocity', 'proposal', 'active'): trace[key] = torch.stack(trace[key])
        trace['episode_results'] = rows[:4]
    return {'formation': formation, 'width_m': width, 'environment_seed': seed,
            'episodes': episodes, 'successes': sum(r['success'] for r in rows),
            'collisions': sum(r['collision'] for r in rows), 'timeouts': sum(r['timeout'] for r in rows),
            'bypasses': sum(r['goal_success'] and not r['corridor_passed'] for r in rows),
            'results': rows}, trace


def run_training_job(job):
    variant, formation, seed, root, protocol = job
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    root = Path(root)
    output = root/variant/formation/f'seed_{seed}'
    output.mkdir(parents=True, exist_ok=True)
    control = VARIANTS[variant]
    task = TaskConfig(formation=formation, formation_weight=control.get('formation_weight', .15),
                      goal_distance_weight=control.get('goal_distance_weight', 1.))
    ppo = PPOConfig(barrier_weight=control.get('barrier_weight', 1.))
    config = {'variant': variant, 'control': control, 'formation': formation, 'seed': seed,
              'task': asdict(task), 'ppo': asdict(ppo), 'research_hash': research_hash(),
              'production_source_hash': source_hash(), 'updates': protocol['updates']}
    if (output/'complete.json').exists():
        complete = json.loads((output/'complete.json').read_text())
        if complete['config'] != config: raise ValueError(f'Completed configuration mismatch: {output}')
        return {'job': f'{variant}/{formation}/seed_{seed}', 'status': 'already_complete'}
    torch.manual_seed(seed)
    env = LocalFormationEnv(task, seed=seed+1000)
    model = ControlledModel(control)
    optimizer = torch.optim.Adam(model.parameters(), lr=ppo.lr, eps=1e-5)
    state = model.initial_state((task.num_envs, 5))
    starts = torch.ones(task.num_envs, 5, dtype=torch.bool)
    start = 0
    if (output/'latest.pt').exists():
        saved = torch.load(output/'latest.pt', weights_only=False)
        if saved['config'] != config: raise ValueError(f'Resume mismatch: {output}')
        model.load_state_dict(saved['model']); optimizer.load_state_dict(saved['optimizer'])
        env.load_state_dict(saved['env']); state, starts = saved['state'], saved['starts']
        torch.set_rng_state(saved['torch_rng']); start = saved['update']
        lines = [json.loads(line) for line in (output/'metrics.jsonl').read_text().splitlines()]
        (output/'metrics.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in lines if r['update']<=start))
    save_json(output/'config.json', {**config, 'actor_parameters': sum(p.numel() for p in model.actor.parameters()),
                                   'training_parameters': sum(p.numel() for p in model.parameters())})
    history = json.loads((output/'validation.json').read_text()) if (output/'validation.json').exists() else []
    history = [r for r in history if r['update']<=start]
    began = time.monotonic()
    for iteration in range(start+1, protocol['updates']+1):
        before = time.monotonic()
        data, state, starts, episodes = collect(model, env, state, starts, ppo)
        metrics = update(model, optimizer, data, ppo, task.dt)
        metrics.update(update=iteration, seconds=time.monotonic()-before,
                       team_transitions=iteration*task.num_envs*ppo.steps,
                       completed_episodes=len(episodes))
        for key in ('success','collision','return','length','goal_distance','formation_error'):
            metrics[key] = sum(e[key] for e in episodes)/len(episodes) if episodes else None
        with (output/'metrics.jsonl').open('a') as stream: stream.write(json.dumps(metrics)+'\n')
        if iteration%protocol['validation_every']==0 or iteration==protocol['updates']:
            # Evaluation consumes no sampling RNG. Preserve it explicitly anyway.
            rng = torch.get_rng_state()
            report, _ = evaluate_control(model.actor, formation, 1.2, protocol['validation_seed'], protocol['validation_episodes'])
            torch.set_rng_state(rng)
            history.append({'update': iteration, **{k:report[k] for k in ('episodes','successes','collisions','timeouts','bypasses')}})
            save_json(output/'validation.json', history)
            temporary = output/'latest.tmp'
            torch.save({'config': config, 'update': iteration, 'model': model.state_dict(),
                        'optimizer': optimizer.state_dict(), 'env': env.state_dict(),
                        'state': state, 'starts': starts, 'torch_rng': torch.get_rng_state()}, temporary)
            temporary.replace(output/'latest.pt')
            export_control(model, output/'actor.pt', task.dt, variant)
            save_json(output/'progress.json', {'update': iteration, 'validation': history[-1],
                                              'training_elapsed_s': time.monotonic()-began})
    evaluation = {'config': config, 'actor_sha256': hashlib.sha256((output/'actor.pt').read_bytes()).hexdigest(), 'cases': []}
    for width in protocol['widths_m']:
        for env_seed in protocol['test_seeds']:
            report, trace = evaluate_control(model.actor, formation, width, env_seed,
                                             protocol['episodes_per_case'],
                                             trajectory=env_seed==protocol['test_seeds'][0] and width in (1.1,1.2,1.6))
            evaluation['cases'].append(report)
            if trace is not None:
                traces = output/'traces'; traces.mkdir(exist_ok=True)
                torch.save(trace, traces/f'width_{width:.1f}.pt')
            save_json(output/'evaluation.json', evaluation)
    save_json(output/'complete.json', {'config': config, 'training_and_evaluation_s': time.monotonic()-began,
                                      'actor_sha256': evaluation['actor_sha256'],
                                      'episodes': sum(c['episodes'] for c in evaluation['cases']),
                                      'successes': sum(c['successes'] for c in evaluation['cases'])})
    return {'job': f'{variant}/{formation}/seed_{seed}', 'status': 'complete',
            'successes': sum(c['successes'] for c in evaluation['cases']),
            'episodes': sum(c['episodes'] for c in evaluation['cases'])}
