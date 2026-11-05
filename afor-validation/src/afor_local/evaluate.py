"""Policy-independent paired fixtures and first-episode evaluation."""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

from .environment import LocalFormationEnv, TaskConfig, _draw_valid_y_jitter, _perturbation_seed
from .observation import SensorConfig
from .config import ScenarioConfig
from .collision_geometry import segment_polygon_distance


def independent_clearances(env, start, end):
    """NumPy float64 reference, separate from the training geometry kernel."""
    start, end = (x.detach().cpu().numpy().astype(np.float64) for x in (start,end))
    vertices = env.vertices.cpu().numpy().astype(np.float64)[:,None]
    masks = env.obstacle_mask.cpu().numpy()[:,None]
    i,j = np.triu_indices(env.n_agents,1)
    relative = start[:,i]-start[:,j]
    change = end[:,i]-end[:,j]-relative
    norm = (change*change).sum(-1)
    time = np.zeros_like(norm)
    np.divide(-(relative*change).sum(-1),norm,out=time,where=norm>1e-20)
    pair = np.linalg.norm(relative+time.clip(0,1)[...,None]*change,axis=-1).min(-1)-2*env.radius
    wall = np.where(masks,segment_polygon_distance(start,end,vertices),np.inf).min((1,2))-env.radius
    boundary = np.minimum(env.bounds.cpu().numpy()-np.abs(start),env.bounds.cpu().numpy()-np.abs(end)).min((1,2))-env.radius
    return np.stack((pair,wall,boundary),-1)


ACCEPTANCE = {
    'physics': 'Unchanged LocalFormationEnv; terminal swept collision, no rollback continuation',
    'oracle': 'Independent NumPy float64 swept pair/wall/boundary clearances',
    'team_success': 'All five cross the actual opening, reach assigned goals, recover shape and stop',
    'goal_distance_m': .3,
    'speed_m_s': .15,
    'edge_length_error_m': .1,
    'line_y_span_m': .15,
    'circle_wedge_centered_template_error_m': .1,
    'dwell_steps': 10,
    'strict_inequalities': True,
    'recovery': 'Time after all-cross to completion of 10 consecutive final-shape-valid frames',
    'selection': 'First episode in each environment only; no outcome-based selection',
}


def _case_hash(manifest):
    payload = {key: value for key, value in manifest.items() if key != 'case_hash'}
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


def build_case_set(dt, formation, passage_width=1.2, jitter=0., seed=183001,
                   episodes=32, horizon=750, max_rejections=100):
    """Return one JSON-ready manifest of fixed-amplitude, legal initial teams.

    Test jitter is U[-jitter,jitter] independently per robot, unlike the
    training mixture which first samples its amplitude. Sampling is on CPU and
    does not depend on actor weights, inference device or episode outcomes.
    """
    if not math.isfinite(jitter) or jitter < 0 or max_rejections < 1:
        raise ValueError('Jitter must be finite and nonnegative; rejection limit positive')
    config = TaskConfig(num_envs=episodes, formation=formation, passage_width=passage_width,
                        dt=dt, horizon=horizon)
    env = LocalFormationEnv(config, seed=seed, scenario=ScenarioConfig(0., 0.))
    initial = env.pos.clone()
    attempts = torch.zeros(episodes, dtype=torch.long)
    if jitter:
        generator = torch.Generator().manual_seed(_perturbation_seed(seed))
        _, attempts = _draw_valid_y_jitter(
            env, torch.arange(episodes), torch.full((episodes,), jitter),
            generator, max_rejections)
    clearance = independent_clearances(env, env.pos, env.pos)
    if not np.isfinite(clearance).all() or not (clearance > 0).all():
        raise AssertionError('Generated fixture has an invalid initial state')
    manifest = {
        'schema': 'afor_g_case_set_v1',
        'task': asdict(config), 'seed': int(seed), 'jitter': float(jitter),
        'sampling': 'Fixed base endpoints; independent uniform y jitter; reject entire invalid team',
        'max_rejections': int(max_rejections),
        'acceptance': dict(ACCEPTANCE),
        'cases': [
            {'episode': index, 'position': env.pos[index].tolist(),
             'goal': env.goal[index].tolist(),
             'y_offset': (env.pos[index, :, 1] - initial[index, :, 1]).tolist(),
             'rejections': max(0, int(attempts[index]) - 1)}
            for index in range(episodes)
        ],
    }
    manifest['case_hash'] = _case_hash(manifest)
    return manifest


def _final_geometry(env):
    """Independent float64 final shape, assigned-goal and stopped checks."""
    pos = env.pos.detach().cpu().numpy().astype(np.float64)
    goal = env.goal.detach().cpu().numpy().astype(np.float64)
    vel = env.vel.detach().cpu().numpy().astype(np.float64)
    template = env.template.detach().cpu().numpy().astype(np.float64)
    adjacency = env.adjacency.detach().cpu().numpy().astype(bool)
    delta = pos[:, None] - pos[:, :, None]
    desired = template[:, None] - template[:, :, None]
    edge_error = np.where(adjacency, np.abs(
        np.linalg.norm(delta, axis=-1) - np.linalg.norm(desired, axis=-1)), 0).max((1, 2))
    centered = pos - pos.mean(1, keepdims=True)
    target = template - template.mean(1, keepdims=True)
    template_error = np.linalg.norm(centered - target, axis=-1).max(-1)
    span = np.ptp(pos[..., 1], axis=-1)
    shape = (edge_error < .1) & (span < .15 if env.config.formation == 'line'
                                else template_error < .1)
    distance = np.linalg.norm(goal - pos, axis=-1)
    speed = np.linalg.norm(vel, axis=-1)
    return shape, distance, speed, edge_error, template_error


@torch.no_grad()
def evaluate_case(actor, dt, formation, passage_width=1.2, jitter=0., seed=183001,
                  episodes=32, horizon=750, trace=False, *, case_set=None):
    """Evaluate one frozen G actor on the recorded first-episode fixtures."""
    fixture = case_set if case_set is not None else build_case_set(
        dt, formation, passage_width, jitter, seed, episodes, horizon)
    if fixture.get('schema') != 'afor_g_case_set_v1' or fixture.get('case_hash') != _case_hash(fixture):
        raise ValueError('Invalid or modified paired case manifest')
    config = TaskConfig(num_envs=episodes, formation=formation, passage_width=passage_width,
                        dt=dt, horizon=horizon)
    if fixture['task'] != asdict(config) or fixture['seed'] != seed or fixture['jitter'] != jitter:
        raise ValueError('Paired case manifest configuration differs')
    if len(fixture['cases']) != episodes:
        raise ValueError('Paired case count differs')
    parameter = next(actor.parameters(), torch.empty(0))
    device = parameter.device
    sensors = getattr(actor, 'sensors', SensorConfig())
    env = LocalFormationEnv(config, sensors, device, seed,
                              ScenarioConfig(perturb_probability=0., max_jitter=0.))
    env.pos.copy_(torch.tensor([case['position'] for case in fixture['cases']], device=device))
    env.goal.copy_(torch.tensor([case['goal'] for case in fixture['cases']], device=device))
    if not (independent_clearances(env, env.pos, env.pos) > 0).all():
        raise ValueError('Paired fixture has colliding initial positions')
    state = actor.initial_state((episodes, env.n_agents))
    finished = torch.zeros(episodes, dtype=torch.bool, device=device)
    crossed = torch.zeros(episodes, env.n_agents, dtype=torch.bool, device=device)
    minima = np.full((episodes, 3), np.inf)
    oracle_hold = np.zeros(episodes, dtype=np.int64)
    shape_hold = np.zeros(episodes, dtype=np.int64)
    crossed_step = np.full(episodes, -1, dtype=np.int64)
    recovered_step = np.full(episodes, -1, dtype=np.int64)
    results = [None] * episodes
    path_length = np.zeros(episodes, dtype=np.float64)
    control_effort = np.zeros(episodes, dtype=np.float64)
    process_formation_error = np.zeros(episodes, dtype=np.float64)
    trace_data = None
    if trace:
        trace_data = {
            'task': asdict(config), 'sensors': asdict(sensors), 'seed': seed,
            'jitter': jitter, 'case_hash': fixture['case_hash'], 'selected_episode': 0,
            'bounds': env.bounds.cpu().clone(), 'goal': env.goal[:1].cpu().clone(),
            'vertices': env.vertices[:1].cpu().clone(),
            'obstacle_mask': env.obstacle_mask[:1].cpu().clone(),
            'template': env.template[:1].cpu().clone(),
            'position': [env.pos[:1].cpu().clone()],
            'velocity': [env.vel[:1].cpu().clone()], 'proposal': [], 'active': [],
        }
    for step in range(1, horizon + 1):
        active = ~finished
        active_np = active.cpu().numpy()
        action, state = actor(env.observe(), state)
        old_position = env.pos.clone()
        old_velocity = env.vel.clone()
        velocity = (env.vel + dt * action.clamp(-1, 1)).clamp(-1, 1)
        candidate = env.pos + dt * velocity
        crossing = (env.pos[..., 1] <= 1.3) & (candidate[..., 1] > 1.3)
        fraction = ((1.3 - env.pos[..., 1]) /
                    (candidate[..., 1] - env.pos[..., 1]).clamp_min(1e-12)).clamp(0, 1)
        crossing_x = env.pos[..., 0] + fraction * (candidate[..., 0] - env.pos[..., 0])
        at_opening = crossing_x.abs() < passage_width / 2 - env.radius
        oracle = independent_clearances(env, env.pos, candidate)
        _, terminated, truncated, info = env.step(action)
        np.testing.assert_allclose(info['clearance'].cpu().numpy(), oracle,
                                   atol=1e-8, rtol=1e-7)
        if ((velocity - old_velocity).abs() > dt + 1e-6).any() or (velocity.abs() > 1 + 1e-6).any():
            raise AssertionError('Actuator limits violated')
        collision_np = info['collision'].cpu().numpy()
        if not np.array_equal(collision_np, (oracle <= 0).any(-1)):
            raise AssertionError('Environment and independent collision verdicts differ')
        # The failed collision proposal is not executed by the physical model.
        # Accumulate only committed motion, and never dummy episodes after reset.
        displacement = (env.pos - old_position).double().norm(dim=-1).mean(-1).cpu().numpy()
        applied = ((env.vel - old_velocity).double() / dt).square().sum(-1).mean(-1).cpu().numpy()
        path_length[active_np] += displacement[active_np]
        control_effort[active_np] += dt * applied[active_np]
        process_formation_error[active_np] += info['formation_error'].cpu().numpy()[active_np]
        crossed |= crossing & at_opening & active[:, None] & ~info['collision'][:, None]
        all_crossed = crossed.all(-1).cpu().numpy()
        new_cross = active_np & all_crossed & (crossed_step < 0)
        crossed_step[new_cross] = step
        shape, distance, speed, edge_error, template_error = _final_geometry(env)
        settled = (distance < .3).all(-1) & (speed < .15).all(-1) & shape & ~collision_np
        oracle_hold[active_np] = np.where(settled[active_np], oracle_hold[active_np] + 1, 0)
        recovering = active_np & all_crossed
        shape_hold[recovering] = np.where(shape[recovering], shape_hold[recovering] + 1, 0)
        recovered = recovering & (shape_hold >= 10) & (recovered_step < 0)
        recovered_step[recovered] = step
        minima[active_np] = np.minimum(minima[active_np], oracle[active_np])
        if trace_data is not None and bool(active[0]):
            trace_data['position'].append(env.pos[:1].cpu().clone())
            trace_data['velocity'].append(env.vel[:1].cpu().clone())
            trace_data['proposal'].append(candidate[:1].cpu().clone())
            trace_data['active'].append(active[:1].cpu().clone())
        done = terminated | truncated
        for index in (done & active).nonzero().flatten().tolist():
            success = bool(info['success'][index]) and bool(all_crossed[index]) and oracle_hold[index] >= 10
            failures = [name for axis, name in enumerate(('pair', 'wall', 'boundary'))
                        if oracle[index, axis] <= 0]
            reasons = []
            if not all_crossed[index]:
                reasons.append('passage')
            if not (distance[index] < .3).all():
                reasons.append('goal')
            if not shape[index]:
                reasons.append('shape')
            if not (speed[index] < .15).all():
                reasons.append('speed')
            if oracle_hold[index] < 10 and not reasons:
                reasons.append('hold')
            if failures:
                failure_type = failures[0]
            elif bool(truncated[index]):
                failure_type = 'timeout'
            elif not all_crossed[index]:
                failure_type = 'bypass'
            elif not success:
                failure_type = 'acceptance'
            else:
                failure_type = None
            length = int(info['steps'][index])
            results[index] = {
                'episode': index, 'success': bool(success),
                'goal_success': bool(info['success'][index]),
                'oracle_goal_success': bool(oracle_hold[index] >= 10 and not collision_np[index]),
                'corridor_passed': bool(all_crossed[index]),
                'collision': bool(collision_np[index]), 'collision_types': failures,
                'failure_type': failure_type, 'timeout': bool(truncated[index]),
                'timeout_reasons': reasons if bool(truncated[index]) else [],
                'length': length, 'duration': length * dt,
                'failure_penalized_duration': length * dt if success else horizon * dt,
                'goal_distance': float(distance[index].mean()),
                'max_goal_distance': float(distance[index].max()),
                'max_speed': float(speed[index].max()),
                'formation_error': float(info['formation_error'][index]),
                'mean_process_formation_error': float(process_formation_error[index] / length),
                'mean_path_length': float(path_length[index]),
                'mean_control_effort': float(control_effort[index]),
                'final_formation_ok': bool(shape[index]),
                'edge_length_error': float(edge_error[index]),
                'centered_template_error': float(template_error[index]),
                'final_y_span_m': float(np.ptp(env.pos[index, :, 1].cpu().numpy().astype(np.float64))),
                'hold': int(oracle_hold[index]), 'environment_hold': int(env.hold[index]),
                'return': float(info['episode_return'][index]),
                'min_clearance': [float(x) for x in minima[index]],
                'all_crossed_time': float(crossed_step[index] * dt) if crossed_step[index] >= 0 else None,
                'recovery_time': float((recovered_step[index] - crossed_step[index]) * dt)
                                 if recovered_step[index] >= 0 else None,
            }
        finished |= done
        if bool(finished.all()):
            break
        # The vectorized production env forbids stepping any ended team.
        # Reset dummy teams but never count or log their later episodes.
        env.reset(done.nonzero().flatten())
        state = torch.where(done[:, None, None], 0., state)
    if any(result is None for result in results):
        raise AssertionError('Evaluation failed to account for every first episode')
    if trace_data is not None:
        for key in ('position', 'velocity', 'proposal', 'active'):
            trace_data[key] = torch.stack(trace_data[key])
    failure_counts = {name: sum(r['failure_type'] == name for r in results)
                      for name in ('pair', 'wall', 'boundary', 'timeout', 'bypass', 'acceptance')}
    report = {
        'scenario': 'narrowing', 'formation': formation, 'seed': int(seed),
        'passage_width': passage_width, 'jitter': jitter, 'dt': dt, 'horizon': horizon,
        'episodes': episodes, 'case_hash': fixture['case_hash'],
        'acceptance': dict(ACCEPTANCE), 'independent_swept_oracle': True,
        'successes': sum(r['success'] for r in results),
        'collisions': sum(r['collision'] for r in results),
        'goal_successes': sum(r['goal_success'] for r in results),
        'corridor_passes': sum(r['corridor_passed'] for r in results),
        'bypass_completions': sum(r['goal_success'] and not r['corridor_passed'] for r in results),
        'timeouts': sum(r['timeout'] for r in results), 'failure_counts': failure_counts,
        'mean_goal_distance': float(np.mean([r['goal_distance'] for r in results])),
        'mean_formation_error': float(np.mean([r['formation_error'] for r in results])),
        'mean_duration': float(np.mean([r['duration'] for r in results])),
        'mean_failure_penalized_duration': float(np.mean([r['failure_penalized_duration'] for r in results])),
        'results': results,
    }
    metric_keys = ('duration', 'failure_penalized_duration', 'mean_path_length',
                   'mean_control_effort', 'mean_process_formation_error',
                   'formation_error', 'edge_length_error', 'centered_template_error',
                   'all_crossed_time', 'recovery_time')
    report['outcome_metrics'] = {}
    for label, subset in (('all', results),
                          ('success', [row for row in results if row['success']]),
                          ('failure', [row for row in results if not row['success']])):
        report['outcome_metrics'][label] = {'episodes': len(subset)}
        for key in metric_keys:
            values = [row[key] for row in subset if row[key] is not None]
            report['outcome_metrics'][label][key] = float(np.mean(values)) if values else None
    return report, trace_data


def evaluate(actor_or_path, dt=None, scenario='narrowing', formation='line',
             episodes=32, seed=183001, horizon=750, trajectory=False,
             passage_width=1.2, jitter=0., device='cpu', *, case_set=None):
    """Evaluate an exported canonical actor or an already constructed actor.

    Actor objects retain the familiar positional evaluation interface. Paths
    use the checkpoint's validated timestep and restricted canonical loader.
    """
    if scenario != 'narrowing':
        raise ValueError('Only the registered narrowing task is supported')
    if isinstance(actor_or_path, (str, Path)):
        from .checkpoints import load_actor
        actor, checkpoint_dt = load_actor(actor_or_path, device)
        if dt is not None and not math.isclose(dt, checkpoint_dt, rel_tol=0., abs_tol=1e-12):
            raise ValueError('Evaluation timestep differs from the actor checkpoint')
        dt = checkpoint_dt
    else:
        actor = actor_or_path
        dt = .05 if dt is None else dt
    return evaluate_case(actor, dt, formation, passage_width, jitter, seed,
                         episodes, horizon, trajectory, case_set=case_set)


def main(argv=None):
    """Use the canonical CLI so seeds, splits and trace retention stay uniform."""
    import sys
    from .__main__ import main as unified_main
    return unified_main(['evaluate', *(sys.argv[1:] if argv is None else argv)])


if __name__ == '__main__':
    main()
