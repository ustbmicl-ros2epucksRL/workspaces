"""Recurrent graph PPO with an independent critic and validation-only selection."""
from dataclasses import asdict
import json
import platform
import time
import numpy as np
import torch
from torch.nn import functional as F
from .barrier import local_braking_loss
from .environment import TaskConfig
from .observation import SensorConfig
from .probability import latent_log_prob
from .config import PPOConfig
from .ppo import advantages, sequence_view
from .artifacts import RunArtifacts, file_hash, save_json, save_torch, source_hash, tensor_state_hash, utc_timestamp
from .config import PolicyConfig, ScenarioConfig, SelectionConfig
from .environment import LocalFormationEnv
from .policy import ActorCritic
from .checkpoints import SCHEMA, export_actor, load_actor
from .selection import BestTracker
from .protocols import TEST_EPISODES, case_specs


@torch.no_grad()
def collect(model, env, state, starts, ppo):
    observations, states, resets, latents, logprobs = [], [], [], [], []
    rewards, values, next_values, terms, truncs = [], [], [], [], []
    episodes = []
    for _ in range(ppo.steps):
        obs = env.observe()
        observations.append(obs)
        states.append(state.clone())
        resets.append(starts.clone())
        action, latent, logp, value, following = model(obs, state, starts)
        reward, term, trunc, info = env.step(action)
        # GAE bootstraps the physical state before a time-limit reset.
        _, _, _, next_value, _ = model(env.observe(), following, deterministic=True)
        latents.append(latent); logprobs.append(logp)
        rewards.append(reward); values.append(value); next_values.append(next_value)
        terms.append(term); truncs.append(trunc)
        done = term | trunc
        for index in done.nonzero().flatten().tolist():
            episodes.append({'success': bool(info['success'][index]), 'collision': bool(info['collision'][index]),
                             'return': float(info['episode_return'][index]), 'length': int(info['steps'][index]),
                             'goal_distance': float(info['goal_distance'][index]),
                             'formation_error': float(info['formation_error'][index])})
        env.reset(done.nonzero().flatten())
        starts = done[:, None].expand(-1, env.n_agents)
        state = torch.where(starts[..., None], 0., following)
    obs = {key: torch.stack([o[key] for o in observations]) for key in observations[0]}
    resets = torch.stack(resets)
    reward, value, next_value, term, trunc = map(torch.stack, (rewards, values, next_values, terms, truncs))
    advantage, returns = advantages(reward, value, next_value, term, trunc, ppo.gamma, ppo.gae_lambda)
    data = {'obs': obs, 'state': torch.stack(states), 'reset': resets, 'latent': torch.stack(latents),
            'logprob': torch.stack(logprobs), 'advantage': advantage, 'returns': returns}
    return data, state, starts, episodes


def update(model, optimizer, data, ppo, dt):
    obs = {key: sequence_view(value, ppo.sequence) for key, value in data['obs'].items()}
    sequences = {key: sequence_view(value, ppo.sequence) for key, value in data.items() if key != 'obs'}
    adv = sequences['advantage']
    sequences['advantage'] = (adv - adv.mean()) / adv.std(unbiased=False).clamp_min(1e-8)
    totals, batches = {}, 0
    for _ in range(ppo.epochs):
        for ids in torch.randperm(len(adv), device=adv.device).split(ppo.sequences_per_batch):
            state = sequences['state'][ids, 0].detach()
            logprobs, predicted, entropies, brakes, attention_trace = [], [], [], [], []
            for t in range(ppo.sequence):
                current = {key: value[ids, t] for key, value in obs.items()}
                features, value_features, state, attention = model.features_with_attention(
                    current, state, sequences['reset'][ids, t])
                attention_trace.append(-(attention.detach() * attention.detach().clamp_min(1e-12).log()).sum(-1))
                distribution = model.actor.distribution(features)
                logprobs.append(latent_log_prob(distribution, sequences['latent'][ids, t]))
                predicted.append(model.value(value_features).squeeze(-1))
                entropies.append(-latent_log_prob(distribution, distribution.rsample()))
                if ppo.barrier_weight:
                    brakes.append(local_braking_loss(current, distribution.mean.tanh(), model.actor.sensors, dt))
            logprob = torch.stack(logprobs, 1)
            predicted = torch.stack(predicted, 1)
            entropy = torch.stack(entropies, 1).mean()
            logratio = logprob - sequences['logprob'][ids]
            ratio = logratio.exp()
            advantage = sequences['advantage'][ids]
            policy_loss = -torch.minimum(ratio * advantage,
                                         ratio.clamp(1 - ppo.clip, 1 + ppo.clip) * advantage).mean()
            value_loss = F.mse_loss(predicted, sequences['returns'][ids])
            brake_loss = torch.stack(brakes).mean() if brakes else policy_loss.new_zeros(())
            loss = policy_loss + .5 * value_loss - ppo.entropy * entropy + ppo.barrier_weight * brake_loss
            if not torch.isfinite(loss):
                raise FloatingPointError('Nonfinite graph PPO objective')
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            actor_norm = torch.nn.utils.clip_grad_norm_(model.actor.parameters(), .5, error_if_nonfinite=True)
            critic_norm = torch.nn.utils.clip_grad_norm_(list(model.critic.parameters()) + list(model.value.parameters()), .5, error_if_nonfinite=True)
            optimizer.step()
            metrics = {'policy_loss': policy_loss, 'value_loss': value_loss, 'entropy': entropy,
                       'braking_loss': brake_loss,
                       'approx_kl': ((ratio - 1) - logratio).mean(),
                       'clip_fraction': ((ratio - 1).abs() > ppo.clip).float().mean(),
                       'actor_grad_norm': actor_norm, 'critic_grad_norm': critic_norm,
                       'attention_entropy': torch.stack(attention_trace).mean()}
            for key, value in metrics.items():
                totals[key] = totals.get(key, 0.) + float(value.detach())
            batches += 1
    return {**{key: value / batches for key, value in totals.items()}, 'optimizer_steps': batches}


def train(output, updates=500, seed=11, device='cpu', task=TaskConfig(), policy=PolicyConfig(),
          sensors=SensorConfig(), ppo=PPOConfig(), scenario=ScenarioConfig(),
          resume=False, checkpoint_every=25, keep_resume=False,
          evaluate_final=False, eval_episodes=TEST_EPISODES, progress_every=10,
          selection=SelectionConfig()):
    if any(type(value) is not int or value < 1 for value in
           (updates, checkpoint_every, progress_every, eval_episodes)):
        raise ValueError('Training budgets, evaluation episodes and logging intervals must be positive integers')
    store = RunArtifacts(output, resume)
    torch.manual_seed(seed)
    env = LocalFormationEnv(task, sensors, device, seed + 1000, scenario)
    model = ActorCritic(policy, sensors, seed=seed).to(device)
    initialization = {'actor_sha256': tensor_state_hash(model.actor.state_dict()),
                      'critic_sha256': tensor_state_hash({
                          **{'critic.' + k: v for k, v in model.critic.state_dict().items()},
                          **{'value.' + k: v for k, v in model.value.state_dict().items()}}),
                      'critic_rule': 'independent original critic, forked seed + 100000'}
    runtime = {'python': platform.python_version(), 'numpy': np.__version__, 'torch': str(torch.__version__),
               'machine': platform.machine(), 'processor': platform.processor(),
               'torch_threads': torch.get_num_threads(), 'torch_interop_threads': torch.get_num_interop_threads(),
               'cuda': torch.version.cuda}
    optimizer = torch.optim.Adam(model.parameters(), lr=ppo.lr, eps=1e-5)
    selector = BestTracker(task.dt, task, selection) if selection.enabled else None
    state = model.initial_state((task.num_envs, env.n_agents))
    starts = torch.ones(task.num_envs, env.n_agents, device=device, dtype=torch.bool)
    config = {'task': asdict(task), 'policy': asdict(policy), 'sensors': asdict(sensors),
              'ppo': asdict(ppo), 'scenario': asdict(scenario), 'seed': seed}
    if selection.enabled:
        config['selection'] = asdict(selection)
    identity = source_hash()
    start, previous_elapsed = 0, 0.
    if resume:
        saved = torch.load(store.checkpoint, map_location=device, weights_only=False)
        if saved.get('schema') != SCHEMA or saved['config'] != config or saved['source_hash'] != identity:
            raise ValueError('Resume requires identical G config and source; use a new directory for a changed experiment')
        if saved['device'] != str(torch.device(device)):
            raise ValueError('Exact resume requires the same device')
        if saved.get('runtime') != runtime:
            raise ValueError('Exact resume requires the same dependency versions and thread/runtime settings')
        model.load_state_dict(saved['model']); optimizer.load_state_dict(saved['optimizer'])
        env.load_state_dict(saved['env'])
        state, starts = saved['state'], saved['starts']
        torch.set_rng_state(saved['torch_rng'].cpu())
        if selector is not None:
            selector.load_state_dict(saved['selection'])
        if str(device).startswith('cuda'):
            torch.cuda.set_rng_state_all([v.cpu() for v in saved['cuda_rng']])
        start, previous_elapsed = saved['update'], saved['elapsed_seconds']
        if updates < start:
            raise ValueError('Resume total updates cannot precede checkpoint update')
        if (selector is not None and updates > start and selector.last_update == start
                and start % selection.interval):
            raise ValueError('Extending an off-schedule final validation changes selection candidates; extend from a scheduled boundary or start a new run')
        store.rewind_metrics(start)
    record = {'implementation': 'afor_local_g', 'schema': SCHEMA, 'status': 'running', 'config': config,
              'source_hash': identity, 'device': str(torch.device(device)), 'torch': torch.__version__,
              'initialization': initialization, 'runtime': runtime,
              'planned_updates': updates, 'started_at': utc_timestamp(), 'completed_updates': start,
              'evidence_scope': 'G-only training; validation-selected weights evaluated on separate test seeds',
              'retention': 'one final actor, one metrics stream, one evaluation and manifest; rolling checkpoint removed on completion unless requested'}
    if selector is not None:
        record['selection'] = selector.summary()
        record['retention'] = 'one validation-selected actor, one terminal actor, one metrics stream, one evaluation and manifest; only one rolling checkpoint containing the selected payload during training'
    save_json(store.manifest, record)
    wall_start = time.perf_counter()
    io_seconds = 0.

    def checkpoint(iteration):
        nonlocal io_seconds
        began = time.perf_counter()
        save_torch(store.checkpoint, {'schema': SCHEMA, 'config': config, 'source_hash': identity,
            'runtime': runtime,
            'device': str(torch.device(device)), 'update': iteration, 'model': model.state_dict(),
            'optimizer': optimizer.state_dict(), 'env': env.state_dict(), 'state': state, 'starts': starts,
            'torch_rng': torch.get_rng_state(), 'cuda_rng': torch.cuda.get_rng_state_all() if str(device).startswith('cuda') else [],
            'selection': selector.state_dict() if selector is not None else None,
            'elapsed_seconds': previous_elapsed + time.perf_counter() - wall_start})
        io_seconds += time.perf_counter() - began

    try:
        if not resume:
            checkpoint(0)
        for iteration in range(start + 1, updates + 1):
            began = time.perf_counter()
            data, state, starts, episodes = collect(model, env, state, starts, ppo)
            rollout_s = time.perf_counter() - began
            began = time.perf_counter()
            metrics = update(model, optimizer, data, ppo, task.dt)
            metrics.update(update=iteration, rollout_seconds=rollout_s, update_seconds=time.perf_counter() - began,
                           team_transitions=iteration * task.num_envs * ppo.steps,
                           agent_transitions=iteration * task.num_envs * ppo.steps * env.n_agents,
                           completed_episodes=len(episodes))
            for key in ('success', 'collision', 'return', 'length', 'goal_distance', 'formation_error'):
                metrics[key] = sum(row[key] for row in episodes) / len(episodes) if episodes else None
            if selector is not None and selector.due(iteration, updates):
                validation_start = time.perf_counter()
                metrics['validation'] = selector.observe(model.actor, iteration)
                metrics['validation_seconds'] = time.perf_counter() - validation_start
                record['selection'] = selector.summary()
            began = time.perf_counter()
            store.append_metrics(metrics)
            io_seconds += time.perf_counter() - began
            record['completed_updates'] = iteration
            if iteration % checkpoint_every == 0 or iteration == updates:
                checkpoint(iteration)
                save_json(store.manifest, record)
            if iteration == start + 1 or iteration % progress_every == 0 or iteration == updates:
                print(json.dumps({key: metrics.get(key) for key in
                    ('update', 'success', 'collision', 'approx_kl', 'rollout_seconds', 'update_seconds')}, allow_nan=False), flush=True)
        began = time.perf_counter()
        if selector is not None:
            if selector.best_actor_payload is None:
                raise RuntimeError('Selection completed without a validated actor')
            export_actor(model, store.root / 'final_actor.pt', task.dt)
            save_torch(store.root / 'actor.pt', selector.best_actor_payload)
            record['selection'] = selector.summary()
            record['final_actor_sha256'] = file_hash(store.root / 'final_actor.pt')
        else:
            export_actor(model, store.root / 'actor.pt', task.dt)
        io_seconds += time.perf_counter() - began
        record.update(status='completed', finished_at=utc_timestamp(),
                      elapsed_seconds=previous_elapsed + time.perf_counter() - wall_start,
                      actor_sha256=file_hash(store.root / 'actor.pt'),
                      actor_parameters=model.actor.parameter_counts(),
                      critic_parameters=sum(p.numel() for p in model.critic.parameters()) + sum(p.numel() for p in model.value.parameters()))
        if evaluate_final:
            began = time.perf_counter()
            from .evaluate import evaluate_case
            evaluation_specs = case_specs(task.passage_width, episodes=eval_episodes)
            def evaluate_actor(actor):
                cases = []
                for spec in evaluation_specs:
                    report, _ = evaluate_case(actor.eval(), task.dt, task.formation,
                                             passage_width=spec['passage_width'], jitter=spec['jitter'],
                                             seed=spec['seed'], episodes=spec['episodes'], horizon=task.horizon)
                    report['test_family'] = spec['family']
                    cases.append(report)
                return cases
            selected_actor = load_actor(store.root / 'actor.pt', device)[0] if selector is not None else model.actor
            cases = evaluate_actor(selected_actor)
            endpoint = None
            if selector is not None:
                # Equal weights need only one deterministic evaluation. The
                # terminal actor remains separately retained for auditability.
                final_payload = torch.load(store.root / 'final_actor.pt', map_location='cpu', weights_only=True)
                same_actor = tensor_state_hash(final_payload['state_dict']) == selector.best_actor_state_sha256
                endpoint = {'actor_sha256': record['final_actor_sha256'], 'update': updates,
                            'cases': cases if same_actor else evaluate_actor(model.actor)}
            record['evaluation_seconds'] = time.perf_counter() - began
            began = time.perf_counter()
            evaluation = {'split': 'test',
                          'actor_sha256': record['actor_sha256'], 'source_hash': identity, 'cases': cases}
            if selector is not None:
                evaluation.update(selection=selector.summary(), endpoint=endpoint)
            save_json(store.root / 'evaluation.json', evaluation)
            io_seconds += time.perf_counter() - began
            record['evaluation'] = 'evaluation.json'
        else:
            # This known artifact can otherwise describe a pre-resume actor.
            (store.root / 'evaluation.json').unlink(missing_ok=True)
        record['elapsed_seconds'] = previous_elapsed + time.perf_counter() - wall_start
        record['io_seconds_this_session'] = io_seconds
        save_json(store.manifest, record)
        store.finish(keep_resume)
    except BaseException as error:
        record.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed', error=f'{type(error).__name__}: {error}')
        save_json(store.manifest, record)
        raise
    return model


def main(argv=None):
    """Use the unified command parser for this training entry."""
    import sys
    from .__main__ import main as command_main
    return command_main(['train', *(sys.argv[1:] if argv is None else argv)])


if __name__ == '__main__':
    main()
