"""Train the current recipe, validate periodically and test the final actor."""
import argparse
import contextlib
from dataclasses import asdict
import json
from pathlib import Path
import torch
from afor_local.environment import TaskConfig
from afor_local.policy import PolicyConfig
from afor_local.train import train, PPOConfig, source_hash
from afor_local.evaluate import evaluate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--formation', required=True, choices=('line', 'circle', 'wedge'))
    parser.add_argument('--resume')
    parser.add_argument('--updates', type=int, default=500)
    parser.add_argument('--every', type=int, default=50)
    parser.add_argument('--seed', type=int, default=2)
    parser.add_argument('--envs', type=int, default=16)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--passage-width', type=float, default=1.2)
    parser.add_argument('--validation-episodes', type=int, default=16)
    parser.add_argument('--test-episodes', type=int, default=128, help='Per final-test seed')
    args = parser.parse_args()
    if min(args.updates, args.every, args.validation_episodes, args.test_episodes) < 1:
        parser.error('Updates, intervals and episode counts must be positive')
    torch.set_num_threads(1)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if not args.resume and any((output/name).exists() for name in ('study.json', 'latest.pt')):
        raise FileExistsError('Choose a new output or explicitly resume')
    task = TaskConfig(num_envs=args.envs, formation=args.formation, passage_width=args.passage_width)
    ppo, policy = PPOConfig(), PolicyConfig()
    resume = args.resume
    start = torch.load(resume, map_location='cpu', weights_only=False)['update'] if resume else 0
    if args.updates <= start:
        raise ValueError('Requested total updates must exceed the resumed update')
    meta = {'task': asdict(task), 'ppo': asdict(ppo), 'policy': asdict(policy), 'seed': args.seed,
            'source_hash': source_hash(), 'validation_seed': 41001,
            'validation_episodes_per_case': args.validation_episodes,
            'final_test_seeds': [51001, 51002, 51003], 'target_updates': args.updates,
            'selection': 'Fixed final actor; validation never selects the delivered checkpoint'}
    history = json.loads((output/'validation.json').read_text()) if (output/'validation.json').exists() else []
    if history and history[-1]['update'] > start:
        raise ValueError('Validation history is newer than the requested resume checkpoint')
    (output/'study.json').write_text(json.dumps(meta, indent=2)+'\n')
    while start < args.updates:
        stop = min(args.updates, (start//args.every+1)*args.every)
        with (output/'train.log').open('a') as log, contextlib.redirect_stdout(log):
            model = train(output, stop, seed=args.seed, device=args.device,
                          task=task, ppo=ppo, policy=policy, resume=resume)
        report, _ = evaluate(model.actor, task.dt, 'narrowing', args.formation,
                             args.validation_episodes, 41001, passage_width=task.passage_width)
        row = {'update': stop, 'success_rate': report['successes']/report['episodes'],
               'episodes': report['episodes'], 'collisions': report['collisions'],
               'timeouts': report['timeouts'], 'cases': [report]}
        history.append(row)
        (output/'validation.json').write_text(json.dumps(history, indent=2)+'\n')
        print(json.dumps({k: v for k, v in row.items() if k != 'cases'}), flush=True)
        start, resume = stop, output/'latest.pt'
    report = {'formation': args.formation, 'selection': 'Final update, no test selection', 'cases': []}
    for seed in meta['final_test_seeds']:
        case, trace = evaluate(model.actor, task.dt, 'narrowing', args.formation,
                               args.test_episodes, seed, trajectory=seed == meta['final_test_seeds'][0],
                               passage_width=task.passage_width)
        report['cases'].append(case)
        if trace is not None:
            torch.save(trace, output/'final_test_trace.pt')
        (output/'final_test.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: sum(c[key] for c in report['cases'])
                      for key in ('episodes', 'successes', 'collisions', 'timeouts')}))


if __name__ == '__main__':
    main()
