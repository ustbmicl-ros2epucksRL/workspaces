"""Train, evaluate, export and summarize the local recurrent graph policy."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

import torch

from .environment import TaskConfig
from .observation import SensorConfig
from .config import PPOConfig, PolicyConfig, ScenarioConfig, SelectionConfig
from .artifacts import file_hash, save_json, source_hash
from .protocols import (FORMATIONS, TRAINING_SEEDS, VALIDATION_SEEDS, TEST_SEEDS,
                        ANALYSIS_SEEDS, UPDATES, TEST_EPISODES, case_specs)


def study_plan():
    entries = [{'formation': formation, 'seed': seed}
               for formation in FORMATIONS for seed in TRAINING_SEEDS]
    return {'protocol': 'afor_local_g_v1', 'training_runs': len(entries), 'updates': UPDATES,
            'task': asdict(TaskConfig()), 'sensors': asdict(SensorConfig()),
            'policy': asdict(PolicyConfig()), 'ppo': asdict(PPOConfig()),
            'scenario': asdict(ScenarioConfig()), 'selection': asdict(SelectionConfig(enabled=True)),
            'splits': {'training': list(TRAINING_SEEDS), 'validation': list(VALIDATION_SEEDS),
                       'test': list(TEST_SEEDS), 'analysis': list(ANALYSIS_SEEDS)},
            'evaluation': {'episodes_per_case': TEST_EPISODES, 'cases': case_specs()},
            'entries': entries}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--threads', type=int, default=1)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('train', help='Train one graph policy from its random seed')
    p.add_argument('--formation', choices=FORMATIONS, default='line')
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--updates', type=int, default=UPDATES)
    p.add_argument('--output')
    p.add_argument('--device', default='cpu')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--keep-resume', action='store_true')
    p.add_argument('--checkpoint-every', type=int, default=25)
    p.add_argument('--progress-every', type=int, default=10)
    p.add_argument('--eval-episodes', type=int, default=TEST_EPISODES)
    p.add_argument('--skip-evaluation', action='store_true')
    p.add_argument('--num-envs', type=int, default=16)
    p.add_argument('--steps', type=int, default=128)
    p.add_argument('--sequence', type=int, default=16)
    p.add_argument('--epochs', type=int, default=4)
    p.add_argument('--sequences-per-batch', type=int, default=64)
    p.add_argument('--no-select-best', action='store_true', help='Keep only the terminal actor')
    p.add_argument('--selection-interval', type=int, default=25)
    p.add_argument('--selection-episodes', type=int, default=24)
    p = commands.add_parser('evaluate', help='Paired first-episode evaluation with independent geometry')
    p.add_argument('actor')
    p.add_argument('--output', required=True)
    p.add_argument('--formation', required=True, choices=FORMATIONS)
    p.add_argument('--split', choices=('validation', 'test', 'analysis'), default='test')
    p.add_argument('--seeds', type=int, nargs='+')
    p.add_argument('--episodes', type=int, default=TEST_EPISODES)
    p.add_argument('--horizon', type=int, default=750)
    p.add_argument('--device', default='cpu')
    p.add_argument('--widths', type=float, nargs='+', default=[1.2])
    p.add_argument('--jitters', type=float, nargs='+', default=[0., .05, .10, .15, .20])
    p.add_argument('--trace', action='store_true', help='Save only the first condition, episode0 trace')
    p = commands.add_parser('export', help='Export a standalone scripted actor with embedded metadata')
    p.add_argument('actor')
    p.add_argument('--output', required=True)
    p = commands.add_parser('study', help='Inspect or execute the registered 15-run study')
    p.add_argument('--output', default='results/g_study')
    p.add_argument('--plan-file')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--workers', type=int, default=3)
    p.add_argument('--formations', choices=FORMATIONS, nargs='+')
    p = commands.add_parser('summarize', help='Write a compact multi-seed summary')
    p.add_argument('root')
    p.add_argument('--output', required=True)
    p.add_argument('--figure')
    args = parser.parse_args(argv)
    if args.threads < 1:
        parser.error('--threads must be positive')
    torch.set_num_threads(args.threads)
    if args.command == 'train':
        from .train import train
        output = args.output or f'results/g_training/{args.formation}/seed_{args.seed}'
        train(output, updates=args.updates, seed=args.seed, device=args.device,
              task=TaskConfig(num_envs=args.num_envs, formation=args.formation),
              ppo=PPOConfig(steps=args.steps, sequence=args.sequence, epochs=args.epochs,
                            sequences_per_batch=args.sequences_per_batch),
              resume=args.resume, keep_resume=args.keep_resume, checkpoint_every=args.checkpoint_every,
              progress_every=args.progress_every, evaluate_final=not args.skip_evaluation,
              eval_episodes=args.eval_episodes,
              selection=SelectionConfig(not args.no_select_best, args.selection_interval, args.selection_episodes))
    elif args.command == 'evaluate':
        from .checkpoints import load_actor
        from .evaluate import evaluate_case
        destination = Path(args.output)
        if destination.exists() or args.trace and destination.with_suffix('.trace.pt').exists():
            raise FileExistsError('Evaluation output exists; choose a new destination to preserve evidence')
        actor, dt = load_actor(args.actor, args.device)
        seeds = args.seeds or {'validation': VALIDATION_SEEDS, 'test': TEST_SEEDS,
                              'analysis': ANALYSIS_SEEDS}[args.split]
        cases = []
        for width in args.widths:
            for jitter in args.jitters:
                for seed in seeds:
                    case, trace = evaluate_case(actor, dt, args.formation, width, jitter, seed,
                                               args.episodes, args.horizon, trace=args.trace and not cases)
                    cases.append(case)
                    if trace is not None:
                        from .artifacts import save_torch
                        save_torch(destination.with_suffix('.trace.pt'), trace)
        save_json(destination, {'split': args.split, 'actor_sha256': file_hash(args.actor),
                                'source_hash': source_hash(), 'cases': cases})
    elif args.command == 'export':
        from .checkpoints import load_actor, export_scripted_actor
        actor, _ = load_actor(args.actor)
        export_scripted_actor(actor, args.output)
    elif args.command == 'study':
        plan = study_plan()
        if args.formations:
            plan['entries'] = [entry for entry in plan['entries'] if entry['formation'] in args.formations]
            plan['training_runs'] = len(plan['entries'])
        if args.plan_file:
            save_json(args.plan_file, plan)
        if not args.execute:
            print(json.dumps(plan, ensure_ascii=False, indent=2))
            return
        from .study import run_study
        run_study(args.output, workers=args.workers, formations=args.formations or FORMATIONS)
    else:
        from .reporting import summarize
        print(json.dumps(summarize(args.root, args.output, args.figure), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
