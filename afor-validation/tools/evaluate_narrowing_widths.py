"""Reuse fixed actors across opening widths; never train or select checkpoints."""
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import hashlib
import json
from pathlib import Path
import torch
from afor_local.evaluate import evaluate
from afor_local.policy import load_actor
from afor_local.train import source_hash

ROOT = Path(__file__).resolve().parents[1]


def run_model(job):
    entry, models, protocol, destination = job
    destination = Path(destination)
    torch.set_num_threads(1)
    actor_path = Path(models)/entry['checkpoint']
    digest = hashlib.sha256(actor_path.read_bytes()).hexdigest()
    if digest != entry['actor_sha256']:
        raise RuntimeError('Fixed actor identity mismatch')
    actor, dt = load_actor(actor_path)
    formation = entry['formation']
    report = {'formation': formation, 'training_seed': entry['training_seed'],
              'update': entry['update'], 'actor': str(actor_path), 'actor_sha256': digest,
              'evaluation_source_hash': source_hash(), 'cases': []}
    for width in protocol['widths']:
        for seed in protocol['test_seeds']:
            case, trace = evaluate(actor, dt, 'narrowing', formation, protocol['episodes_per_case'],
                                   seed, 750, seed == protocol['test_seeds'][0], width)
            case['passage_width'] = width
            report['cases'].append(case)
            if trace is not None:
                torch.save(trace, destination/f'{formation}_width{width:g}.pt')
            (destination/f'{formation}.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        cases = [c for c in report['cases'] if c['passage_width'] == width]
        print(json.dumps({'formation': formation, 'width': width,
                          **{k: sum(c[k] for c in cases) for k in
                             ('episodes', 'successes', 'collisions', 'timeouts', 'bypass_completions')}}), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models', type=Path, default=ROOT/'models')
    parser.add_argument('--output', type=Path, default=ROOT/'runs/widths')
    parser.add_argument('--widths', type=float, nargs='+', default=[1.1, 1.2, 1.3, 1.4, 1.5, 1.6])
    parser.add_argument('--seeds', type=int, nargs='+', default=[58001, 58002, 58003])
    parser.add_argument('--episodes', type=int, default=32, help='Per width and environment seed')
    parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args()
    if min(args.episodes, args.workers) < 1 or any(not 1.1 <= w <= 1.6 for w in args.widths):
        parser.error('Positive episode/worker counts and widths in [1.1, 1.6] required')
    models, destination = args.models.resolve(), args.output.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    if any(destination.iterdir()):
        raise FileExistsError('Choose an empty output directory to preserve evidence')
    manifest = json.loads((models/'manifest.json').read_text())
    protocol = {'scope': 'Original narrowing corridor; only the upper opening changes',
                'actors': 'Fixed manifest actors trained at 1.2m, reused for every width',
                'models': manifest['models'], 'widths': args.widths, 'test_seeds': args.seeds,
                'episodes_per_case': args.episodes, 'horizon': 750, 'filter': 'none',
                'acceptance': 'Opening crossing, final goal, shape, stopping and 10-step dwell; independent float64 swept oracle'}
    (destination/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    jobs = [(entry, str(models), protocol, str(destination)) for entry in manifest['models']]
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_model, job) for job in jobs]):
            report = future.result()
            for width in args.widths:
                cases = [c for c in report['cases'] if c['passage_width'] == width]
                results = [r for c in cases for r in c['results']]
                rows.append({'formation': report['formation'], 'training_seed': report['training_seed'],
                             'width': width, 'actor_sha256': report['actor_sha256'],
                             **{k: sum(c[k] for c in cases) for k in
                                ('episodes', 'successes', 'collisions', 'timeouts', 'corridor_passes', 'bypass_completions')},
                             'min_pair_wall_boundary_clearance_m': [min(r['min_clearance'][i] for r in results) for i in range(3)]})
            rows.sort(key=lambda r: (r['formation'], r['width']))
            (destination/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')


if __name__ == '__main__':
    main()
