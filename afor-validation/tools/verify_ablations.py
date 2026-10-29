"""Audit the registered ablation study without training or selecting models."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import torch
from ablation_support import save_json
from afor_local.environment import TaskConfig
from afor_local.train import PPOConfig, source_hash


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    if isinstance(value, dict):
        return all(finite(v) for v in value.values())
    if isinstance(value, list):
        return all(finite(v) for v in value)
    return not isinstance(value, float) or math.isfinite(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('results/ablations'))
    args = parser.parse_args()
    root = args.root.resolve()
    project = Path(__file__).resolve().parents[1]
    protocol = read(root/'protocol.json')['protocol']
    assert source_hash() == protocol['production_source_hash'], 'Production source changed'
    source = read(root/'source/manifest.json')
    for name, expected in source.items():
        assert sha(root/'source'/name) == expected, name
    digest = hashlib.sha256(protocol['production_source_hash'].encode())
    for name in ('ablation_support.py', 'run_ablations.py'):
        relative = f'tools/{name}'
        assert relative in source, f'Missing frozen training source: {relative}'
        digest.update(name.encode())
        digest.update((root/'source'/relative).read_bytes())
    assert digest.hexdigest() == protocol['research_hash'], 'Frozen training source hash mismatch'
    assert protocol['variants'] == {
        'full': {},
        'no_braking': {'barrier_weight': 0.0},
        'no_neighbours': {'neighbours': False},
        'shared_value': {'shared_value': True},
        'no_goal_cost': {'goal_distance_weight': 0.0},
    }
    assert protocol['widths_m'] == [1.1, 1.2, 1.3, 1.4]
    expected_jobs = {
        (v, f, s) for v in protocol['variants']
        for f in protocol['formations'] for s in protocol['training_seeds']}
    actual_jobs = {
        (p.parents[2].name, p.parents[1].name, int(p.parent.name.removeprefix('seed_')))
        for p in root.glob('*/*/seed_*/complete.json')}
    assert actual_jobs == expected_jobs, f'Missing/extra jobs: {actual_jobs ^ expected_jobs}'
    total_episodes = 0
    run_results = []
    for variant, form, seed in sorted(expected_jobs):
        run = root/variant/form/f'seed_{seed}'
        config = read(run/'config.json')
        control = protocol['variants'][variant]
        expected_config = {
            'variant': variant, 'control': control, 'formation': form, 'seed': seed,
            'task': asdict(TaskConfig(formation=form,
                formation_weight=control.get('formation_weight', .15),
                goal_distance_weight=control.get('goal_distance_weight', 1.))),
            'ppo': asdict(PPOConfig(barrier_weight=control.get('barrier_weight', 1.))),
            'research_hash': protocol['research_hash'],
            'production_source_hash': protocol['production_source_hash'],
            'updates': protocol['updates']}
        assert {k: config[k] for k in expected_config} == expected_config, str(run)
        assert config['actor_parameters'] == 35140
        assert config['training_parameters'] == (35205 if control.get('shared_value') else 70213)
        done = read(run/'complete.json')
        report = read(run/'evaluation.json')
        assert done['config'] == report['config'] == expected_config
        assert done['actor_sha256'] == report['actor_sha256'] == sha(run/'actor.pt')
        metrics = [json.loads(x) for x in (run/'metrics.jsonl').read_text().splitlines()]
        assert [x['update'] for x in metrics] == list(range(1, protocol['updates']+1))
        assert finite(metrics) and finite(report)
        history = read(run/'validation.json')
        updates = list(range(protocol['validation_every'], protocol['updates']+1,
                             protocol['validation_every']))
        if not updates or updates[-1] != protocol['updates']:
            updates.append(protocol['updates'])
        assert [x['update'] for x in history] == updates
        assert all(x['episodes'] == protocol['validation_episodes'] for x in history)
        saved = torch.load(run/'latest.pt', map_location='cpu', weights_only=False)
        actor = torch.load(run/'actor.pt', map_location='cpu', weights_only=False)
        assert saved['update'] == protocol['updates'] and saved['config'] == expected_config
        assert actor['variant'] == variant and actor['control'] == control
        for key, value in saved['model'].items():
            assert torch.isfinite(value).all(), (run, key)
        for key, value in actor['state_dict'].items():
            assert torch.equal(value, saved['model']['actor.'+key]), (run, key)
        cases = {(c['width_m'], c['environment_seed']) for c in report['cases']}
        assert len(cases) == len(report['cases']) == len(protocol['widths_m'])*len(protocol['test_seeds'])
        assert cases == {(w, s) for w in protocol['widths_m'] for s in protocol['test_seeds']}
        for case in report['cases']:
            rows = case['results']
            assert len(rows) == case['episodes'] == protocol['episodes_per_case']
            assert [r['episode'] for r in rows] == list(range(len(rows)))
            assert sum(case[k] for k in ('successes', 'collisions', 'timeouts', 'bypasses')) == len(rows)
            for name, field in [('successes', 'success'), ('collisions', 'collision'), ('timeouts', 'timeout')]:
                assert case[name] == sum(r[field] for r in rows)
            for row in rows:
                assert row['duration_s'] == row['steps']*config['task']['dt']
                if row['success']:
                    assert row['goal_success'] and row['corridor_passed'] and row['final_shape_ok']
                    assert not row['collision'] and not row['timeout']
                    assert row['max_goal_distance_m'] < .3 and row['max_speed_m_s'] < .15
                    assert min(row['min_clearance_m']) > 0
        assert done['episodes'] == sum(c['episodes'] for c in report['cases'])
        assert done['successes'] == sum(c['successes'] for c in report['cases'])
        traces = sorted((run/'traces').glob('width_*.pt'))
        if seed == protocol['training_seeds'][0]:
            assert {float(path.stem.removeprefix('width_')) for path in traces} == set(protocol['trajectory_widths_m'])
            for width in protocol['trajectory_widths_m']:
                trace = torch.load(run/'traces'/f'width_{width:.1f}.pt', weights_only=False)
                assert trace['seed'] == protocol['test_seeds'][0]
                assert trace['task']['passage_width'] == width
                case = next(c for c in report['cases'] if c['width_m'] == width
                            and c['environment_seed'] == trace['seed'])
                assert trace['episode_results'] == case['results'][:4]
        else:
            assert not traces, f'Unexpected trajectory files: {run}'
        total_episodes += done['episodes']
        run_results.append({'variant': variant, 'formation': form, 'seed': seed,
                            'episodes': done['episodes'], 'successes': done['successes']})
    baseline = {}
    for form in protocol['formations']:
        old = [json.loads(x) for x in (project/'results/training'/form/'metrics.jsonl').read_text().splitlines()]
        new = [json.loads(x) for x in (root/'full'/form/'seed_2/metrics.jsonl').read_text().splitlines()]
        keys = set(new[0]) & set(old[0]) - {'seconds'}
        assert len(old) == len(new)
        assert all(all(a[k] == b[k] for k in keys) for a, b in zip(old, new)), form
        retained = torch.load(project/'models'/form/'actor.pt', weights_only=False)
        fresh = torch.load(root/'full'/form/'seed_2/actor.pt', weights_only=False)
        assert retained['state_dict'].keys() == fresh['state_dict'].keys()
        assert all(torch.equal(v, fresh['state_dict'][k]) for k, v in retained['state_dict'].items()), form
        baseline[form] = {'updates_checked': len(new), 'exact_learning_metrics_match': True,
                          'actor_tensors_exactly_match_delivered_model': True}
    for item in read(project/'models/manifest.json')['models']:
        assert sha(project/'models'/item['checkpoint']) == item['actor_sha256']
    result = {'verified_utc': datetime.now(timezone.utc).isoformat(), 'status': 'passed',
              'completed_jobs': len(actual_jobs), 'total_training_updates': len(actual_jobs)*protocol['updates'],
              'total_evaluation_episodes': total_episodes, 'all_numeric_metrics_finite': True,
              'all_final_checkpoints_match_exported_actors': True,
              'all_configs_cases_seeds_and_trajectories_match_protocol': True,
              'frozen_source_files_verified': len(source), 'production_and_delivered_models_unchanged': True,
              'full_seed_2_reproduction': baseline, 'runs': run_results}
    save_json(root/'final_verification.json', result)
    print(json.dumps({k: v for k, v in result.items() if k not in ('runs', 'full_seed_2_reproduction')}, indent=2))


if __name__ == '__main__':
    main()
