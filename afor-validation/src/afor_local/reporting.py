"""Audit G-only evaluations and summarize independent training seeds."""
import json
import math
from pathlib import Path

import numpy as np

from .artifacts import file_hash, save_json
from .checkpoints import SCHEMA
from .protocols import FORMATIONS, TRAINING_SEEDS, UPDATES, case_specs


def outcome_summary(records):
    """Keep failures visible; short failed trajectories are not efficient runs."""
    successes = [row for row in records if row['success']]
    metrics = ('failure_penalized_duration', 'duration', 'mean_path_length',
               'mean_control_effort', 'mean_process_formation_error',
               'formation_error', 'edge_length_error', 'centered_template_error',
               'all_crossed_time', 'recovery_time')
    result = {'episodes': len(records), 'successes': len(successes),
              'success_rate': len(successes) / len(records) if records else None,
              'failure_counts': {name: sum(row['failure_type'] == name for row in records)
                                 for name in ('pair', 'wall', 'boundary', 'timeout', 'bypass', 'acceptance')}}
    for label, subset in (('all', records), ('success', successes),
                          ('failure', [row for row in records if not row['success']])):
        result[label] = {'episodes': len(subset)}
        for key in metrics:
            values = [row[key] for row in subset if row.get(key) is not None]
            result[label][key] = float(np.mean(values)) if values else None
    return result


def _case_key(case):
    return case['passage_width'], case['jitter'], case['seed'], case['episodes']


def audit_run(directory, fixtures=None):
    """Recount raw episodes, strict success acceptance and registered conditions."""
    directory = Path(directory)
    run = json.loads((directory / 'run.json').read_text())
    if (run.get('schema') != SCHEMA or run.get('implementation') != 'afor_local_g'
            or run.get('status') != 'completed'):
        raise ValueError(f'Not a completed G training run: {directory}')
    if file_hash(directory / 'actor.pt') != run.get('actor_sha256'):
        raise ValueError(f'Actor hash differs from run.json: {directory}')
    evaluation = json.loads((directory / 'evaluation.json').read_text())
    if (evaluation.get('split') != 'test' or evaluation.get('actor_sha256') != run['actor_sha256']
            or evaluation.get('source_hash') != run['source_hash']):
        raise ValueError(f'Test identity differs from retained actor/source: {directory}')
    task = run['config']['task']
    expected = {_case_key(spec): spec for spec in case_specs(task['passage_width'])}
    registered = {_case_key(item['spec']): item['fixture'] for item in fixtures or ()}
    if fixtures is not None and set(registered) != set(expected):
        raise ValueError('Registered fixtures do not cover the complete test protocol')
    seen, families, by_jitter = set(), {}, {}
    for case in evaluation['cases']:
        key = _case_key(case)
        if key not in expected or key in seen:
            raise ValueError(f'Unexpected or duplicate test condition: {directory}: {key}')
        seen.add(key)
        if (case['formation'] != task['formation'] or case['dt'] != task['dt']
                or case['horizon'] != task['horizon'] or not case['independent_swept_oracle']):
            raise ValueError(f'Test task or collision oracle differs: {directory}')
        if registered and case['case_hash'] != registered[key]['case_hash']:
            raise ValueError(f'Test case hash differs from registration: {directory}')
        records = case['results']
        if len(records) != case['episodes'] or [r['episode'] for r in records] != list(range(len(records))):
            raise ValueError(f'Missing or duplicate raw first episodes: {directory}')
        for row in records:
            if type(row['success']) is not bool or type(row['length']) is not int or not 1 <= row['length'] <= task['horizon']:
                raise ValueError('Malformed raw episode')
            if not math.isclose(row['duration'], row['length'] * task['dt'], abs_tol=1e-10):
                raise ValueError('Episode duration differs from its physical length')
            penalty = row['duration'] if row['success'] else task['horizon'] * task['dt']
            if not math.isclose(row['failure_penalized_duration'], penalty, abs_tol=1e-10):
                raise ValueError('Failure penalty was not applied consistently')
            if row['success'] and (row['collision'] or not row['corridor_passed']
                                   or not row['oracle_goal_success'] or not row['final_formation_ok']
                                   or not row['max_goal_distance'] < .3 or not row['max_speed'] < .15
                                   or not row['edge_length_error'] < .1
                                   or task['formation'] == 'line' and not row['final_y_span_m'] < .15
                                   or task['formation'] != 'line' and not row['centered_template_error'] < .1
                                   or row['hold'] < 10
                                   or not all(math.isfinite(value) and value > 0 for value in row['min_clearance'])
                                   or row['failure_type'] is not None):
                raise ValueError(f'Success violates independent acceptance: {directory}')
            for metric in ('mean_path_length', 'mean_control_effort', 'mean_process_formation_error'):
                if not math.isfinite(row[metric]) or row[metric] < 0:
                    raise ValueError(f'Invalid process metric {metric}')
        for name, flag in (('successes', 'success'), ('collisions', 'collision'), ('timeouts', 'timeout')):
            if type(case.get(name)) is not int or case[name] != sum(r[flag] for r in records):
                raise ValueError(f'Case {name} count differs from raw episodes')
        failures = {name: sum(row['failure_type'] == name for row in records)
                    for name in ('pair', 'wall', 'boundary', 'timeout', 'bypass', 'acceptance')}
        if (case.get('failure_counts') != failures
                or any(type(count) is not int for count in case['failure_counts'].values())):
            raise ValueError('Case failure counts differ from raw episodes')
        families.setdefault(expected[key]['family'], []).extend(records)
        if case['passage_width'] == task['passage_width']:
            by_jitter.setdefault(str(case['jitter']), []).extend(records)
    if seen != set(expected):
        raise ValueError(f'Incomplete registered test: {directory}')
    selection = run.get('selection', {})
    return {'path': str(directory.resolve()), 'formation': task['formation'],
            'seed': run['config']['seed'], 'updates': run['completed_updates'],
            'selected_update': selection.get('selected_update'),
            'source_hash': run['source_hash'], 'actor_sha256': run['actor_sha256'],
            'config': run['config'],
            'families': {name: outcome_summary(rows) for name, rows in families.items()},
            'by_jitter': {name: outcome_summary(rows) for name, rows in by_jitter.items()},
            'test_episodes': sum(item['episodes'] for item in evaluation['cases'])}


def _seed_stats(values):
    values = np.asarray(values, dtype=float)
    result = {'mean': float(values.mean()),
              'std': float(values.std(ddof=1)) if len(values) > 1 else None}
    if len(values) >= 3:
        rng = np.random.default_rng(20261105)
        indices = rng.integers(0, len(values), size=(10000, len(values)))
        result['ci95'] = np.quantile(values[indices].mean(1), [.025, .975]).tolist()
    return result


def summarize(root, output=None, figure=None):
    root = Path(root)
    study_path = root / 'study.json'
    study = json.loads(study_path.read_text()) if study_path.exists() else None
    rows = []
    for path in sorted(root.rglob('run.json')):
        run = json.loads(path.read_text())
        if (run.get('schema') != SCHEMA or run.get('implementation') != 'afor_local_g'
            or run.get('status') != 'completed'):
            continue
        if not path.with_name('evaluation.json').is_file():
            continue
        fixtures = None
        if study:
            formation = run['config']['task']['formation']
            fixture_path = root / 'cases' / f'{formation}.json'
            if file_hash(fixture_path) != study['case_files'][formation]['sha256']:
                raise ValueError('Registered case file has changed')
            fixtures = json.loads(fixture_path.read_text())['cases']
            if run['source_hash'] != study['source_hash']:
                raise ValueError('Training source differs from the registered source')
        rows.append(audit_run(path.parent, fixtures))
    identities = [(row['formation'], row['seed']) for row in rows]
    if len(set(identities)) != len(identities):
        raise ValueError('Duplicate independent training seed within a formation')
    groups = []
    for formation in FORMATIONS:
        group = [row for row in rows if row['formation'] == formation]
        if not group:
            continue
        recipes = {json.dumps({k: v for k, v in row['config'].items() if k != 'seed'}, sort_keys=True)
                   for row in group}
        if len(recipes) != 1 or len({r['source_hash'] for r in group}) != 1 or len({r['updates'] for r in group}) != 1:
            raise ValueError(f'Mixed training recipes for {formation}')
        families = {}
        for family in group[0]['families']:
            families[family] = {
                'success_rate': _seed_stats([r['families'][family]['success_rate'] for r in group]),
                'failure_penalized_duration': _seed_stats([
                    r['families'][family]['all']['failure_penalized_duration'] for r in group]),
                'episodes': sum(r['families'][family]['episodes'] for r in group),
                'successes': sum(r['families'][family]['successes'] for r in group),
            }
        jitter = {key: _seed_stats([r['by_jitter'][key]['success_rate'] for r in group])
                  for key in group[0]['by_jitter']}
        groups.append({'formation': formation, 'training_seeds': len(group),
                       'seeds': sorted(row['seed'] for row in group), 'families': families,
                       'by_jitter': jitter})
    expected = {(entry['formation'], entry['seed']) for entry in study['entries']} if study else {
        (formation, seed) for formation in FORMATIONS for seed in TRAINING_SEEDS}
    complete = set(identities) == expected and all(row['updates'] == UPDATES for row in rows)
    summary = {'schema': 'afor_g_report_v1', 'method': 'G',
               'status': 'complete' if complete else 'partial',
               'scope': 'Five independent training seeds per formation; validation-selected frozen actors. No external-baseline or new-mechanism claim.',
               'statistics': {'unit': 'independent training seed', 'std': 'sample standard deviation (ddof=1)',
                              'interval': '10000 seed percentile bootstrap; finite-sample descriptive interval',
                              'bootstrap_seed': 20261105},
               'completed_runs': len(rows), 'expected_runs': len(expected),
               'test_episodes': sum(row['test_episodes'] for row in rows),
               'per_run': rows, 'groups': groups}
    if output:
        save_json(output, summary)
    if figure:
        _plot(rows, groups, figure)
    return summary


def _plot(rows, groups, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), constrained_layout=True)
    colors = dict(zip(FORMATIONS, ('#0072B2', '#009E73', '#D55E00')))
    for formation in FORMATIONS:
        for row in [r for r in rows if r['formation'] == formation]:
            metrics = [json.loads(line) for line in (Path(row['path']) / 'metrics.jsonl').read_text().splitlines() if line.strip()]
            x, y = [], []
            for start in range(0, len(metrics), 25):
                window = [m for m in metrics[start:start + 25] if m.get('success') is not None]
                weight = sum(m['completed_episodes'] for m in window)
                if weight:
                    x.append(window[-1]['update'])
                    y.append(sum(m['success'] * m['completed_episodes'] for m in window) / weight)
            axes[0].plot(x, y, color=colors[formation], alpha=.35, lw=.8)
    for i, group in enumerate(groups):
        formation = group['formation']
        x = sorted(float(key) for key in group['by_jitter'])
        means = [group['by_jitter'][str(value)]['mean'] for value in x]
        std = [group['by_jitter'][str(value)]['std'] or 0 for value in x]
        axes[1].errorbar(x, means, yerr=std, marker='o', label=formation, color=colors[formation])
        score = group['families']['main_challenge']['failure_penalized_duration']
        axes[2].bar(i, score['mean'], yerr=score['std'] or 0, color=colors[formation], capsize=4)
    axes[0].set(xlabel='PPO update', ylabel='Training team success (25-update windows)', ylim=(-.02, 1.02))
    axes[1].set(xlabel='Initial longitudinal jitter (m)', ylabel='Test team success', ylim=(-.02, 1.08))
    if groups:
        axes[1].legend()
    axes[2].set(xticks=range(len(groups)), xticklabels=[g['formation'] for g in groups], ylabel='Challenge penalized duration (s)')
    fig.suptitle('G policy — three formations; error bars are training-seed sample standard deviations')
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    fig.savefig(path.with_suffix('.pdf'))
    plt.close(fig)
