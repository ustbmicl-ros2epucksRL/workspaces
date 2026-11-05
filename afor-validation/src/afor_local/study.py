"""Registered, restartable three-formation G study with isolated subprocesses."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import zipfile

import torch

from .artifacts import file_hash, save_json, source_hash, utc_timestamp
from .config import PolicyConfig, PPOConfig, ScenarioConfig, SelectionConfig
from .environment import TaskConfig
from .observation import SensorConfig
from .protocols import FORMATIONS, TRAINING_SEEDS, UPDATES, case_specs


def _recipe(formations):
    from .__main__ import study_plan
    plan = study_plan()
    plan['entries'] = [item for item in plan['entries'] if item['formation'] in formations]
    plan['training_runs'] = len(plan['entries'])
    plan['task'] = asdict(TaskConfig())
    plan['sensors'] = asdict(SensorConfig())
    plan['selection']['enabled'] = True
    plan['source_hash'] = source_hash()
    return plan


def register_study(output, formations=FORMATIONS):
    """Freeze the source and legal paired initial states before training starts."""
    formations = tuple(formations)
    if not formations or len(set(formations)) != len(formations) or any(f not in FORMATIONS for f in formations):
        raise ValueError('Choose unique registered formations')
    root = Path(output).resolve()
    recipe = _recipe(formations)
    encoded = json.dumps(recipe, sort_keys=True, separators=(',', ':')).encode()
    recipe_hash = hashlib.sha256(encoded).hexdigest()
    manifest_path = root / 'study.json'
    if manifest_path.exists():
        study = json.loads(manifest_path.read_text())
        if study.get('recipe_sha256') != recipe_hash or study.get('recipe') != recipe:
            raise ValueError('Study configuration/source changed; use a new output directory')
        if file_hash(root / 'source_snapshot.zip') != study['source_snapshot_sha256']:
            raise ValueError('Frozen source snapshot has changed')
        for entry in study['case_files'].values():
            if file_hash(root / entry['path']) != entry['sha256']:
                raise ValueError('Frozen test fixtures have changed')
        return study
    root.mkdir(parents=True, exist_ok=True)
    if any(root.iterdir()):
        raise FileExistsError('A new study requires an empty output directory')
    from .evaluate import build_case_set
    case_files = {}
    for formation in formations:
        entries = [{'spec': spec, 'fixture': build_case_set(.05, formation,
                    passage_width=spec['passage_width'], jitter=spec['jitter'],
                    seed=spec['seed'], episodes=spec['episodes'])} for spec in case_specs()]
        destination = root / 'cases' / f'{formation}.json'
        save_json(destination, {'schema': 'afor_g_registered_cases_v1', 'formation': formation,
                                'cases': entries})
        case_files[formation] = {'path': f'cases/{formation}.json', 'sha256': file_hash(destination)}
    source_root = Path(__file__).resolve().parent
    project = source_root.parents[1]
    snapshot = root / 'source_snapshot.zip'
    with zipfile.ZipFile(snapshot, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source_root.rglob('*.py')):
            archive.write(path, path.relative_to(project))
        for name in ('pyproject.toml', 'configs/protocol.json', 'docs/EXPERIMENT_PROTOCOL.md', 'docs/METHOD.md'):
            path = project / name
            if path.is_file():
                archive.write(path, name)
    study = {'schema': 'afor_g_study_v1', 'method': 'G', 'status': 'registered',
             'registered_at': utc_timestamp(), 'recipe_sha256': recipe_hash, 'recipe': recipe,
             'source_hash': recipe['source_hash'], 'source_snapshot_sha256': file_hash(snapshot),
             'case_files': case_files,
             'entries': [{**item, 'status': 'pending'} for item in recipe['entries']],
             'evaluation': 'One frozen validation-selected actor per training seed; no test-based checkpoint selection'}
    save_json(manifest_path, study)
    return study


def evaluate_run(root, formation, seed):
    """Test a frozen actor; checkpoint choice is already fixed in run.json."""
    from .checkpoints import load_actor
    from .evaluate import evaluate_case
    from .reporting import audit_run
    root = Path(root).resolve()
    study = json.loads((root / 'study.json').read_text())
    case_path = root / study['case_files'][formation]['path']
    if source_hash() != study['source_hash'] or file_hash(case_path) != study['case_files'][formation]['sha256']:
        raise ValueError('Source or test fixtures changed after registration')
    directory = root / formation / f'seed_{seed}'
    run = json.loads((directory / 'run.json').read_text())
    if (run.get('status') != 'completed' or run['completed_updates'] != UPDATES
            or run['config']['task']['formation'] != formation or run['config']['seed'] != seed
            or run['source_hash'] != study['source_hash']
            or file_hash(directory / 'actor.pt') != run['actor_sha256']):
        raise ValueError('Training is incomplete or differs from the registered actor')
    fixtures = json.loads(case_path.read_text())['cases']
    if (directory / 'evaluation.json').is_file():
        return audit_run(directory, fixtures)
    actor, dt = load_actor(directory / 'actor.pt')
    progress_path = root / '.evaluation' / f'{formation}_{seed}.json'
    cases = []
    if progress_path.exists():
        partial = json.loads(progress_path.read_text())
        if partial.get('actor_sha256') != run['actor_sha256'] or partial.get('source_hash') != study['source_hash']:
            raise ValueError('Evaluation recovery record describes a different actor')
        cases = partial['cases']
        if len(cases) > len(fixtures) or any(c['case_hash'] != item['fixture']['case_hash'] for c, item in zip(cases, fixtures)):
            raise ValueError('Evaluation recovery cases differ from registration')
    for item in fixtures[len(cases):]:
        spec, fixture = item['spec'], item['fixture']
        report, _ = evaluate_case(actor, dt, formation, passage_width=spec['passage_width'],
                                  jitter=spec['jitter'], seed=spec['seed'], episodes=spec['episodes'],
                                  horizon=run['config']['task']['horizon'], case_set=fixture)
        report['test_family'] = spec['family']
        cases.append(report)
        save_json(progress_path, {'actor_sha256': run['actor_sha256'],
                                 'source_hash': study['source_hash'], 'cases': cases})
    save_json(directory / 'evaluation.json', {'split': 'test', 'actor_sha256': run['actor_sha256'],
                                             'source_hash': study['source_hash'], 'cases': cases})
    result = audit_run(directory, fixtures)
    progress_path.unlink(missing_ok=True)
    return result


def recommend_from_validation(root):
    """Pick one deployment candidate per formation using validation scores only."""
    from .selection import selection_key
    root = Path(root)
    study = json.loads((root / 'study.json').read_text())
    recommendations = {}
    for formation in study['case_files']:
        candidates = []
        for entry in study['entries']:
            if entry['formation'] != formation:
                continue
            directory = root / formation / f"seed_{entry['seed']}"
            run = json.loads((directory / 'run.json').read_text())
            if (run.get('status') != 'completed' or run.get('completed_updates') != UPDATES
                    or run.get('source_hash') != study['source_hash']
                    or file_hash(directory / 'actor.pt') != run.get('actor_sha256')):
                raise ValueError('Recommendations require completed, frozen registered actors')
            selection = run['selection']
            # Stable tie-break: earlier selected update, then smaller train seed.
            key = (*selection_key(selection['best_score']), -selection['selected_update'], -entry['seed'])
            candidates.append((key, directory, run))
        _, directory, run = max(candidates, key=lambda item: item[0])
        recommendations[formation] = {'source_actor': str(directory / 'actor.pt'),
                                      'actor_sha256': run['actor_sha256'],
                                      'training_seed': run['config']['seed'],
                                      'selected_update': run['selection']['selected_update'],
                                      'validation_score': run['selection']['best_score']}
    result = {'schema': 'afor_g_recommended_models_v1', 'method': 'G',
              'selection': 'Validation only; independent test scores do not participate in model ranking',
              'source_hash': study['source_hash'], 'formations': recommendations}
    destination = root / 'recommended_models.json'
    if destination.exists():
        if json.loads(destination.read_text()) != result:
            raise ValueError('Frozen validation recommendations changed; preserve the existing study')
    else:
        save_json(destination, result)
    return result


def run_study(output, workers=3, formations=FORMATIONS):
    """Run the registered 500-update recipes; repeat to resume unfinished runs."""
    if type(workers) is not int or workers < 1:
        raise ValueError('workers must be a positive integer')
    torch.set_num_threads(1)
    root = Path(output).resolve()
    study = register_study(root, formations)
    study.update(status='training', started_at=study.get('started_at', utc_timestamp()), workers=workers)
    save_json(root / 'study.json', study)
    lock = threading.Lock()
    package_root = str(Path(__file__).resolve().parents[1])
    environment = {**os.environ, 'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1',
                   'OPENBLAS_NUM_THREADS': '1', 'PYTHONUNBUFFERED': '1',
                   'PYTHONPATH': package_root + os.pathsep + os.environ.get('PYTHONPATH', '')}
    log_root = root / '.logs'
    log_root.mkdir(exist_ok=True)

    def record(entry, **changes):
        with lock:
            if changes.get('status') != 'failed':
                entry.pop('error', None)
                entry.pop('log_tail', None)
                entry.pop('failed_phase', None)
            entry.update(changes)
            save_json(root / 'study.json', study)

    def execute(entry, phase):
        formation, seed = entry['formation'], entry['seed']
        directory = root / formation / f'seed_{seed}'
        manifest = directory / 'run.json'
        logfile = log_root / f'{formation}_{seed}.log'
        try:
            run = json.loads(manifest.read_text()) if manifest.exists() else None
            if run and (run['source_hash'] != study['source_hash'] or run['config']['seed'] != seed
                        or run['config']['task'] != asdict(TaskConfig(formation=formation))
                        or run['config']['policy'] != asdict(PolicyConfig())
                        or run['config'].get('sensors') != asdict(SensorConfig())
                        or run['config']['ppo'] != asdict(PPOConfig())
                        or run['config']['scenario'] != asdict(ScenarioConfig())
                        or run['config'].get('selection') != asdict(SelectionConfig(enabled=True))
                        or run['planned_updates'] != UPDATES):
                raise ValueError('Existing training run differs from the frozen recipe')
            if phase == 'train' and (not run or run['status'] != 'completed'):
                record(entry, status='training', training_started_at=utc_timestamp())
                command = [sys.executable, '-m', 'afor_local', 'train', '--formation', formation,
                           '--seed', str(seed), '--updates', str(UPDATES), '--output', str(directory),
                           '--skip-evaluation']
                if run:
                    command.append('--resume')
                with logfile.open('a') as stream:
                    subprocess.run(command, env=environment, stdout=stream, stderr=subprocess.STDOUT, check=True)
            run = json.loads(manifest.read_text())
            if (run.get('status') != 'completed' or run.get('completed_updates') != UPDATES
                    or file_hash(directory / 'actor.pt') != run.get('actor_sha256')):
                raise ValueError('Training has not produced a completed, frozen actor')
            if phase == 'train':
                record(entry, status='trained', selected_update=run['selection']['selected_update'],
                       actor_sha256=run['actor_sha256'], training_completed_at=utc_timestamp())
                return True
            if (entry.get('actor_sha256') != run['actor_sha256']
                    or entry.get('selected_update') != run['selection']['selected_update']
                    or file_hash(root / 'recommended_models.json') != study['recommendations_sha256']):
                raise ValueError('Actor identity or validation recommendation changed before testing')
            record(entry, status='testing', selected_update=run['selection']['selected_update'],
                   actor_sha256=run['actor_sha256'], testing_started_at=utc_timestamp())
            with logfile.open('a') as stream:
                subprocess.run([sys.executable, '-m', 'afor_local.study', 'evaluate', str(root), formation, str(seed)],
                               env=environment, stdout=stream, stderr=subprocess.STDOUT, check=True)
            record(entry, status='completed', completed_at=utc_timestamp())
            logfile.unlink(missing_ok=True)
            return True
        except Exception as error:
            tail = '\n'.join(logfile.read_text().splitlines()[-8:]) if logfile.exists() else ''
            record(entry, status='failed', failed_phase=phase,
                   error=f'{type(error).__name__}: {error}', log_tail=tail)
            return False

    def run_phase(phase, completed_status):
        with ThreadPoolExecutor(max_workers=min(workers, len(study['entries']))) as pool:
            futures = [pool.submit(execute, entry, phase) for entry in study['entries']]
            for future in as_completed(futures):
                future.result()
                done = sum(entry['status'] == completed_status for entry in study['entries'])
                print(json.dumps({'phase': phase, 'completed': done, 'total': len(study['entries']),
                                  'time': utc_timestamp()}), flush=True)
        if any(entry['status'] != completed_status for entry in study['entries']):
            study.update(status='failed', failed_phase=phase, finished_at=utc_timestamp())
            save_json(root / 'study.json', study)
            raise RuntimeError('Some runs failed; inspect study.json and retained logs, then repeat to resume')

    # Freeze every selected actor and deployment recommendation before opening
    # the independent test split. A failed training run never starts any test.
    run_phase('train', 'trained')
    try:
        recommend_from_validation(root)
        recommendation_hash = file_hash(root / 'recommended_models.json')
        if study.get('recommendations_sha256', recommendation_hash) != recommendation_hash:
            raise ValueError('Frozen validation recommendation file has changed')
    except Exception:
        study.update(status='failed', failed_phase='recommendation', finished_at=utc_timestamp())
        save_json(root / 'study.json', study)
        raise
    study.update(status='testing', recommendations_sha256=recommendation_hash,
                 test_started_at=study.get('test_started_at', utc_timestamp()))
    study.pop('failed_phase', None)
    save_json(root / 'study.json', study)
    run_phase('test', 'completed')
    from .reporting import summarize
    summary = summarize(root, root / 'analysis' / 'report.json', root / 'analysis' / 'summary.png')
    study.update(status='completed', finished_at=utc_timestamp(),
                 completed_runs=summary['completed_runs'], test_episodes=summary['test_episodes'])
    save_json(root / 'study.json', study)
    for folder in (root / '.logs', root / '.evaluation'):
        if folder.exists() and not any(folder.iterdir()):
            folder.rmdir()
    return summary


if __name__ == '__main__':
    if len(sys.argv) != 5 or sys.argv[1] != 'evaluate':
        raise SystemExit('Internal usage: python -m afor_local.study evaluate ROOT FORMATION SEED')
    torch.set_num_threads(1)
    evaluate_run(sys.argv[2], sys.argv[3], int(sys.argv[4]))
