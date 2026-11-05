"""Study registration, identities and recovery without launching training jobs."""
from dataclasses import asdict
import hashlib
import importlib
import json
import subprocess
import zipfile

import pytest

from afor_local.artifacts import file_hash, save_json
from afor_local.config import PolicyConfig, PPOConfig, ScenarioConfig, SelectionConfig
from afor_local.environment import TaskConfig
from afor_local.observation import SensorConfig
from afor_local.protocols import FORMATIONS, TRAINING_SEEDS, UPDATES, case_specs

study = importlib.import_module('afor_local.study')
evaluation = importlib.import_module('afor_local.evaluate')
reporting = importlib.import_module('afor_local.reporting')
checkpoints = importlib.import_module('afor_local.checkpoints')


@pytest.fixture
def registered(tmp_path, monkeypatch):
    """Use compact fixture payloads; real geometry is tested independently."""
    calls = []

    def fixture(dt, formation, **spec):
        value = {'dt': dt, 'formation': formation, **spec}
        calls.append(value)
        digest = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
        return {'case_hash': digest, 'parameters': value}

    monkeypatch.setattr(evaluation, 'build_case_set', fixture)
    monkeypatch.setattr(study, 'source_hash', lambda: 'frozen-source')
    return tmp_path / 'registered', calls


def _run(root, formation='line', seed=11, *, status='completed', duration=9., update=100):
    directory = root / formation / f'seed_{seed}'
    directory.mkdir(parents=True, exist_ok=True)
    actor = directory / 'actor.pt'
    actor.write_bytes(f'{formation}-{seed}-frozen-actor'.encode())
    record = {
        'implementation': 'afor_local_g', 'status': status, 'source_hash': 'frozen-source',
        'planned_updates': UPDATES, 'completed_updates': UPDATES if status == 'completed' else 25,
        'actor_sha256': file_hash(actor),
        'config': {'seed': seed, 'task': asdict(TaskConfig(formation=formation)),
                   'sensors': asdict(SensorConfig()),
                   'policy': asdict(PolicyConfig()), 'ppo': asdict(PPOConfig()),
                   'scenario': asdict(ScenarioConfig()),
                   'selection': asdict(SelectionConfig(enabled=True))},
        'selection': {'selected_update': update, 'best_score': {
            'nominal_successes': 48, 'nominal_episodes': 48,
            'challenge_successes': 144, 'challenge_episodes': 144,
            'mean_failure_penalized_duration': duration}},
    }
    save_json(directory / 'run.json', record)
    return record


@pytest.mark.parametrize('formations', [(), ('line', 'line'), ('unknown',)])
def test_registration_rejects_ambiguous_formations(registered, formations):
    root, _ = registered
    with pytest.raises(ValueError, match='unique registered formations'):
        study.register_study(root, formations)
    assert not root.exists()


def test_registration_freezes_complete_three_formation_cases_and_source(registered):
    root, calls = registered
    manifest = study.register_study(root)
    assert manifest['status'] == 'registered'
    assert manifest['recipe']['sensors'] == asdict(SensorConfig())
    assert {(e['formation'], e['seed']) for e in manifest['entries']} == {
        (formation, seed) for formation in FORMATIONS for seed in TRAINING_SEEDS}
    assert len(calls) == 3 * 23
    assert manifest['source_snapshot_sha256'] == file_hash(root / 'source_snapshot.zip')
    with zipfile.ZipFile(root / 'source_snapshot.zip') as snapshot:
        assert {'src/afor_local/policy.py', 'src/afor_local/study.py',
                'docs/EXPERIMENT_PROTOCOL.md', 'docs/METHOD.md'} <= set(snapshot.namelist())
    for formation, identity in manifest['case_files'].items():
        path = root / identity['path']
        assert file_hash(path) == identity['sha256']
        cases = json.loads(path.read_text())['cases']
        assert [item['spec'] for item in cases] == case_specs()
        assert all(item['fixture']['parameters']['formation'] == formation for item in cases)
        assert sum(item['spec']['episodes'] for item in cases) == 736
    before = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
    assert study.register_study(root) == manifest
    assert len(calls) == 3 * 23
    assert all(path.read_bytes() == contents for path, contents in before.items())


@pytest.mark.parametrize('target', ['source', 'snapshot', 'case', 'formations'])
def test_registration_reentry_rejects_changed_identity(registered, monkeypatch, target):
    root, _ = registered
    study.register_study(root)
    if target == 'source':
        monkeypatch.setattr(study, 'source_hash', lambda: 'different-source')
    elif target == 'snapshot':
        (root / 'source_snapshot.zip').write_bytes(b'changed snapshot')
    elif target == 'case':
        (root / 'cases' / 'circle.json').write_text('{}')
    with pytest.raises(ValueError):
        study.register_study(root, ('line',) if target == 'formations' else FORMATIONS)


def test_registration_preserves_unrelated_files_and_incomplete_registration(registered, monkeypatch):
    root, _ = registered
    root.mkdir()
    marker = root / 'unrelated.txt'
    marker.write_text('keep this evidence')
    with pytest.raises(FileExistsError, match='empty output'):
        study.register_study(root)
    assert marker.read_text() == 'keep this evidence'
    assert not (root / 'study.json').exists()


def test_recommendation_uses_validation_then_earlier_update_then_seed(registered):
    root, _ = registered
    study.register_study(root, ('line',))
    for seed in TRAINING_SEEDS:
        _run(root, seed=seed, duration=10., update=100)
        save_json(root / 'line' / f'seed_{seed}' / 'evaluation.json',
                  {'test_success': 1. if seed == 11 else 0., 'test_time': .01})
    _run(root, seed=23, duration=9., update=100)
    _run(root, seed=37, duration=9., update=75)
    _run(root, seed=51, duration=9., update=75)
    result = study.recommend_from_validation(root)
    assert result['formations']['line']['training_seed'] == 37
    assert result['formations']['line']['selected_update'] == 75
    # Test outcomes are deliberately contradictory and are never read.
    assert 'independent test' in result['selection']
    assert json.loads((root / 'recommended_models.json').read_text()) == result
    original = (root / 'recommended_models.json').read_bytes()
    assert study.recommend_from_validation(root) == result
    assert (root / 'recommended_models.json').read_bytes() == original
    _run(root, seed=11, duration=1., update=25)
    with pytest.raises(ValueError, match='Frozen validation recommendations changed'):
        study.recommend_from_validation(root)
    assert (root / 'recommended_models.json').read_bytes() == original


def test_evaluation_recovers_registered_prefix_and_audits_existing_output(registered, monkeypatch):
    root, _ = registered
    study.register_study(root, ('line',))
    _run(root)
    monkeypatch.setattr(checkpoints, 'load_actor', lambda path: (object(), .05))
    audits, calls = [], []

    def audit(directory, fixtures):
        rows = json.loads((directory / 'evaluation.json').read_text())['cases']
        assert [r['case_hash'] for r in rows] == [f['fixture']['case_hash'] for f in fixtures]
        audits.append(len(rows))
        return {'audited': len(rows)}

    def evaluate(actor, dt, formation, **kwargs):
        calls.append(kwargs['case_set']['case_hash'])
        if len(calls) == 2:
            raise RuntimeError('interrupted test process')
        return {'case_hash': kwargs['case_set']['case_hash']}, None

    monkeypatch.setattr(reporting, 'audit_run', audit)
    monkeypatch.setattr(evaluation, 'evaluate_case', evaluate)
    with pytest.raises(RuntimeError, match='interrupted test'):
        study.evaluate_run(root, 'line', 11)
    progress = root / '.evaluation' / 'line_11.json'
    assert len(json.loads(progress.read_text())['cases']) == 1
    assert not (root / 'line' / 'seed_11' / 'evaluation.json').exists()
    assert study.evaluate_run(root, 'line', 11) == {'audited': 23}
    assert not progress.exists()
    assert len(calls) == 24  # The interrupted condition is retried, not the completed prefix.
    assert study.evaluate_run(root, 'line', 11) == {'audited': 23}
    assert len(calls) == 24 and audits == [23, 23]


@pytest.mark.parametrize('changed', ['actor', 'source', 'fixture', 'incomplete', 'partial_actor'])
def test_evaluation_rejects_changed_actor_or_recovery_identity(registered, monkeypatch, changed):
    root, _ = registered
    study.register_study(root, ('line',))
    run = _run(root)
    directory = root / 'line' / 'seed_11'
    if changed == 'actor':
        (directory / 'actor.pt').write_bytes(b'changed weights')
    elif changed == 'source':
        monkeypatch.setattr(study, 'source_hash', lambda: 'changed source')
    elif changed == 'fixture':
        (root / 'cases' / 'line.json').write_text('{}')
    elif changed == 'incomplete':
        run['status'] = 'training'
        save_json(directory / 'run.json', run)
    else:
        monkeypatch.setattr(checkpoints, 'load_actor', lambda path: (object(), .05))
        save_json(root / '.evaluation' / 'line_11.json', {
            'actor_sha256': 'some other model', 'source_hash': 'frozen-source', 'cases': []})
    with pytest.raises(ValueError):
        study.evaluate_run(root, 'line', 11)


def _two_run_recipe(monkeypatch):
    original = study._recipe

    def recipe(formations):
        result = original(formations)
        result['entries'] = result['entries'][:2]
        result['training_runs'] = 2
        return result

    monkeypatch.setattr(study, '_recipe', recipe)


def test_runner_retains_failure_then_resumes_only_incomplete_training(registered, monkeypatch):
    root, _ = registered
    _two_run_recipe(monkeypatch)
    study.register_study(root, ('line',))
    _run(root, status='training')
    calls, fail_once = [], {11}

    def process(command, **kwargs):
        assert kwargs['check'] is True
        assert kwargs['env']['OMP_NUM_THREADS'] == '1'
        calls.append(command)
        if 'train' in command:
            seed = int(command[command.index('--seed') + 1])
            if seed in fail_once:
                fail_once.remove(seed)
                kwargs['stdout'].write('interrupted training\n')
                raise subprocess.CalledProcessError(1, command)
            _run(root, seed=seed)
        else:
            assert command[1:4] == ['-m', 'afor_local.study', 'evaluate']
            frozen = json.loads((root / 'study.json').read_text())
            assert frozen['test_started_at']
            assert frozen['recommendations_sha256'] == file_hash(root / 'recommended_models.json')
            assert all(entry.get('actor_sha256') for entry in frozen['entries'])
            assert all(json.loads((root / 'line' / f'seed_{seed}' / 'run.json').read_text())['status']
                       == 'completed' for seed in (11, 23))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(study.subprocess, 'run', process)
    monkeypatch.setattr(reporting, 'summarize', lambda *args: {'completed_runs': 2, 'test_episodes': 1472})
    with pytest.raises(RuntimeError, match='Some runs failed'):
        study.run_study(root, workers=1, formations=('line',))
    failed = json.loads((root / 'study.json').read_text())
    assert failed['status'] == 'failed'
    assert [e['status'] for e in failed['entries']] == ['failed', 'trained']
    assert failed['failed_phase'] == 'train'
    assert not any(command[2] == 'afor_local.study' for command in calls)
    assert not (root / 'recommended_models.json').exists()
    assert 'interrupted training' in failed['entries'][0]['log_tail']
    assert (root / '.logs' / 'line_11.log').is_file()
    assert '--resume' in calls[0]
    assert study.run_study(root, workers=1, formations=('line',))['completed_runs'] == 2
    commands = [command for command in calls if 'train' in command]
    assert [int(c[c.index('--seed') + 1]) for c in commands] == [11, 23, 11]
    assert '--resume' not in commands[1] and '--resume' in commands[2]
    done = json.loads((root / 'study.json').read_text())
    assert done['status'] == 'completed' and done['test_episodes'] == 1472
    assert all('failed_phase' not in entry for entry in done['entries'])
    assert not (root / '.logs').exists()
    before = len(commands)
    study.run_study(root, workers=1, formations=('line',))
    assert len([command for command in calls if 'train' in command]) == before


@pytest.mark.parametrize('changed', ['ppo', 'sensors'])
def test_runner_rejects_existing_training_recipe_change_before_launch(registered, monkeypatch, changed):
    root, _ = registered
    _two_run_recipe(monkeypatch)
    study.register_study(root, ('line',))
    for seed in (11, 23):
        record = _run(root, seed=seed)
        if changed == 'ppo':
            record['config']['ppo']['lr'] = 1e-5
        else:
            record['config']['sensors']['neighbour_range'] = 9.
        save_json(root / 'line' / f'seed_{seed}' / 'run.json', record)

    def forbidden(*args, **kwargs):
        raise AssertionError('A different frozen training recipe must not launch')

    monkeypatch.setattr(study.subprocess, 'run', forbidden)
    with pytest.raises(RuntimeError, match='Some runs failed'):
        study.run_study(root, workers=1, formations=('line',))
    result = json.loads((root / 'study.json').read_text())
    assert all('differs from the frozen recipe' in entry['error'] for entry in result['entries'])


def test_test_failure_reentry_preserves_recommendation_and_never_retrains(registered, monkeypatch):
    root, _ = registered
    _two_run_recipe(monkeypatch)
    study.register_study(root, ('line',))
    for seed in (11, 23):
        _run(root, seed=seed)
    calls, once = [], [True]

    def process(command, **kwargs):
        assert 'train' not in command
        frozen = json.loads((root / 'study.json').read_text())
        assert frozen['recommendations_sha256'] == file_hash(root / 'recommended_models.json')
        calls.append(command)
        if once[0]:
            once[0] = False
            raise subprocess.CalledProcessError(1, command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(study.subprocess, 'run', process)
    monkeypatch.setattr(reporting, 'summarize', lambda *args: {'completed_runs': 2, 'test_episodes': 1472})
    with pytest.raises(RuntimeError, match='Some runs failed'):
        study.run_study(root, workers=1, formations=('line',))
    failed = json.loads((root / 'study.json').read_text())
    assert failed['failed_phase'] == 'test'
    before = (root / 'recommended_models.json').read_bytes()
    study.run_study(root, workers=1, formations=('line',))
    done = json.loads((root / 'study.json').read_text())
    assert done['status'] == 'completed'
    assert done['test_started_at'] == failed['test_started_at']
    assert done['recommendations_sha256'] == failed['recommendations_sha256']
    assert (root / 'recommended_models.json').read_bytes() == before
    assert len(calls) == 4


@pytest.mark.parametrize('workers', [0, -1, True, 1.5])
def test_runner_rejects_invalid_worker_count(registered, workers):
    root, _ = registered
    with pytest.raises(ValueError, match='positive integer'):
        study.run_study(root, workers=workers)
    assert not root.exists()
