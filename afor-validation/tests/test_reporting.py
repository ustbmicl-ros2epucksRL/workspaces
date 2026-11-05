"""Audit real first-episode records before computing training-seed statistics."""
import copy
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from afor_local.artifacts import file_hash, save_json, source_hash
from afor_local.checkpoints import SCHEMA, export_actor
from afor_local.config import PPOConfig, PolicyConfig, ScenarioConfig
from afor_local.environment import TaskConfig
from afor_local.evaluate import build_case_set, evaluate_case
from afor_local.observation import SensorConfig
from afor_local.policy import ActorCritic
from afor_local.protocols import FORMATIONS, TEST_SEEDS, UPDATES, case_specs
from afor_local.reporting import audit_run, outcome_summary, summarize
import afor_local.reporting as reporting


torch.set_num_threads(1)


@pytest.fixture(scope='module')
def real_evaluations():
    """Tiny genuine evaluations, not fabricated success flags or counts."""
    model = ActorCritic(seed=71).eval()
    results = {}
    for formation in FORMATIONS:
        cases, fixtures = [], []
        for spec in case_specs():
            arguments = {key: spec[key] for key in ('passage_width', 'jitter', 'seed', 'episodes')}
            fixture = build_case_set(.05, formation, horizon=2, **arguments)
            case, _ = evaluate_case(model.actor, .05, formation, horizon=2,
                                    case_set=fixture, **arguments)
            cases.append(case)
            fixtures.append({'spec': spec, 'fixture': fixture})
        results[formation] = {'cases': cases, 'fixtures': fixtures}
    return model.actor, results


def write_run(root, real_evaluations, name='run', *, formation='line', seed=11,
              updates=2, source=None):
    actor, templates = real_evaluations
    directory = root / name
    directory.mkdir(parents=True)
    export_actor(actor, directory / 'actor.pt', .05)
    task = TaskConfig(formation=formation, horizon=2)
    config = {'task': asdict(task), 'policy': asdict(PolicyConfig()),
              'sensors': asdict(SensorConfig()), 'ppo': asdict(PPOConfig()),
              'scenario': asdict(ScenarioConfig()), 'seed': seed}
    identity = source if source is not None else source_hash()
    run = {'schema': SCHEMA, 'implementation': 'afor_local_g', 'status': 'completed',
           'config': config, 'completed_updates': updates, 'source_hash': identity,
           'actor_sha256': file_hash(directory / 'actor.pt')}
    evaluation = {'split': 'test', 'actor_sha256': run['actor_sha256'],
                  'source_hash': identity, 'cases': copy.deepcopy(templates[formation]['cases'])}
    save_json(directory / 'run.json', run)
    save_json(directory / 'evaluation.json', evaluation)
    (directory / 'metrics.jsonl').write_text(
        json.dumps({'update': 1, 'success': 0., 'completed_episodes': 16}) + '\n')
    return directory, run, evaluation


@pytest.mark.parametrize('formation', FORMATIONS)
def test_complete_twenty_three_conditions_are_audited_from_real_raw_episodes(tmp_path, real_evaluations, formation):
    directory, _, _ = write_run(tmp_path, real_evaluations, formation=formation)
    fixtures = real_evaluations[1][formation]['fixtures']
    row = audit_run(directory, fixtures)
    assert row['formation'] == formation and row['seed'] == 11 and row['updates'] == 2
    assert row['test_episodes'] == 736
    assert row['families']['main_challenge']['episodes'] == 288
    assert row['families']['nominal_regression']['episodes'] == 96
    assert row['families']['stronger_perturbation']['episodes'] == 96
    assert row['families']['width_transfer']['episodes'] == 160
    assert row['families']['joint_shift']['episodes'] == 96
    assert sum(family['successes'] for family in row['families'].values()) == 0
    assert all(group['success_rate'] == 0 for group in row['by_jitter'].values())
    summary = summarize(tmp_path)
    assert summary['status'] == 'partial' and summary['completed_runs'] == 1
    assert summary['groups'][0]['families']['main_challenge']['success_rate'] == {'mean': 0., 'std': None}


@pytest.mark.parametrize('change', ['successes', 'collisions', 'timeouts', 'failure_counts', 'missing_case', 'duplicate_case',
                                    'missing_episode', 'duplicate_episode', 'wrong_seed',
                                    'wrong_split', 'wrong_formation', 'wrong_dt', 'wrong_horizon',
                                    'no_oracle', 'wrong_duration', 'missing_failure_penalty',
                                    'invalid_process_metric', 'false_success'])
def test_malformed_or_incomplete_evidence_is_rejected(tmp_path, real_evaluations, change):
    directory, _, evaluation = write_run(tmp_path, real_evaluations)
    case = evaluation['cases'][0]
    row = case['results'][0]
    if change == 'successes':
        case['successes'] += 1
    elif change == 'collisions':
        case['collisions'] += 1
    elif change == 'timeouts':
        case['timeouts'] -= 1
    elif change == 'failure_counts':
        case['failure_counts']['timeout'] -= 1
    elif change == 'missing_case':
        evaluation['cases'].pop()
    elif change == 'duplicate_case':
        evaluation['cases'].append(copy.deepcopy(case))
    elif change == 'missing_episode':
        case['results'].pop()
    elif change == 'duplicate_episode':
        case['results'][1]['episode'] = 0
    elif change == 'wrong_seed':
        case['seed'] = 72001
    elif change == 'wrong_split':
        evaluation['split'] = 'validation'
    elif change in ('wrong_formation', 'wrong_dt', 'wrong_horizon'):
        field = change.removeprefix('wrong_')
        case[field] = {'formation': 'circle', 'dt': .1, 'horizon': 3}[field]
    elif change == 'no_oracle':
        case['independent_swept_oracle'] = False
    elif change == 'wrong_duration':
        row['duration'] += .1
    elif change == 'missing_failure_penalty':
        row['failure_penalized_duration'] = 0.
    elif change == 'invalid_process_metric':
        row['mean_control_effort'] = float('nan')
    else:
        row['success'] = True
        row['failure_type'] = None
        case['successes'] = 1
    # Explicit JSON encoding permits the intentional NaN corruption fixture.
    (directory / 'evaluation.json').write_text(json.dumps(evaluation))
    with pytest.raises(ValueError):
        audit_run(directory)


@pytest.mark.parametrize('change', ['actor_bytes', 'evaluation_actor_hash', 'evaluation_source'])
def test_retained_actor_and_evaluator_identity_cannot_be_relabelled(tmp_path, real_evaluations, change):
    directory, _, evaluation = write_run(tmp_path, real_evaluations)
    if change == 'actor_bytes':
        (directory / 'actor.pt').write_bytes(b'changed actor bytes')
    elif change == 'evaluation_actor_hash':
        evaluation['actor_sha256'] = 'another-checkpoint'
    else:
        evaluation['source_hash'] = 'another-evaluator'
    save_json(directory / 'evaluation.json', evaluation)
    with pytest.raises(ValueError, match='hash|identity'):
        audit_run(directory)


@pytest.mark.parametrize('change', ['missing_fixture', 'case_hash'])
def test_registered_fixture_identity_is_required(tmp_path, real_evaluations, change):
    directory, _, _ = write_run(tmp_path, real_evaluations)
    fixtures = copy.deepcopy(real_evaluations[1]['line']['fixtures'])
    if change == 'missing_fixture':
        fixtures.pop()
    else:
        fixtures[0]['fixture']['case_hash'] = 'not-the-registered-scene'
    with pytest.raises(ValueError, match='fixtures|case hash'):
        audit_run(directory, fixtures)


def test_duplicate_independent_training_seed_is_rejected_even_for_different_budget(tmp_path, real_evaluations):
    write_run(tmp_path, real_evaluations, 'first', seed=11, updates=2)
    write_run(tmp_path, real_evaluations, 'duplicate', seed=11, updates=3)
    with pytest.raises(ValueError, match='Duplicate independent training seed'):
        summarize(tmp_path)


@pytest.mark.parametrize('change', ['source', 'recipe', 'budget'])
def test_incomparable_recipes_cannot_be_pooled(tmp_path, real_evaluations, change):
    write_run(tmp_path, real_evaluations, 'first', seed=11)
    directory, run, evaluation = write_run(tmp_path, real_evaluations, 'second', seed=23)
    if change == 'source':
        run['source_hash'] = evaluation['source_hash'] = 'different-frozen-source'
    elif change == 'recipe':
        run['config']['ppo']['lr'] /= 2
    else:
        run['completed_updates'] += 1
    save_json(directory / 'run.json', run)
    save_json(directory / 'evaluation.json', evaluation)
    with pytest.raises(ValueError, match='Mixed training recipes'):
        summarize(tmp_path)


def test_independent_seed_mean_sample_std_and_bootstrap_preserve_numpy_rng(tmp_path, real_evaluations, monkeypatch):
    for seed in (11, 23, 37):
        write_run(tmp_path, real_evaluations, f'seed_{seed}', seed=seed)
    original = reporting.audit_run
    scores = {11: .25, 23: .5, 37: .75}

    def audited_score(directory, fixtures=None):
        # Statistics are tested at the audited-summary boundary: no fabricated
        # successful physical trajectory is passed through the oracle audit.
        row = original(directory, fixtures)
        for family in row['families'].values():
            family['success_rate'] = scores[row['seed']]
            family['successes'] = round(family['episodes'] * scores[row['seed']])
        return row

    monkeypatch.setattr(reporting, 'audit_run', audited_score)
    before = np.random.get_state()
    summary = summarize(tmp_path)
    after = np.random.get_state()
    assert before[0] == after[0] and before[2:] == after[2:]
    np.testing.assert_array_equal(before[1], after[1])
    assert summarize(tmp_path) == summary
    group = summary['groups'][0]
    assert group['training_seeds'] == 3 and group['seeds'] == [11, 23, 37]
    score = group['families']['main_challenge']['success_rate']
    assert score['mean'] == .5 and score['std'] == pytest.approx(.25)
    assert score['ci95'] == [.25, .75]
    assert summary['statistics']['unit'] == 'independent training seed'


def test_success_and_failure_metrics_remain_separate_and_empty_success_is_explicit(real_evaluations):
    failed = real_evaluations[1]['line']['cases'][0]['results'][0]
    summary = outcome_summary([failed])
    assert summary['episodes'] == 1 and summary['successes'] == 0
    assert summary['success']['episodes'] == 0
    assert summary['success']['duration'] is None
    assert summary['all']['failure_penalized_duration'] == .1
    assert summary['failure']['mean_path_length'] == failed['mean_path_length']
    empty = outcome_summary([])
    assert empty['success_rate'] is None and empty['all']['duration'] is None


def test_summary_json_png_and_pdf_preserve_cli_api(tmp_path, real_evaluations):
    write_run(tmp_path / 'runs', real_evaluations)
    output, figure = tmp_path / 'report.json', tmp_path / 'figure.png'
    summary = summarize(tmp_path / 'runs', output, figure)
    assert json.loads(output.read_text()) == summary
    assert figure.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
    assert figure.with_suffix('.pdf').read_bytes().startswith(b'%PDF')


def test_study_registration_binds_source_and_fixture_file(tmp_path, real_evaluations):
    directory, run, _ = write_run(tmp_path, real_evaluations, 'line/seed_11', updates=UPDATES)
    fixture_path = tmp_path / 'cases' / 'line.json'
    save_json(fixture_path, {'cases': real_evaluations[1]['line']['fixtures']})
    study = {'entries': [{'formation': 'line', 'seed': 11}], 'source_hash': run['source_hash'],
             'case_files': {'line': {'sha256': file_hash(fixture_path)}}}
    save_json(tmp_path / 'study.json', study)
    assert summarize(tmp_path)['status'] == 'complete'
    study['source_hash'] = 'different-registration-source'
    save_json(tmp_path / 'study.json', study)
    with pytest.raises(ValueError, match='registered source'):
        summarize(tmp_path)
    study['source_hash'] = run['source_hash']
    save_json(tmp_path / 'study.json', study)
    fixture_path.write_text(fixture_path.read_text() + ' ')
    with pytest.raises(ValueError, match='case file has changed'):
        summarize(tmp_path)


def test_missing_evaluation_keeps_study_partial_without_claiming_a_score(tmp_path, real_evaluations):
    directory, _, _ = write_run(tmp_path, real_evaluations)
    (directory / 'evaluation.json').unlink()
    summary = summarize(tmp_path)
    assert summary['status'] == 'partial' and summary['completed_runs'] == 0
    assert summary['groups'] == []


def test_legacy_schema_cannot_be_relabelled_as_a_new_graph_run(tmp_path, real_evaluations):
    directory, run, _ = write_run(tmp_path, real_evaluations)
    run['schema'] = 'afor_local_v2'
    save_json(directory / 'run.json', run)
    with pytest.raises(ValueError, match='completed G training run'):
        audit_run(directory)


@pytest.mark.parametrize('formation,field,boundary', [
    ('line', 'final_y_span_m', .15),
    ('line', 'edge_length_error', .1),
    ('circle', 'centered_template_error', .1),
    ('wedge', 'centered_template_error', .1),
])
def test_success_claim_must_satisfy_strict_independent_shape_thresholds(
        tmp_path, real_evaluations, formation, field, boundary):
    directory, run, evaluation = write_run(tmp_path, real_evaluations, formation=formation)
    # Construct one internally consistent success claim for the acceptance-unit
    # boundary, rather than pretending an untrained two-step rollout succeeded.
    run['config']['task']['horizon'] = 750
    for case in evaluation['cases']:
        case['horizon'] = 750
        for row in case['results']:
            row['failure_penalized_duration'] = 37.5
    case = evaluation['cases'][0]
    row = case['results'][0]
    row.update(success=True, collision=False, corridor_passed=True, oracle_goal_success=True,
               final_formation_ok=True, max_goal_distance=.1, max_speed=.1,
               hold=10, failure_type=None, edge_length_error=.01,
               centered_template_error=.01, final_y_span_m=.01,
               min_clearance=[.01, .01, .01], length=10, duration=.5,
               failure_penalized_duration=.5, timeout=False)
    case['successes'] = 1
    case['timeouts'] -= 1
    case['failure_counts']['timeout'] -= 1
    save_json(directory / 'run.json', run)
    save_json(directory / 'evaluation.json', evaluation)
    assert audit_run(directory)['families']['nominal_regression']['successes'] == 1
    row[field] = boundary
    save_json(directory / 'evaluation.json', evaluation)
    with pytest.raises(ValueError, match='Success violates independent acceptance'):
        audit_run(directory)
