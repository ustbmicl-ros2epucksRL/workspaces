"""Paired validation selection is isolated, immutable and exactly resumable."""
import copy
import importlib
import random

import numpy as np

import pytest
import torch

from afor_local.checkpoints import actor_payload, export_actor, load_actor
from afor_local.artifacts import tensor_state_hash
from afor_local.config import PolicyConfig, SelectionConfig
from afor_local.environment import TaskConfig
from afor_local.policy import ActorCritic
from afor_local.selection import BestTracker, build_validation_cases, selection_key, validate_current_actor


selection = importlib.import_module('afor_local.selection')


def tiny_tracker():
    return BestTracker(.05, TaskConfig(horizon=2), SelectionConfig(True, interval=2, episodes=2))


def score(tracker, nominal=4, challenge=10, duration=.09):
    return {'nominal_successes': nominal, 'nominal_episodes': 2*tracker.config.episodes,
            'challenge_successes': challenge, 'challenge_episodes': 6*tracker.config.episodes,
            'mean_failure_penalized_duration': duration,
            'validation_seeds': [172001, 172002],
            'case_hashes': selection._case_identities(tracker.case_sets)}


def assert_nested_equal(first, second):
    if isinstance(first, torch.Tensor):
        assert isinstance(second, torch.Tensor) and torch.equal(first, second)
    elif isinstance(first, dict):
        assert first.keys() == second.keys()
        for key in first:
            assert_nested_equal(first[key], second[key])
    elif isinstance(first, (tuple, list)):
        assert type(first) is type(second) and len(first) == len(second)
        for a, b in zip(first, second):
            assert_nested_equal(a, b)
    else:
        assert first == second


@pytest.mark.parametrize('settings', [{'enabled': 1}, {'interval': 0}, {'episodes': 0},
                                     {'interval': 2.5}, {'episodes': True}])
def test_selection_configuration_rejects_invalid_or_ambiguous_settings(settings):
    with pytest.raises(ValueError):
        SelectionConfig(**settings)


def test_disabled_selection_builds_no_fixtures_or_best_actor(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Disabled selection must not evaluate or construct fixtures')
    monkeypatch.setattr(selection, 'build_case_set', forbidden)
    before = torch.get_rng_state().clone()
    tracker = BestTracker(.05, TaskConfig())
    assert not tracker.due(25, 500)
    assert tracker.case_sets == () and tracker.best_actor_payload is None
    assert tracker.summary()['selected_update'] is None
    assert torch.equal(before, torch.get_rng_state())


def test_schedule_includes_interval_and_final_update():
    tracker = tiny_tracker()
    assert not tracker.due(0, 5)
    assert not tracker.due(1, 5)
    assert tracker.due(2, 5) and tracker.due(4, 5) and tracker.due(5, 5)


def test_lexicographic_order_prioritizes_nominal_then_challenge_then_duration():
    tracker = tiny_tracker()
    assert selection_key(score(tracker, 4, 0, .1)) > selection_key(score(tracker, 3, 12, .01))
    assert selection_key(score(tracker, 4, 11, .1)) > selection_key(score(tracker, 4, 10, .01))
    assert selection_key(score(tracker, 4, 11, .08)) > selection_key(score(tracker, 4, 11, .09))


@pytest.mark.parametrize('field,value', [('nominal_successes', -1), ('challenge_successes', 13),
                                        ('nominal_episodes', 0), ('challenge_successes', 1.5),
                                        ('mean_failure_penalized_duration', float('nan')),
                                        ('mean_failure_penalized_duration', -1.)])
def test_malformed_scores_cannot_select_an_actor(field, value):
    tracker = tiny_tracker()
    invalid = score(tracker)
    invalid[field] = value
    with pytest.raises(ValueError):
        selection_key(invalid)


def test_best_tracker_keeps_earlier_ties_and_one_immutable_actor(monkeypatch):
    tracker = tiny_tracker()
    actor = ActorCritic(PolicyConfig(), seed=11).actor
    observations = [score(tracker, 3, 12, .02), score(tracker, 4, 0, .1),
                    score(tracker, 4, 1, .1), score(tracker, 4, 1, .08), score(tracker, 4, 1, .08)]
    iterator = iter(observations)
    monkeypatch.setattr(selection, 'validate_current_actor', lambda *args, **kwargs: next(iterator))
    records = [tracker.observe(actor, update) for update in (2, 4, 6, 8, 10)]
    assert [record['improved'] for record in records] == [True, True, True, True, False]
    assert tracker.best_update == 8 and tracker.last_update == 10 and tracker.check_count == 5
    assert records[-1]['selected_update'] == 8
    assert records[-1]['selected_actor_state_sha256'] == tracker.best_actor_state_sha256
    assert tracker.best_actor_state_sha256 == tensor_state_hash(tracker.best_actor_payload['state_dict'])
    frozen_weights = {name: value.clone() for name, value in tracker.best_actor_payload['state_dict'].items()}
    with torch.no_grad():
        for parameter in actor.parameters():
            parameter.add_(1.)
    assert all(torch.equal(value, tracker.best_actor_payload['state_dict'][name])
               for name, value in frozen_weights.items())
    assert all(value.device.type == 'cpu' for value in tracker.best_actor_payload['state_dict'].values())
    public = tracker.summary()
    assert len(public['history']) == 5 and public['interval'] == 2 and public['episodes'] == 2
    assert 'best_actor_payload' not in public
    public['history'].clear()
    assert len(tracker.history) == 5
    with pytest.raises(ValueError, match='strictly increasing'):
        tracker.observe(actor, 10)


def test_tracker_resume_restores_best_payload_scores_schedule_and_history(monkeypatch):
    original = tiny_tracker()
    actor = ActorCritic(seed=12).actor
    monkeypatch.setattr(selection, 'validate_current_actor', lambda *args, **kwargs: score(original))
    original.observe(actor, 2)
    snapshot = original.state_dict()
    restored = tiny_tracker()
    restored.load_state_dict(snapshot)
    assert_nested_equal(original.state_dict(), restored.state_dict())
    assert original.observe(actor, 4) == restored.observe(actor, 4)
    assert_nested_equal(original.state_dict(), restored.state_dict())
    # A loaded tracker does not keep aliases into its recovery snapshot.
    next(iter(snapshot['best_actor_payload']['state_dict'].values())).zero_()
    assert not torch.equal(next(iter(snapshot['best_actor_payload']['state_dict'].values())),
                           next(iter(restored.best_actor_payload['state_dict'].values())))


@pytest.mark.parametrize('change', ['config', 'cases', 'schedule', 'history'])
def test_tracker_resume_rejects_changed_protocol_or_malformed_state(change):
    tracker = tiny_tracker()
    state = tracker.state_dict()
    if change == 'config':
        state['config']['episodes'] = 24
    elif change == 'cases':
        state['case_hashes'][0]['case_hash'] = 'different-fixture'
    elif change == 'schedule':
        state['check_count'] = -1
    else:
        state['history'] = [{'update': 2}]
    with pytest.raises(ValueError):
        tracker.load_state_dict(state)


def test_real_validation_preserves_cpu_cuda_rng_and_every_module_mode_with_shared_cases():
    task, config = TaskConfig(horizon=2), SelectionConfig(True, interval=2, episodes=2)
    actor = ActorCritic(PolicyConfig(), seed=19).actor
    actor.train()
    actor.graph.peer.eval()  # Preserve intentionally mixed submodule modes too.
    modes = [module.training for module in actor.modules()]
    before = torch.get_rng_state().clone()
    cuda_before = torch.cuda.get_rng_state_all() if torch.cuda.is_initialized() else None
    cases = build_validation_cases(.05, task, config)
    assert torch.equal(before, torch.get_rng_state())
    first = validate_current_actor(actor, .05, task, config, case_sets=cases)
    assert torch.equal(before, torch.get_rng_state())
    assert [module.training for module in actor.modules()] == modes
    if cuda_before is not None:
        assert all(torch.equal(a, b) for a, b in zip(cuda_before, torch.cuda.get_rng_state_all()))
    assert first['validation_seeds'] == [172001, 172002]
    assert first['nominal_episodes'] == 4 and first['challenge_episodes'] == 12
    assert len(first['case_hashes']) == 8
    assert {case['jitter'] for case in first['case_hashes']} == {0., .05, .10, .15}
    other = ActorCritic(PolicyConfig(), seed=20).actor
    second = validate_current_actor(other, .05, task, config, case_sets=cases)
    assert first['case_hashes'] == second['case_hashes']


def test_validation_restores_rng_and_modes_after_evaluator_failure(monkeypatch):
    tracker = tiny_tracker()
    actor = ActorCritic(seed=12).actor.train()
    before = torch.get_rng_state().clone()
    python_before, numpy_before = random.getstate(), np.random.get_state()
    def failed(*args, **kwargs):
        torch.rand(11)
        random.random()
        np.random.random(5)
        raise RuntimeError('failed independent evaluation')
    monkeypatch.setattr(selection, 'evaluate_case', failed)
    with pytest.raises(RuntimeError, match='failed independent evaluation'):
        tracker.observe(actor, 2)
    assert torch.equal(before, torch.get_rng_state())
    assert actor.training
    assert random.getstate() == python_before
    assert np.array_equal(np.random.get_state()[1], numpy_before[1])
    assert tracker.check_count == tracker.last_update == 0
    assert tracker.best_actor_payload is None


def test_actor_payload_owns_cpu_weights_and_preserves_export_wire_format(tmp_path):
    actor = ActorCritic(PolicyConfig(), seed=13).actor.train()
    before = torch.get_rng_state().clone()
    payload = actor_payload(actor, .05)
    assert actor.training and torch.equal(before, torch.get_rng_state())
    path = tmp_path / 'actor.pt'
    export_actor(actor, path, .05)
    assert_nested_equal(payload, torch.load(path, weights_only=True))
    restored, _ = load_actor(path)
    assert restored.config == actor.config
    frozen = copy.deepcopy(payload['state_dict'])
    with torch.no_grad():
        actor.ego[0].weight.add_(2.)
    assert_nested_equal(frozen, payload['state_dict'])
