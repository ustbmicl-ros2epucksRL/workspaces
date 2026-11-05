import json

import pytest
import torch
from torch import nn

from afor_local.environment import LocalFormationEnv, TaskConfig
from afor_local.observation import SensorConfig
from afor_local.config import ScenarioConfig
from afor_local.evaluate import build_case_set, evaluate_case


class ZeroActor(nn.Module):
    sensors = SensorConfig()

    def __init__(self, crash_first=False):
        super().__init__()
        self.anchor = nn.Parameter(torch.zeros(()))
        self.crash_first = crash_first

    def initial_state(self, batch_shape=()):
        return self.anchor.new_zeros(tuple(batch_shape) + (64,))

    def forward(self, obs, state, reset=None):
        action = obs['ego'].new_zeros(obs['ego'].shape[:-1] + (2,))
        if self.crash_first:
            action[0, 0, 0] = -1.
        return action, state


def test_disabled_randomization_preserves_endpoint_rng_and_physics():
    config = TaskConfig(num_envs=3, horizon=10)
    old = LocalFormationEnv(config, seed=7, scenario=ScenarioConfig(0., 0.))
    new = LocalFormationEnv(config, seed=7,
                              scenario=ScenarioConfig(perturb_probability=1., max_jitter=0.))
    for _ in range(3):
        assert torch.equal(old.pos, new.pos)
        assert torch.equal(old.goal, new.goal)
        assert set(new.observe()) == {'ego', 'scan', 'neighbours', 'mask'}
        for key, tensor in old.observe().items():
            assert torch.equal(tensor, new.observe()[key])
        action = torch.zeros_like(old.vel)
        old_result, new_result = old.step(action), new.step(action)
        for left, right in zip(old_result[:3], new_result[:3]):
            assert torch.equal(left, right)
        for key in old_result[3]:
            assert torch.equal(old_result[3][key], new_result[3][key])
    old.reset([0, 2])
    new.reset([0, 2])
    assert torch.equal(old.pos, new.pos)
    assert torch.equal(old.rng.get_state(), new.rng.get_state())


def test_paired_manifest_does_not_depend_on_actor_or_outcomes():
    arguments = dict(dt=.05, formation='line', jitter=.1, episodes=3, horizon=5)
    fixture = build_case_set(**arguments)
    assert json.loads(json.dumps(fixture, allow_nan=False)) == fixture
    first, no_trace = evaluate_case(ZeroActor(), **arguments, case_set=fixture)
    second, _ = evaluate_case(ZeroActor(crash_first=True), **arguments)
    assert first['case_hash'] == second['case_hash'] == fixture['case_hash']
    assert no_trace is None
    assert len(first['results']) == 3
    json.dumps(first, allow_nan=False)


def test_perturbed_initial_teams_are_valid_and_rejections_are_bounded():
    config = TaskConfig(num_envs=16, formation='circle')
    env = LocalFormationEnv(config, seed=19,
                              scenario=ScenarioConfig(1., 1.5, 100))
    assert (env.clearances(env.pos, env.pos) > 0).all()
    assert (env.initial_jitter.abs() <= env.jitter_amplitude[:, None] + 1e-6).all()
    assert env.perturbation_counters['rejected_proposals'] > 0
    with pytest.raises(RuntimeError, match='collision-free'):
        LocalFormationEnv(TaskConfig(num_envs=2), seed=9,
                            scenario=ScenarioConfig(1., 1e8, 2))
    with pytest.raises(RuntimeError, match='collision-free'):
        build_case_set(.05, 'line', jitter=1e8, episodes=2, max_rejections=2)


def test_exact_resume_includes_both_rng_streams_and_counters():
    config = TaskConfig(num_envs=4, formation='wedge')
    scenario = ScenarioConfig(.5, .15, 100)
    original = LocalFormationEnv(config, seed=23, scenario=scenario)
    original.step(torch.zeros_like(original.vel))
    original.reset([0, 2])
    saved = original.state_dict()
    restored = LocalFormationEnv(config, seed=999, scenario=scenario)
    restored.load_state_dict(saved)
    for ids in ([1], [0, 2, 3], [0, 1, 2, 3]):
        original.reset(ids)
        restored.reset(ids)
        for name in original.state_fields:
            assert torch.equal(getattr(original, name), getattr(restored, name))
        for name in ('jitter_amplitude', 'initial_jitter', 'reset_attempts'):
            assert torch.equal(getattr(original, name), getattr(restored, name))
        assert original.perturbation_counters == restored.perturbation_counters
        assert torch.equal(original.perturb_rng.get_state(), restored.perturb_rng.get_state())
    wrong = LocalFormationEnv(config, scenario=ScenarioConfig(1., .15, 100))
    with pytest.raises(ValueError, match='perturbation'):
        wrong.load_state_dict(saved)


def test_only_first_episode_counts_when_another_team_runs_longer():
    report, trace = evaluate_case(ZeroActor(crash_first=True), .05, 'line',
                                  episodes=2, horizon=120, trace=True)
    assert len(report['results']) == report['episodes'] == 2
    assert [r['episode'] for r in report['results']] == [0, 1]
    assert report['results'][0]['length'] < 120
    assert report['results'][0]['failure_type'] in ('pair', 'wall', 'boundary')
    assert report['results'][1]['length'] == 120
    assert report['results'][1]['timeout']
    assert report['results'][1]['timeout_reasons']
    assert trace['selected_episode'] == 0
    assert trace['position'].shape[0] == report['results'][0]['length'] + 1
    assert trace['position'].shape[1] == 1
    assert report['collisions'] == 1


def test_fixture_modification_is_detected():
    fixture = build_case_set(.05, 'line', episodes=1, horizon=2)
    fixture['cases'][0]['position'][0][0] += .01
    with pytest.raises(ValueError, match='modified'):
        evaluate_case(ZeroActor(), .05, 'line', episodes=1, horizon=2, case_set=fixture)


def test_goal_dwell_without_actual_opening_crossing_is_a_bypass():
    # An adversarial but physically legal fixture must not fake traversal.
    from afor_local.evaluate import _case_hash
    fixture = build_case_set(.05, 'line', episodes=2, horizon=12)
    for case in fixture['cases']:
        case['position'] = case['goal']
    fixture['case_hash'] = _case_hash(fixture)
    report, _ = evaluate_case(ZeroActor(), .05, 'line', episodes=2,
                              horizon=12, case_set=fixture)
    assert report['successes'] == report['corridor_passes'] == 0
    assert report['goal_successes'] == report['bypass_completions'] == 2
    assert all(row['oracle_goal_success'] and row['failure_type'] == 'bypass'
               for row in report['results'])


def test_high_level_evaluation_loads_only_canonical_exports(tmp_path):
    from afor_local.checkpoints import export_actor
    from afor_local.config import PolicyConfig
    from afor_local.evaluate import evaluate
    from afor_local.policy import ActorCritic
    model = ActorCritic(PolicyConfig(), seed=7)
    path = tmp_path/'actor.pt'
    export_actor(model, path, .05)
    loaded, _ = evaluate(path, formation='line', episodes=2, horizon=3)
    eager, _ = evaluate(model.actor, .05, formation='line', episodes=2, horizon=3)
    assert loaded == eager
    with pytest.raises(ValueError, match='timestep'):
        evaluate(path, dt=.1, episodes=1, horizon=1)
