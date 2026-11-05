"""Process metrics must describe committed first-episode physical trajectories."""
import copy

import numpy as np
import pytest
import torch
from torch import nn

from afor_local.evaluate import _case_hash, build_case_set, evaluate_case
from afor_local.formations import make_formation
from afor_local.observation import SensorConfig


torch.set_num_threads(1)


class ConstantActor(nn.Module):
    sensors = SensorConfig()

    def __init__(self, commands):
        super().__init__()
        self.device_parameter = nn.Parameter(torch.zeros(()))
        self.register_buffer('commands', torch.as_tensor(commands, dtype=torch.float32))

    def initial_state(self, shape):
        return self.device_parameter.new_zeros(tuple(shape) + (64,))

    def forward(self, observation, state):
        return self.commands.expand(*observation['ego'].shape[:-1], 2), state


class FirstTeamGoalActor(ConstantActor):
    """Simple test controller: move the first team; leave remaining teams still."""
    def __init__(self):
        super().__init__(torch.zeros(5, 2))

    def forward(self, observation, state):
        action = observation['ego'].new_zeros(observation['ego'].shape[:-1] + (2,))
        action[0] = (2 * observation['ego'][0, :, :2]
                     - 2 * observation['ego'][0, :, 2:4]).clamp(-1, 1)
        return action, state


def seal(fixture):
    fixture['case_hash'] = _case_hash(fixture)
    return fixture


def trace_metrics(trace, dt):
    """Recompute metrics from saved executed positions and velocities only."""
    position = trace['position'][:, 0].double()
    velocity = trace['velocity'][:, 0].double()
    path = position.diff(dim=0).norm(dim=-1).mean(-1).sum()
    effort = (velocity.diff(dim=0) / dt).square().sum(-1).mean(-1).sum() * dt
    formation = make_formation('line', 'cpu')
    template = formation.template.double()
    adjacency = torch.zeros(5, 5, dtype=torch.float64)
    for i, j in formation.edges:
        adjacency[i, j] = adjacency[j, i] = 1
    actual_delta = position[1:, None] - position[1:, :, None]
    desired_delta = template[None] - template[:, None]
    per_edge = (actual_delta - desired_delta).norm(dim=-1) * adjacency
    per_agent = per_edge.sum(-1) / adjacency.sum(-1)
    return {'mean_path_length': float(path), 'mean_control_effort': float(effort),
            'mean_process_formation_error': float(per_agent.mean())}


def test_process_metrics_equal_executed_trace_and_use_applied_velocity_change():
    dt, horizon = .05, 30
    fixture = build_case_set(dt, 'line', episodes=1, horizon=horizon)
    fixture['cases'][0]['position'] = [[x, -3.2] for x in (-1., -.5, 0., .5, 1.)]
    commands = torch.tensor([[-.2, 2.], [-.1, 2.], [0., 2.], [.1, 2.], [.2, 2.]])
    report, trace = evaluate_case(ConstantActor(commands), dt, 'line', episodes=1,
                                  horizon=horizon, trace=True, case_set=seal(fixture))
    result = report['results'][0]
    assert result['timeout'] and not result['collision']
    assert trace['position'].shape[0] == horizon + 1
    expected = trace_metrics(trace, dt)
    assert expected['mean_path_length'] > 0
    assert expected['mean_process_formation_error'] > 0
    for metric, value in expected.items():
        assert result[metric] == pytest.approx(value, abs=1e-7)
        assert report['outcome_metrics']['all'][metric] == result[metric]
    # Forward velocity has saturated. Squared commanded accelerations would
    # overcount effort; the evaluator integrates the applied velocity changes.
    assert trace['velocity'][-1, 0, :, 1].eq(1).all()
    commanded_effort = float(commands.clamp(-1, 1).square().sum(-1).mean()) * horizon * dt
    assert result['mean_control_effort'] < commanded_effort
    assert report['outcome_metrics']['success']['episodes'] == 0
    assert all(value is None for key, value in report['outcome_metrics']['success'].items()
               if key != 'episodes')


def test_terminal_collision_proposal_does_not_add_motion_or_control_effort():
    dt = .05
    fixture = build_case_set(dt, 'line', episodes=1, horizon=4)
    # The initial pair is legal, but the first commanded movement would collide.
    fixture['cases'][0]['position'] = [[x, -3.2] for x in (-1., -.7395, -.3, .3, .9)]
    commands = torch.zeros(5, 2)
    commands[0, 0] = 1.
    report, trace = evaluate_case(ConstantActor(commands), dt, 'line', episodes=1,
                                  horizon=4, trace=True, case_set=seal(fixture))
    result = report['results'][0]
    assert result['length'] == 1 and result['collision']
    assert result['failure_type'] == 'pair'
    assert not torch.equal(trace['proposal'][0], trace['position'][0])
    assert torch.equal(trace['position'][0], trace['position'][1])
    assert torch.equal(trace['velocity'][0], trace['velocity'][1])
    assert result['mean_path_length'] == result['mean_control_effort'] == 0.
    for metric, value in trace_metrics(trace, dt).items():
        assert result[metric] == pytest.approx(value, abs=1e-7)
    assert report['outcome_metrics']['failure']['episodes'] == 1
    assert report['outcome_metrics']['failure']['mean_path_length'] == 0.


@pytest.fixture(scope='module')
def mixed_outcomes():
    dt, horizon = .05, 100
    fixture = build_case_set(dt, 'line', passage_width=1.6, episodes=2, horizon=horizon)
    # A legal compact line just before the opening lets a simple controller
    # complete the full physical crossing, final restoration, and stop criteria.
    fixture['cases'][0]['position'] = [[x, 1.2] for x in (-.6, -.3, 0., .3, .6)]
    report, trace = evaluate_case(FirstTeamGoalActor(), dt, 'line', passage_width=1.6,
                                  episodes=2, horizon=horizon, trace=True, case_set=seal(fixture))
    return report, trace, fixture


def test_successful_first_episode_is_unchanged_by_later_dummy_resets(mixed_outcomes):
    report, trace, fixture = mixed_outcomes
    success, failure = report['results']
    assert success['success'] and success['length'] < report['horizon']
    assert failure['timeout'] and failure['length'] == report['horizon']
    assert success['corridor_passed'] and success['hold'] >= 10
    # First-team tracing ends at its first completion even while the second
    # team's timeout requires later simulation and resets of dummy first teams.
    assert trace['position'].shape[0] == success['length'] + 1
    assert trace['proposal'].shape[0] == success['length']
    assert trace['active'].all()
    for metric, value in trace_metrics(trace, report['dt']).items():
        assert success[metric] == pytest.approx(value, abs=1e-7)
    solo_fixture = copy.deepcopy(fixture)
    solo_fixture['task']['num_envs'] = 1
    solo_fixture['cases'] = solo_fixture['cases'][:1]
    solo_report, solo_trace = evaluate_case(FirstTeamGoalActor(), report['dt'], 'line',
        passage_width=1.6, episodes=1, horizon=report['horizon'], trace=True, case_set=seal(solo_fixture))
    assert solo_report['results'][0] == success
    for key in ('position', 'velocity', 'proposal', 'active'):
        torch.testing.assert_close(solo_trace[key], trace[key], atol=0, rtol=0)
    assert solo_report['outcome_metrics']['failure']['episodes'] == 0
    assert all(value is None for key, value in solo_report['outcome_metrics']['failure'].items()
               if key != 'episodes')


def test_outcome_groups_average_only_their_own_first_episodes(mixed_outcomes):
    report, _, _ = mixed_outcomes
    groups = report['outcome_metrics']
    success, failure = report['results']
    assert groups['all']['episodes'] == 2
    assert groups['success']['episodes'] == groups['failure']['episodes'] == 1
    assert success['failure_penalized_duration'] == success['duration']
    assert failure['failure_penalized_duration'] == report['horizon'] * report['dt']
    for metric in groups['all']:
        if metric == 'episodes':
            continue
        assert groups['success'][metric] == success[metric]
        assert groups['failure'][metric] == failure[metric]
        values = [row[metric] for row in report['results'] if row[metric] is not None]
        if values:
            assert groups['all'][metric] == pytest.approx(float(np.mean(values)))
        else:
            assert groups['all'][metric] is None
    assert failure['all_crossed_time'] is failure['recovery_time'] is None
    assert success['all_crossed_time'] is not None and success['recovery_time'] is not None
