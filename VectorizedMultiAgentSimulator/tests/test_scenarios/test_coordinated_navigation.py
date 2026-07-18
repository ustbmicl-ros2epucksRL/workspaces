#  Copyright (c) ProrokLab.
#
#  This source code is licensed under the license found in the
#  LICENSE file in the root directory of this source tree.
import torch

from vmas.scenarios.coordinated_navigation import HeuristicPolicy
from vmas.scenarios.navigation import HeuristicPolicy as NavigationHeuristicPolicy


def test_heuristic_independently_implements_navigation_control_with_goal_memory():
    policy = HeuristicPolicy(continuous_action=True)
    navigation_policy = NavigationHeuristicPolicy(
        continuous_action=True,
        goal_gain=policy.goal_gain,
    )
    assert not isinstance(policy, NavigationHeuristicPolicy)

    positions = [
        torch.tensor([[-1.0, -1.0], [-0.9, 0.8]]),
        torch.tensor([[1.0, 1.0], [0.9, -0.8]]),
    ]
    goals = [
        torch.tensor([[0.4, 0.7], [0.6, -0.5]]),
        torch.tensor([[-0.6, -0.4], [-0.5, 0.6]]),
    ]
    observations = [
        torch.cat(
            (
                position,
                torch.zeros_like(position),
                position - goal,
                torch.zeros(position.shape[0], 12),
            ),
            dim=-1,
        )
        for position, goal in zip(positions, goals)
    ]
    expected_actions = navigation_policy.compute_actions(observations, [1.0, 1.0])
    visible_actions = policy.compute_actions(observations, [1.0, 1.0])

    for action, expected_action in zip(visible_actions, expected_actions):
        torch.testing.assert_close(action, expected_action)

    for remembered_goal, expected_goal in zip(policy._remembered_goals, goals):
        torch.testing.assert_close(remembered_goal, expected_goal)

    occluded_observations = [observation.clone() for observation in observations]
    for observation in occluded_observations:
        observation[:, 4:6] = 0.0
    actions = policy.compute_actions(occluded_observations, [1.0, 1.0])

    assert all(torch.count_nonzero(action) > 0 for action in actions)
    assert all(
        torch.count_nonzero(observation[:, 4:6]) == 0
        for observation in occluded_observations
    )

    policy.reset()
    assert policy._remembered_goals is None
    assert policy._goal_memory_valid is None
    assert policy._previous_positions is None
    assert policy._estimated_motions is None


def test_heuristic_estimates_hidden_velocity_from_position_history():
    policy = HeuristicPolicy(continuous_action=True)
    observation = torch.tensor([[0.0, 0.0, 0.0, 0.0, -1.0, 0.0]])
    policy.compute_actions([observation], [1.0])

    moved_observation = observation.clone()
    moved_observation[:, 0] = 0.1
    moved_observation[:, 4] = -0.9
    policy.compute_actions([moved_observation], [1.0])

    torch.testing.assert_close(
        policy._estimated_motions[0],
        torch.tensor([[0.05, 0.0]]),
    )


def test_heuristic_action_does_not_read_teammate_observations():
    own_observation = torch.tensor([[-0.8, 0.0, 0.0, 0.0, -1.4, 0.0] + [0.0] * 12])
    teammate_a = torch.tensor([[0.8, 0.0, 0.0, 0.0, 1.4, 0.0] + [0.0] * 12])
    teammate_b = teammate_a.clone()
    teammate_b[:, :2] = torch.tensor([[0.0, 0.01]])

    first_policy = HeuristicPolicy(continuous_action=True)
    second_policy = HeuristicPolicy(continuous_action=True)
    first_action = first_policy.compute_actions(
        [own_observation, teammate_a], [1.0, 1.0]
    )[0]
    second_action = second_policy.compute_actions(
        [own_observation, teammate_b], [1.0, 1.0]
    )[0]

    torch.testing.assert_close(first_action, second_action)


def test_heuristic_supports_variable_agent_and_lidar_counts():
    for n_agents, n_lidar_rays in ((3, 8), (7, 16)):
        policy = HeuristicPolicy(continuous_action=True)
        observations = []
        for index in range(n_agents):
            position = torch.tensor([[0.2 * index, -0.1 * index]])
            goal = torch.tensor([[-0.3 + 0.1 * index, 0.4]])
            observations.append(
                torch.cat(
                    (
                        position,
                        torch.zeros_like(position),
                        position - goal,
                        torch.zeros(1, n_lidar_rays),
                    ),
                    dim=-1,
                )
            )

        actions = policy.compute_actions(observations, [1.0] * n_agents)

        assert len(actions) == n_agents
        assert all(action.shape == (1, 2) for action in actions)
        assert all(bool(torch.isfinite(action).all()) for action in actions)
