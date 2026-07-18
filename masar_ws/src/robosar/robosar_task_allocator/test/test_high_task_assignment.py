import numpy as np

from task_allocator.Environment import Task, UnknownEnvironment
from task_allocator.TA import TA_HIGH_OA


class _Robot:
    def __init__(self, position):
        self.pos = np.array(position, dtype=float)
        self.curr = None


class _EuclideanCost:
    @staticmethod
    def a_star_cost(goal, start, *_args):
        return float(np.linalg.norm(np.asarray(goal) - np.asarray(start))), True

    @staticmethod
    def utility_discount_fn(goal, node):
        distance = float(np.linalg.norm(np.asarray(goal) - np.asarray(node)))
        return max(0.0, 1.0 - distance / 1.5)


def test_high_optimal_assignment_discounts_selected_frontier_cluster_immediately():
    names = ["agent_{}".format(index) for index in range(5)]
    robots = {name: _Robot((0.0, 0.0)) for name in names}
    east_cluster = [(1.0 + 0.1 * index, 0.0) for index in range(5)]
    distributed = [(-2.0, 0.0), (0.0, 2.0), (0.0, -2.0), (-1.5, 1.5)]
    tasks = [
        Task(task_type="frontier", pos=np.array(position), info_gain=1.0, id=index)
        for index, position in enumerate(east_cluster + distributed)
    ]
    env = UnknownEnvironment(frontier_tasks=tasks, robot_info=robots)
    env.available_tasks = list(tasks)
    allocator = TA_HIGH_OA(env)

    _assigned_names, assigned_tasks = allocator.assign(names, _EuclideanCost())

    selected = [tuple(task.pos) for task in assigned_tasks]
    assert len(selected) == len(names)
    assert any(position not in east_cluster for position in selected)
