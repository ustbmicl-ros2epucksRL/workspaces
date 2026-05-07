"""
Task Allocation classes
- TA_greedy: greedy task allocator
- TA_mTSP: Multiple Traveling Salesman Problem task allocator
- TA_frontier_greedy: greedy task allocator with dynamic frontiers
"""

from abc import ABC, abstractmethod
import rospy

import matplotlib.pyplot as plt

# import mTSP_utils
import numpy as np
import task_allocator.mTSP_utils as mTSP_utils
import task_allocator.utils as utils
from scipy.optimize import linear_sum_assignment


class TA(ABC):
    """
    Task allocation abstract class
    """

    def init(self, env=None):
        """
        initializes task allocation object
        env: Environment object
        """
        self.env = env

    def reached(self, name, curr_node):
        """
        called after robot completes task
        name: string
        curr_node: int
        """
        r = self.env.robots[name]
        if r.prev is not curr_node:
            r.prev = curr_node
            r.visited.append(curr_node)
            self.env.frontier.remove(curr_node)
            self.env.visited.add(curr_node)
            print("Robot {} reached node {}".format(name, curr_node))
            if len(self.env.visited) + len(self.env.frontier) < self.env.num_nodes:
                self.assign(name, curr_node)
        else:
            r.done = True

    @abstractmethod
    def assign(self, name, curr_node):
        """
        called by reached() to assign robot its next task
        name: string
        curr_node: int
        """
        pass


class TA_greedy(TA):
    """
    TA_greedy: greedy task allocator
    """

    def init(self, env):
        super().init(env)
        self.objective_value = [0] * len(self.env.robots)

    def assign(self, name, curr_node):
        # Find next unvisited min cost task
        C = self.env.adj[curr_node, :]
        robot = self.env.robots[name]
        min_node_list = np.argsort(C)
        min_node = -1
        for i in min_node_list:
            if C[i] > 0 and i not in self.env.visited and i not in self.env.frontier:
                min_node = i
                break
        # No feasible path to any task
        if min_node == -1:
            print("USING EUCLIDEAN DISTANCE")
            E = []
            idx = []
            for i, node in enumerate(self.env.nodes):
                if i not in self.env.visited and i not in self.env.frontier:
                    idx.append(i)
                    E.append(
                        np.sqrt(
                            (node[0] - robot.pos[0]) ** 2
                            + (node[1] - robot.pos[1]) ** 2
                        )
                    )
            if E:
                min_node_i = np.argmin(np.array(E))
                min_node = idx[min_node_i]

        print(
            "Assigned {}: node {} at {}".format(
                name, min_node, self.env.nodes[min_node]
            )
        )
        plt.plot(
            self.env.nodes[min_node][0], self.env.nodes[min_node][1], "go", zorder=101
        )
        robot.next = min_node
        self.objective_value[self.env.get_robot_id(name)] += self.env.adj[robot.prev][
            robot.next
        ]
        self.env.frontier.add(min_node)


class TA_mTSP(TA):
    """
    TA_mTSP: Multiple Traveling Salesman Problem task allocator
    """

    def init(self, env, timeout=5):
        super().init(env)
        self.timeout = timeout
        self.tours = self.calculate_mtsp(True)
        self.objective_value = [0] * len(self.env.robots)

    def reached(self, name, curr_node):
        r = self.env.robots[name]
        # print("{}: {}".format(id, self.tours[self.env.id_dict[id]]))
        self.tours[self.env.get_robot_id(name)].pop(0)
        if not self.tours[self.env.get_robot_id(name)]:  # if robot has finished
            plt.plot(
                self.env.nodes[r.prev][0], self.env.nodes[r.prev][1], "go", zorder=200
            )
            r.done = True
        if r.next is None:  # first assignment
            plt.plot(
                self.env.nodes[r.prev][0], self.env.nodes[r.prev][1], "go", zorder=200
            )
            self.assign(name, curr_node)
        elif r.prev is not curr_node:  # if robot not done
            r.prev = curr_node
            r.visited.append(curr_node)
            plt.plot(
                self.env.nodes[r.prev][0], self.env.nodes[r.prev][1], "go", zorder=200
            )
            self.env.frontier.remove(curr_node)
            self.env.visited.add(curr_node)
            print("Robot {} reached node {}".format(name, curr_node))
            if len(self.env.visited) + len(self.env.frontier) < self.env.num_nodes:
                self.assign(name, curr_node)
        elif self.tours[
            self.env.get_robot_id(name)
        ]:  # if robot was done, but is now reassigned
            r.done = False
            if len(self.env.visited) + len(self.env.frontier) < self.env.num_nodes:
                self.assign(name, curr_node)
        # plt.pause(0.1)

    def assign(self, name, curr_node):
        robot = self.env.robots[name]
        if self.tours[self.env.id_dict[name]]:
            min_node = self.tours[self.env.id_dict[name]][0]
            print(
                "Assigned {}: node {} at {}".format(
                    name, min_node, self.env.nodes[min_node]
                )
            )
            robot.next = min_node
            self.objective_value[self.env.get_robot_id(name)] += self.env.adj[
                robot.prev
            ][robot.next]
            self.env.frontier.add(min_node)

    def get_next_adj(self):
        """
        Get cost matrix for unvisited tasks
        """
        to_visit = []
        starts = [r.next for r in self.env.robots.values()]
        for i in range(self.env.num_nodes):
            if i not in self.env.visited or i in starts:
                to_visit.append(i)

        adj = np.zeros((len(to_visit), len(to_visit)))
        for idx1, n1 in enumerate(to_visit):
            for idx2, n2 in enumerate(to_visit):
                adj[idx1][idx2] = self.env.adj[n1][n2]
        start_idx = [to_visit.index(s) for s in starts]
        adj = self.tsp2hamiltonian(adj, start_idx)
        return adj, to_visit

    def calculate_mtsp(self, initial=True):
        """
        Uses Google OR-Tools VRP Library to solve for optimal tours
        initial: Bool
        """
        data = {}
        starts = [r.prev for r in self.env.robots.values()]
        if initial:  # first solution
            adj = self.tsp2hamiltonian(self.env.adj, starts)
            data["starts"] = starts
            data["ends"] = [self.env.num_nodes + i for i in range(self.env.num_robots)]
        else:
            adj, to_visit = self.get_next_adj()
            # prev = {r.id: r.prev for r in self.env.robots.values()}
            data["starts"] = [to_visit.index(r.next) for r in self.env.robots.values()]
            data["ends"] = [len(to_visit) + i for i in range(self.env.num_robots)]
        data["num_vehicles"] = len(self.env.robots)
        data["distance_matrix"] = adj
        tours = mTSP_utils.solve(data, self.timeout)

        if not initial:
            self.tours = [
                [to_visit[tour[i]] for i in range(1, len(tour))] for tour in tours
            ]
            for name, robot in self.env.robots.items():
                if len(self.tours[self.env.get_robot_id(name)]) > 1:
                    robot.done = False
                else:
                    robot.done = True

        return tours

    def tsp2hamiltonian(self, adj, starts):
        """
        Converts TSP problem formulation to Hamiltonian Path
        adj: nxn np.array
        starts: n list
        """
        adj_new = np.zeros(
            (len(adj) + self.env.num_robots, len(adj) + self.env.num_robots)
        )
        adj_new[: len(adj), : len(adj)] = adj
        for i in range(self.env.num_robots):
            for j in range(i + 1, self.env.num_robots):
                adj_new[len(adj) + i, len(adj) + j] = 10e4
                adj_new[len(adj) + j, len(adj) + i] = 10e4
        return adj_new


class TA_frontier_greedy(TA):
    """
    round robin greedy assignment
    """

    def __init__(self, env):
        # (UnknownEnvironment, float) -> None
        super().init(env)
        self.robot_info_dict = env.robot_info_dict

    def assign(self, name):
        # (str) -> Task
        robot_info = self.robot_info_dict[name]
        # cost_fn
        dist_cost = robot_info.costs / np.max(robot_info.costs)
        n_utility = self.env.get_utility_arr_from_ntasks(robot_info.n_tasks)
        cost_fn = (
            0.5 * dist_cost
            - 0.5 * n_utility
            # + 0.2 * robot_info.obstacle_costs
            # - 0.1 * robot_info.proximity_bonus
        )
        # print("costs: ", robot_info.costs)
        # print("utility: ", n_utility)
        # print("obs: ", robot_info.obstacle_costs)
        # print("prox: ", robot_info.proximity_bonus)
        # print("tot: ", cost_fn)
        # get least cost node
        min_node_list = np.argsort(cost_fn)
        min_node = None
        for i in min_node_list:
            if not robot_info.n_tasks[i].visited:
                min_node = robot_info.n_tasks[i]
                min_node.visited = True
                break
        if min_node is None:
            print("{} unused".format(name))
            return min_node

        print(
            "Assigned {}: {} task {} at {}".format(
                name, min_node.task_type, min_node.id, min_node.pos
            )
        )

        return min_node


class TA_HIGH(TA):
    """
    HIGH Task Allocator: Hierarchical Information Gain Heuristic
    """

    def __init__(self, env):
        # (UnknownEnvironment, float) -> None
        super().init(env)
        self.robot_info_dict = env.robot_info_dict

    def prepare_costs(self, task, avail_robots, cost_calculator):
        dist_cost = np.zeros(
            (
                len(
                    avail_robots,
                )
            )
        )
        for i, r in enumerate(avail_robots):
            dist_cost[i], _ = cost_calculator.a_star_cost(task.pos, r.pos)
        return dist_cost

    def prepare_task_costs(self, tasks, robot_pos, cost_calculator):
        dist_cost = np.zeros(
            (
                len(
                    tasks,
                )
            )
        )
        for i, task in enumerate(tasks):
            dist_cost[i], _ = cost_calculator.a_star_cost(task.pos, robot_pos)
        return dist_cost

    def prepare_e2_weights(self, tasks):
        # Calculate explore-exploit weights
        exploration_weight = self.env.exploration_weight
        exploit_weight = 1 - exploration_weight
        e2_weights = np.zeros((len(tasks)))
        for i, task in enumerate(tasks):
            if task.task_type == "frontier":
                e2_weights[i] = exploration_weight
            else:
                e2_weights[i] = exploit_weight
        return e2_weights

    def get_relevant_tasks(self, robot_pos, cost_calculator):
        dist_cost = []
        n_tasks = []
        # closest euclidean frontier tasks
        f_tasks = self.env.get_n_closest_tasks(5, robot_pos, "frontier")
        
        # highest info gain coverage tasks
        c_tasks = self.env.get_highest_priority_coverage_tasks()[:5]
        tasks = f_tasks + c_tasks # concatenate

        # calculate distances
        for task in tasks:
            cost, found_flag = cost_calculator.a_star_cost(task.pos, robot_pos, 5000)
            if found_flag:
                dist_cost.append(cost)
                n_tasks.append(task)
            else:
                rospy.logwarn("{} could not find astar path".format(task.pos))

        return n_tasks, np.array(dist_cost)

    def reached(self, robot_info, goal_id):
        rospy.logwarn(
            "{} reached task {}, current task is {}".format(
                robot_info.name,
                goal_id,
                robot_info.curr.id if robot_info.curr else None,
            )
        )
        if goal_id is not None and goal_id in self.env.coverage_tasks_dict:
            self.env.coverage_tasks_dict[goal_id].visited = True

    def assign(self, names, cost_calculator):
        # (List[str], CostCalculator) -> Task
        assigned_names = []
        assigned_tasks = []
        avail_robots = set(names)
        rospy.logwarn("calculating")

        for robot_id in names:
            # set current task to unassigned
            if self.robot_info_dict[robot_id].curr is not None:
                self.robot_info_dict[robot_id].curr.assigned = False
                self.robot_info_dict[robot_id].curr = None
        
        for robot_id in names:
            # closest tasks
            rp = self.robot_info_dict[robot_id].pos
            n_tasks, dist_fn = self.get_relevant_tasks(rp, cost_calculator)
            if len(n_tasks) == 0:
                print("{} unused".format(robot_id))
                continue
            # costs to each task
            dist_cost_fn = dist_fn / np.max(dist_fn)
            # calculate task priorities based on info_gain
            info_gain = np.array([n_tasks[i].info_gain for i in range(len(n_tasks))])
            utility_fn = np.array([n_tasks[i].utility for i in range(len(n_tasks))])
            e2_weights = self.prepare_e2_weights(n_tasks)
            reward_fn = (2 * utility_fn - dist_cost_fn) * info_gain * e2_weights

            # print
            for i, t in enumerate(n_tasks):
                print(
                    "task_pos: {}, e2 weight: {}, dist cost: {}, info_gain: {}, utility: {}, reward_fn: {}".format(
                        t.pos,
                        e2_weights[i],
                        dist_cost_fn[i],
                        info_gain[i],
                        utility_fn[i],
                        reward_fn[i],
                    )
                )

            best_node_list = np.argsort(reward_fn)[::-1]
            best_node = None
            for i in best_node_list:
                rospy.logwarn(
                    "{} task {} is visited: {}, assigned: {}".format(
                        n_tasks[i].task_type, n_tasks[i].id, n_tasks[i].visited, n_tasks[i].assigned
                    )
                )
                if not n_tasks[i].visited and not n_tasks[i].assigned:
                    best_node = n_tasks[i]
                    if n_tasks[i].task_type == "frontier":
                        best_node.visited = True
                    best_node.assigned = True
                    break
            if best_node is None:
                print("{} unused".format(robot_id))
                continue

            print(
                "Assigned {}: {} task {} at {}\n".format(
                    robot_id, best_node.task_type, best_node.id, best_node.pos
                )
            )
            self.env.update_utility(best_node, cost_calculator.utility_discount_fn)
            avail_robots.remove(robot_id)
            assigned_names.append(robot_id)
            assigned_tasks.append(best_node)

        return assigned_names, assigned_tasks

    # def assign(self, names, cost_calculator):
    #     # (List[str], CostCalculator) -> Task
    #     assigned_names = []
    #     assigned_tasks = []
    #     num_avail_robots = len(names)
    #     avail_robots = set(names)
    #     avail_tasks = self.env.available_tasks
    #     n_assign = min(num_avail_robots, len(avail_tasks))

    #     # Calculate explore-exploit weights
    #     e2_weights = self.prepare_e2_weights(avail_tasks)
    #     rospy.logwarn("e2 weights: {}".format(e2_weights))

    #     # Assign a robot to each task in priority order
    #     for it in range(n_assign):
    #         # calculate task priorities based on (info_gain+utility)*e2_weight
    #         info_gains = [avail_tasks[i].info_gain for i in range(len(avail_tasks))]
    #         rospy.logwarn("info_gains: {}".format(info_gains))
    #         priority_fn = [(avail_tasks[i].info_gain + avail_tasks[i].utility) * e2_weights[i] for i in range(len(avail_tasks))]
    #         rospy.logwarn("priority fn: {}".format(priority_fn))
    #         idx = np.argsort(priority_fn)[::-1]
    #         # get highest priority task
    #         task = None
    #         for i in idx:
    #             if not avail_tasks[i].visited:
    #                 task = avail_tasks[i]
    #                 break
    #         if task is None:
    #             return assigned_names, assigned_tasks
    #         # costs to each robot
    #         avail_robot_list = [self.robot_info_dict[name] for name in avail_robots]
    #         cost_fn = self.prepare_costs(task, avail_robot_list, cost_calculator)
    #         min_robot_idx = np.argmin(cost_fn)
    #         min_robot = avail_robot_list[min_robot_idx]
    #         # mark visited
    #         task.visited = True
    #         self.env.update_utility(task, cost_calculator.utility_discount_fn)
    #         avail_robots.remove(min_robot.name)
    #         assigned_names.append(min_robot.name)
    #         assigned_tasks.append(task)

    #         print("Assigned {}: {} task {} at {}".format(
    #             min_robot.name, task.task_type, task.id, task.pos
    #         ))

    #     return assigned_names, assigned_tasks

class TA_HIGH_OA(TA_HIGH):
    """
    HIGH Task Allocator: Hierarchical Information Gain Heuristic
    With optimal assignment
    """

    def create_cost_matrix(self, names, task_list, cached_dist, cost_calculator):
        C = np.zeros((len(names), len(task_list)))
        for i, robot_id in enumerate(names):
            for j, task in enumerate(task_list):
                rp = self.robot_info_dict[robot_id].pos
                if task.id in cached_dist[i]:
                    cost = cached_dist[i][task.id]
                else:
                    cost, _ = cost_calculator.a_star_cost(task.pos, rp, 5000)
                C[i,j] = cost
        return C

    def get_optimal_assignment(self, C, names, task_list):
        assigned_dict = {}
        row_ind, col_ind = linear_sum_assignment(C)
        for i in range(len(row_ind)):
            assigned_dict[names[row_ind[i]]] = task_list[col_ind[i]]
        return assigned_dict


    def assign(self, names, cost_calculator):
        # (List[str], CostCalculator) -> Task
        assigned_names = []
        assigned_tasks = []
        greedy_assigned_tasks = []
        cached_dist = [{} for _ in names]
        avail_robots = set(names)
        rospy.logwarn("calculating")

        for robot_id in names:
            # set current task to unassigned
            if self.robot_info_dict[robot_id].curr is not None:
                self.robot_info_dict[robot_id].curr.assigned = False
                self.robot_info_dict[robot_id].curr = None
        
        for i, robot_id in enumerate(names):
            # closest tasks
            rp = self.robot_info_dict[robot_id].pos
            n_tasks, dist_fn = self.get_relevant_tasks(rp, cost_calculator)
            if len(n_tasks) == 0:
                print("{} unused".format(robot_id))
                continue
            # costs to each task
            for j in range(len(n_tasks)):
                cached_dist[i][n_tasks[j].id] = dist_fn[j]
            dist_cost_fn = dist_fn / np.max(dist_fn)
            # calculate task priorities based on info_gain
            info_gain = np.array([n_tasks[i].info_gain for i in range(len(n_tasks))])
            utility_fn = np.array([n_tasks[i].utility for i in range(len(n_tasks))])
            e2_weights = self.prepare_e2_weights(n_tasks)
            reward_fn = (2 * utility_fn - dist_cost_fn) * info_gain * e2_weights
            # greedy assignment
            best_node_list = np.argsort(reward_fn)[::-1]
            best_node = None # Task
            for i in best_node_list:
                if not n_tasks[i].visited and not n_tasks[i].assigned:
                    best_node = n_tasks[i]
                    if n_tasks[i].task_type == "frontier":
                        best_node.visited = True
                    best_node.assigned = True
                    # plot
                    # pix_rp = utils.m_to_pixels(
                    #     rp, cost_calculator.scale, cost_calculator.origin
                    # )
                    # pix_goal = utils.m_to_pixels(
                    #     best_node.pos, cost_calculator.scale, cost_calculator.origin
                    # )
                    # plt.plot([pix_rp[0], pix_goal[0]], [pix_rp[1], pix_goal[1]], '-m')
                    break
            if best_node:
                greedy_assigned_tasks.append(best_node)
        
        print("Tasks: ", [t.pos for t in greedy_assigned_tasks])
        C = self.create_cost_matrix(names, greedy_assigned_tasks, cached_dist, cost_calculator)
        print("C: ", C)
        assigned_dict = self.get_optimal_assignment(C, names, greedy_assigned_tasks)
        for robot_id, task in assigned_dict.items():
            self.env.update_utility(task, cost_calculator.utility_discount_fn)
            avail_robots.remove(robot_id)
            assigned_names.append(robot_id)
            assigned_tasks.append(task)
            print(
                "Assigned {}: {} task {} at {}".format(
                    robot_id, task.task_type, task.id, task.pos
                )
            )

        return assigned_names, assigned_tasks
