"""
Environment class
- Represents map as a graph, dictionary of robots, and maintains set of visited and frontier nodes
"""

import numpy as np
import heapq

# from Robot import Robot
from task_allocator.Robot import Robot


class Environment:
    def __init__(self, nodes=[], adj=[], robots={}):
        """
        nodes: nx2 np.array of task coordinates
        adj: nxn np.array adjacency matrix
        robots: {string: Robot} dictionary
        """
        self.nodes = nodes
        self.adj = adj
        # fill in adjacency matrix to be fully connected
        for i in range(len(adj)):
            for j in range(i, len(adj)):
                if i == j:
                    self.adj[i][j] = 0
                elif self.adj[i][j] < 0:
                    d = self.dijkstra(i, j)
                    if d < 0:
                        print(
                            "WARNING: could not find path from {} to {}".format(
                                nodes[i], nodes[j]
                            )
                        )
                        d = 3 * np.linalg.norm(nodes[i] - nodes[j])
                    self.adj[i][j] = d
                    self.adj[j][i] = self.adj[i][j]

        self.num_nodes = len(adj)
        self.robots = robots
        self.id_dict = {}
        for idx, name in enumerate(self.robots.keys()):
            self.id_dict[name] = idx
        self.num_robots = len(self.robots)
        self.visited = set()
        for r in self.robots.values():
            self.visited.add(r.prev)
        self.frontier = set()

    def get_robot_id(self, name):
        """
        name: string
        """
        return self.id_dict[name]

    def add_robot(self, name, start):
        """
        id: int
        start: int node number
        """
        robot = Robot(name, self.nodes[start], start)
        self.robots[name] = robot
        self.num_robots = len(self.robots)
        self.id_dict[name] = self.num_robots - 1
        self.visited.add(robot.prev)

    def remove_robot(self, agent):
        """
        agent: string
        """
        self.robots.pop(agent)
        self.id_dict.pop(agent)
        for idx, name in enumerate(self.robots.keys()):
            self.id_dict[name] = idx
        self.num_robots = len(self.robots)

    def fleet_update(self, agent_active_status):
        """
        agent_active_status: {id: Bool}
        """
        for agent, active in agent_active_status.items():
            if not active and agent in self.robots:
                print("FLEET UPDATE: {} died".format(agent))
                self.robots.pop(agent)
                self.id_dict.pop(agent)
        for idx, name in enumerate(self.robots.keys()):
            self.id_dict[agent] = idx
        self.num_robots = len(self.robots)

    def dijkstra(self, start, goal):
        """
        start: int
        goal: int
        """
        m = self.adj.shape[0]
        dist = [float("inf") for _ in range(m)]
        dist[start] = 0
        minHeap = [(0, start)]  # distance, node
        while minHeap:
            d, s = heapq.heappop(minHeap)
            if d > dist[s]:
                continue
            if s == goal:
                return d  # Reach to goal
            neighbors = []
            for i, cost in enumerate(self.adj[s]):
                if cost > 0:
                    neighbors.append(i)
            for i in neighbors:
                newDist = d + self.adj[s][i]
                if dist[i] > newDist:
                    dist[i] = newDist
                    heapq.heappush(minHeap, (dist[i], i))
        return -1


class UnknownEnvironment:
    def __init__(self, frontier_tasks=[], robot_info={}) -> None:
        """
        Takes in frontier nodes and list of robots
        """
        # dict[int, RobotInfo]
        self.robot_info_dict = robot_info
        # frontiers are rewritten every time update() is called
        self.frontier_tasks = frontier_tasks  # List[Task]
        # coverage tasks persist and are updated every time update() is called
        self.coverage_tasks_dict = {}  # Dict[int, Task]
        # total unvisited tasks
        self.available_tasks = []  # List[Task]
        # reset every time update() is called
        self.utility = np.ones((len(self.available_tasks),))
        self.exploration_weight = 1.0
        self.map_msg = None

    def euclidean(self, x1, x2):
        return np.linalg.norm([x1[0] - x2[0], x1[1] - x2[1]])

    def get_highest_priority_coverage_tasks(self):
        avail_coverage = self.get_unvisited_coverage_tasks()
        sorted_tasks = sorted(
            avail_coverage, key=lambda t: t.info_gain, reverse=True
        )
        return sorted_tasks

    def get_visited_coverage_tasks(self):
        visited = []
        for id, ct in self.coverage_tasks_dict.items():
            if ct.visited:
                visited.append(id)
        return visited

    def get_visited_coverage_tasks_pos(self):
        visited = []
        for ct in self.coverage_tasks_dict.values():
            if ct.visited:
                visited.append(ct.pos)
        return np.array(visited)

    def get_unvisited_coverage_tasks(self):
        unvisited = []
        for id, ct in self.coverage_tasks_dict.items():
            if not ct.visited:
                unvisited.append(ct)
        return unvisited

    def get_unvisited_coverage_tasks_pos(self):
        unvisited = []
        for ct in self.coverage_tasks_dict.values():
            if not ct.visited:
                unvisited.append(ct.pos)
        return np.array(unvisited)

    def update_utility(self, goal, utility_discount_fn):
        # (Task, lambda_fn) -> None
        if goal == None:
            return
        goal_pos = goal.pos
        for i in range(len(self.available_tasks)):
            node_pos = self.available_tasks[i].pos
            p = utility_discount_fn(goal_pos, node_pos)
            self.available_tasks[i].utility = max(0.0, self.available_tasks[i].utility-p)

    def reset_utility(self):
        for task in self.coverage_tasks_dict.values():
            task.utility = 1.0

    def get_utility_arr_from_ntasks(self, n_tasks):
        n_utility = np.zeros((len(n_tasks),))
        for idx, task in enumerate(n_tasks):
            n_utility[idx] = task.utility
        return n_utility

    def get_available_task_pos(self):
        task_pos = np.zeros((len(self.available_tasks), 2))
        for idx, task in enumerate(self.available_tasks):
            task_pos[idx, :] = task.pos
        return task_pos

    def get_n_closest_tasks(self, n, robot_pos, task_type=None):
        if task_type == "frontier":
            avail_task_pos = np.array([f.pos for f in  self.frontier_tasks])
            avail_tasks = self.frontier_tasks
        elif task_type == "coverage":
            avail_task_pos = self.get_unvisited_coverage_tasks_pos()
            avail_tasks = self.get_unvisited_coverage_tasks()
        else:
            avail_task_pos = self.get_available_task_pos()
            avail_tasks = self.available_tasks
        assert(len(avail_tasks) == len(avail_task_pos))
        if len(avail_tasks) == 0:
            return []
        C = np.linalg.norm(avail_task_pos - robot_pos, axis=1)
        min_node_list = np.argsort(C)
        n_tasks_idx = min_node_list[:n]
        return [avail_tasks[t] for t in n_tasks_idx]

    def update_tasks(
        self,
        frontier_tasks,
        coverage_tasks_dict={},
        frontier_info_gain=None,
    ):
        self.frontier_tasks = []
        for id, ft in enumerate(frontier_tasks):
            if frontier_info_gain is not None:
                self.frontier_tasks.append(
                    Task(
                        task_type="frontier",
                        pos=ft,
                        info_gain=frontier_info_gain[id],
                        id=id,
                    )
                )
            else:
                self.frontier_tasks.append(Task(task_type="frontier", pos=ft, id=id))
        for id, ct in coverage_tasks_dict.items():
            if id in self.coverage_tasks_dict:
                self.coverage_tasks_dict[id].pos = ct[:2]
                if len(ct) > 2:
                    self.coverage_tasks_dict[id].info_gain = ct[2]
            else:
                if len(ct) > 2:
                    self.coverage_tasks_dict[id] = Task(task_type="coverage", pos=ct[:2], id=id, info_gain=ct[2])
                else:
                    self.coverage_tasks_dict[id] = Task(task_type="coverage", pos=ct, id=id)
        self.available_tasks = []
        for task in self.frontier_tasks:
            self.available_tasks.append(task)
        for task in self.get_unvisited_coverage_tasks():
            self.available_tasks.append(task)
        self.reset_utility()

    def update_robot_info(self, robot_info_dict={}):
        self.robot_info_dict = robot_info_dict


class Task:
    def __init__(
        self, task_type="frontier", pos=[0.0, 0.0], info_gain=0.0, id=0
    ) -> None:
        self.task_type = task_type
        self.pos = pos
        self.id = id
        self.utility = 1.0
        self.info_gain = info_gain
        self.visited = False
        self.assigned = False
