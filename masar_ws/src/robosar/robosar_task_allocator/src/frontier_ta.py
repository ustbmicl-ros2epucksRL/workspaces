#!/usr/bin/env python3

from queue import Queue

import numpy as np
import rospy
import skimage.measure
import tf
from robosar_messages.msg import *
from robosar_messages.srv import *
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import task_allocator.utils as utils
from generate_graph.gridmap import OccupancyGridMap
from task_allocator.CostCalculator import CostCalculator
from task_allocator.Environment import UnknownEnvironment
from task_allocator.TA import *
from task_commander import TaskCommander
from task_transmitter.task_listener_robosar_control import TaskListenerRobosarControl


class RobotInfo:
    def __init__(self, name="", pos=[], n_tasks=[], costs=[]) -> None:
        self.name = name
        self.pos = pos
        self.curr = None
        self.prev = None
        self.n_tasks = n_tasks
        self.costs = costs
        self.utility = []
        self.obstacle_costs = []
        self.active = False


class FrontierAssignmentCommander(TaskCommander):
    
    def __init__(self):
        super().__init__()
        self.reassign_period = rospy.get_param("~reassign_period", 30.0)
        self.utility_range = rospy.get_param("~utility_range", 1.5)
        timer = rospy.Timer(rospy.Duration(self.reassign_period), self.timer_flag_callback)
        self.rate = rospy.Rate(0.5)
        self.image_pub = rospy.Publisher(
            "task_allocation_image", Image, queue_size=10, latch=True
        )
        rospy.Subscriber(
            "/robosar_agent_bringup_node/status", Bool, self.status_callback
        )
        self.tflistener = tf.TransformListener()
        self.task_listener = None
        self.timer_flag = False
        self.frontiers = []
        self.coverage_tasks = {}
        self.available_tasks = []
        self.map_data = None
        self.map_msg = None
        self.robot_info_dict = {}  # type: dict[str, RobotInfo]
        self.env = None  # type: UnknownEnvironment
        self.gmap = None  # type: OccupancyGridMap
        self.downsample = 2
        self.cost_calculator = CostCalculator(self.utility_range, self.downsample)
        self.n = 5

    def task_graph_client(self):
        task_ids = []
        points = []
        task_types = []
        info_gains = []
        # print("calling task graph getter service")
        rospy.wait_for_service("/robosar_task_generator/task_graph_getter")
        try:
            task_graph_getter_service = rospy.ServiceProxy(
                "/robosar_task_generator/task_graph_getter", task_graph_getter
            )
            resp1 = task_graph_getter_service()
            task_ids = resp1.task_ids
            points = resp1.points
            task_types = resp1.task_types
            info_gains = resp1.info_gains
        except rospy.ServiceException as e:
            print("task graph getter service call failed: %s" % e)

        for i in range(len(task_ids)):
            if task_types[i] == resp1.COVERAGE:
                self.coverage_tasks[task_ids[i]] = [points[i].x, points[i].y]
        return

    def send_visited_to_task_graph(self):
        visited_ids = self.env.get_visited_coverage_tasks()
        # print("calling task graph setter service")
        rospy.wait_for_service("/robosar_task_generator/task_graph_setter")
        try:
            task_graph_getter_service = rospy.ServiceProxy(
                "/robosar_task_generator/task_graph_setter", task_graph_setter
            )
            task_graph_getter_service(visited_ids)
        except rospy.ServiceException as e:
            print("task graph getter service call failed: %s" % e)

    def timer_flag_callback(self, event=None):
        self.timer_flag = True

    def frontier_callback(self, msg):
        points = []
        for point in msg.points:
            points.append([point.x, point.y])
        self.frontiers = np.array(points)

    def get_agent_position(self):
        """
        Get robot positions
        """
        robot_pos = {}
        for name in self.agent_active_status:
            if not self.agent_active_status[name]:
                continue
            now = rospy.Time.now()
            self.tflistener.waitForTransform(
                "map", name + "/base_link", rospy.Time(0), rospy.Duration(60.0)
            )
            now = rospy.Time.now()
            (trans, rot) = self.tflistener.lookupTransform(
                "map", name + "/base_link", rospy.Time(0)
            )
            robot_pos[name] = [trans[0], trans[1]]
        return robot_pos

    def arr_m_to_pixels(self, arr):
        output = []
        for i in arr:
            output.append(utils.m_to_pixels([i[0], i[1]], self.scale, self.origin))
        return np.array(output)

    def rrt_path_cost_client(self, robot_x, robot_y, goal_x, goal_y):
        print("calling rrt_path_cost service")
        rospy.wait_for_service("/frontier_rrt_search_node/rrt_path_cost")
        try:
            rrt_path_service = rospy.ServiceProxy(
                "/frontier_rrt_search_node/rrt_path_cost", rrt_path_cost
            )
            resp1 = rrt_path_service(robot_x, robot_y, goal_x, goal_y)
            return resp1.cost
        except rospy.ServiceException as e:
            print("RRT path service call failed: %s" % e)

    def prepare_costs(self, robot_id):
        rp = self.robot_info_dict[robot_id].pos
        # print(rp)
        # only calculate rrt cost for n euclidean closest frontiers
        n_tasks = self.env.get_n_closest_tasks(n=self.n, robot_pos=rp)
        costs = []
        obstacle_costs = []
        # prox_bonus = []
        for task in n_tasks:
            task_pos = task.pos
            # # RRT path
            # cost = self.rrt_path_cost_client(
            #     rp[0], rp[1], task_pos[0], task_pos[1]
            # )
            # # Euclidean
            # cost = np.linalg.norm(rp - task_pos)
            # A* path
            cost, _ = self.cost_calculator.a_star_cost(rp, task_pos)
            pc = self.cost_calculator.obstacle_cost(task_pos, 1.0)
            # pb = 0.0
            # if self.robot_info_dict[robot_id].prev is not None:
            #     pb = self.cost_calculator.proximity_bonus(
            #         task_pos, self.robot_info_dict[rrobot_id.prev.pos, 2.0
            # )
            costs.append(cost)
            obstacle_costs.append(pc)
            # prox_bonus.append(pb)
        # update robot infos
        self.robot_info_dict[robot_id].prev = self.robot_info_dict[robot_id].curr
        self.robot_info_dict[robot_id].n_tasks = n_tasks
        self.robot_info_dict[robot_id].costs = np.array(costs)
        self.robot_info_dict[robot_id].obstacle_costs = np.array(obstacle_costs)
        # self.robot_info_dict[robot_id].proximity_bonus = np.array(prox_bonus)

    def prepare_env(self):
        # get frontiers
        got_frontiers = True
        try:
            msg = rospy.wait_for_message(
                "/frontier_filter/filtered_frontiers",
                PointArray,
                timeout=self.reassign_period,
            )
            self.frontier_callback(msg)
        except:
            rospy.logwarn("no frontier messages received.")
            self.frontiers = np.array([])
            self.fronters_info_gain = []
            got_frontiers = False

        if len(self.frontiers) == 0:
            got_frontiers = False

        # get new coverage tasks
        got_coverage = self.task_graph_client()

        if not got_frontiers and not got_coverage:
            rospy.logwarn("no tasks received.")
            return False

        # get map
        (
            self.map_msg,
            self.map_data,
            self.scale,
            self.origin,
            self.covered_area,
        ) = self.get_map_info()
        robot_pos = self.get_agent_position()
        for r, rp in robot_pos.items():
            if r not in self.robot_info_dict:
                self.robot_info_dict[r] = RobotInfo(name=r)
                self.robot_info_dict[r].active = True
            self.robot_info_dict[r].pos = rp
        resized_image = skimage.measure.block_reduce(
            self.map_data, (self.downsample, self.downsample), np.max
        )
        self.gmap = OccupancyGridMap.from_data(resized_image)

        # update env
        self.env.update_tasks(self.frontiers, self.coverage_tasks)

        self.cost_calculator.update_map_data(self.gmap, self.map_msg)

        return True

    def reassign(self, name, solver):
        # print("Reassigning")

        # get costs
        self.prepare_costs(name)
        self.env.update_robot_info(self.robot_info_dict)

        # get assignment
        goal = solver.assign(name)
        if goal is not None:
            self.robot_info_dict[name].curr = goal
            # update utility
            self.env.update_utility(goal, self.cost_calculator.utility_discount_fn)
            task_type = 1
            if goal.task_type == "coverage":
                task_type = 2
            return self.robot_info_dict[name].pos, goal.pos, task_type, goal.id

        return [], [], None, None

    def publish_visualize(
        self,
        names,
        starts,
        goals,
        goal_type,
        goal_ids,
        unvisited_coverage=[],
        visited_coverage=[],
    ):
        # publish tasks
        rospy.loginfo("publishing")
        task_msg = task_allocation()
        task_msg.id = [n for n in names]
        task_msg.startx = [s[0] for s in starts]
        task_msg.starty = [s[1] for s in starts]
        task_msg.goalx = [g[0] for g in goals]
        task_msg.goaly = [g[1] for g in goals]
        task_msg.goal_type = [t for t in goal_type]
        task_msg.goal_id = goal_ids
        while self.task_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscriber to task topic:")
            rospy.sleep(1)
        self.task_pub.publish(task_msg)

        # plot
        utils.plot_pgm_data(self.map_data)
        if len(self.frontiers) > 0:
            pix_frontier = self.arr_m_to_pixels(self.frontiers)
            plt.plot(pix_frontier[:, 0], pix_frontier[:, 1], "go", zorder=100)
        if len(unvisited_coverage) > 0:
            pix_coverage = self.arr_m_to_pixels(unvisited_coverage)
            plt.plot(pix_coverage[:, 0], pix_coverage[:, 1], "co", zorder=100)
        if len(visited_coverage) > 0:
            pix_coverage = self.arr_m_to_pixels(visited_coverage)
            plt.plot(pix_coverage[:, 0], pix_coverage[:, 1], "bo", zorder=100)
        for id in self.agent_active_status:
            if self.agent_active_status[id]:
                pix_rob = utils.m_to_pixels(
                    self.robot_info_dict[id].pos, self.scale, self.origin
                )
                if self.robot_info_dict[id].curr is not None:
                    pix_goal = utils.m_to_pixels(
                        self.robot_info_dict[id].curr.pos, self.scale, self.origin
                    )
                    plt.plot(pix_rob[0], pix_rob[1], "ro", zorder=100)
                    plt.plot(
                        [pix_rob[0], pix_goal[0]],
                        [pix_rob[1], pix_goal[1]],
                        "k-",
                        zorder=90,
                    )
                    # An assigned task leaves the available-task scatter set,
                    # so redraw it with the same semantic circle instead of a
                    # generic endpoint cross.
                    task_marker = (
                        "co"
                        if self.robot_info_dict[id].curr.task_type == "coverage"
                        else "go"
                    )
                    plt.plot(pix_goal[0], pix_goal[1], task_marker, zorder=110)
        self.publish_image(self.image_pub)
        plt.clf()

    def fleet_update(self):
        # fleet update
        if self.callback_triggered:
            self.get_active_agents()
            for agent, active in self.agent_active_status.items():
                if not active and agent in self.robot_info_dict and self.robot_info_dict[agent].active:
                    rospy.logwarn("FLEET UPDATE: {} died".format(agent))
                elif active and agent in self.robot_info_dict and not self.robot_info_dict[agent].active:
                    rospy.logwarn("FLEET UPDATE: {} is back".format(agent))
                elif active and agent not in self.robot_info_dict:
                    rospy.logwarn("FLEET UPDATE: new agent {}".format(agent))
                    self.robot_info_dict[agent] = RobotInfo(name=agent)
                    self.task_listener.add_robot_listener(agent)
                self.robot_info_dict[agent].active = active
            self.callback_triggered = False

    def execute(self):
        """
        Uses frontiers as tasks
        """
        # Get active agents
        self.get_active_agents()

        # Get map
        self.map_msg, self.map_data, self.scale, self.origin, _ = self.get_map_info()

        # Get frontiers
        rospy.loginfo("Waiting for frontiers")
        while len(self.frontiers) == 0:
            msg = rospy.wait_for_message(
                "/frontier_filter/filtered_frontiers", PointArray, timeout=None
            )
            self.frontier_callback(msg)

        # get robot position
        self.tflistener.waitForTransform(
            "map",
            list(self.agent_active_status.keys())[0] + "/base_link",
            rospy.Time(),
            rospy.Duration(1.0),
        )
        robot_pos = self.get_agent_position()

        # Create TA
        for name, active in self.agent_active_status.items():
            robot_info = RobotInfo(name=name)
            robot_info.active = active
            robot_info.pos = robot_pos[name] if name in robot_pos else []
            self.robot_info_dict[name] = robot_info
        self.env = UnknownEnvironment(
            frontier_tasks=self.frontiers, robot_info=self.robot_info_dict
        )
        solver = TA_frontier_greedy(self.env)

        # Create listener object
        self.task_listener = TaskListenerRobosarControl(
            [name for name in self.agent_active_status]
        )

        # plot
        utils.plot_pgm_data(self.map_data)
        pix_frontier = self.arr_m_to_pixels(self.frontiers)
        plt.plot(pix_frontier[:, 0], pix_frontier[:, 1], "go", zorder=100)
        for pos in robot_pos.values():
            pix_rob = utils.m_to_pixels(pos, self.scale, self.origin)
            plt.plot(pix_rob[0], pix_rob[1], "ro", zorder=100)
        self.publish_image(self.image_pub)

        # every time robot reaches a frontier, or every 5 seconds, reassign
        # get robot positions, updates env
        # cost: rrt path, utility: based on paper below
        # https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1435481&tag=1
        # 1. greedy assignment
        # 2. hungarian assignment
        # 3. mtsp
        rospy.loginfo("Starting frontier task allocator")

        self.task_graph_client()
        self.prepare_env()
        names = []
        starts = []
        goals = []
        goal_types = []
        goal_ids = []
        for name in self.agent_active_status:
            start, goal, goal_type, goal_id = self.reassign(name, solver)
            if len(start) > 0:
                names.append(name)
                starts.append(start)
                goals.append(goal)
                goal_types.append(goal_type)
                goal_ids.append(goal_id)
                self.task_listener.setBusyStatus(name)
        if len(names) > 0:
            self.publish_visualize(names, starts, goals, goal_types, goal_ids)
        self.timer_flag = False

        while not rospy.is_shutdown():
            # fleet update
            self.fleet_update()

            # get frontiers
            if not self.prepare_env():
                continue
            
            curr_active_robots = []
            for name, active in self.agent_active_status.items():
                if active:
                    curr_active_robots.append(name)
            agent_reached = {name: False for name in curr_active_robots}
            agent_reached_flag = False
            for name in curr_active_robots:
                status = self.task_listener.getStatus(name)
                if status == 2:
                    agent_reached[name] = True
                    agent_reached_flag = True

            if self.timer_flag or agent_reached_flag:
                # get new coverage tasks
                self.task_graph_client()

                unvisited_coverage = self.env.get_unvisited_coverage_tasks_pos()
                visited_coverage = self.env.get_visited_coverage_tasks_pos()

                # reassign
                names = []
                starts = []
                goals = []
                goal_types = []
                goal_ids = []
                for name in curr_active_robots:
                    if (
                        self.robot_info_dict[name].curr
                        and self.robot_info_dict[name].curr.task_type == "coverage"
                        and not agent_reached[name]
                    ):
                        self.env.update_utility(self.robot_info_dict[name].curr, self.cost_calculator.utility_discount_fn)
                        continue
                    start, goal, goal_type, goal_id = self.reassign(name, solver)
                    if len(start) > 0:
                        names.append(name)
                        starts.append(start)
                        goals.append(goal)
                        goal_types.append(goal_type)
                        goal_ids.append(goal_id)
                        self.task_listener.setBusyStatus(name)
                    # print("{}: status {}".format(name, self.task_listener.getStatus(name)))
                self.send_visited_to_task_graph()
                self.timer_flag = False

                if len(names) > 0:
                    self.publish_visualize(
                        names,
                        starts,
                        goals,
                        goal_types,
                        goal_ids,
                        unvisited_coverage,
                        visited_coverage,
                    )

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("task_commander", anonymous=False, log_level=rospy.INFO)

    try:
        tc = FrontierAssignmentCommander()
        tc.execute()
    except rospy.ROSInterruptException:
        pass
    plt.close("all")
