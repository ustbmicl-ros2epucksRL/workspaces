#!/usr/bin/env python3
"""
Greedy Commander
- This is deprecated lol
"""

from task_commander import TaskCommander
import rospy
from task_allocator.Environment import Environment
from task_allocator.Robot import Robot
from task_allocator.TA import *
from task_transmitter.task_tx_move_base import TaskTxMoveBase
import numpy as np
import argparse
from actionlib_msgs.msg import GoalStatus


class GreedyCommander(TaskCommander):

    def __init__(self):
        super().__init__()
        self._graph_name = rospy.get_param("graph_name")

    def pixels_to_m(self, pixels):
        return [pixels[0]*0.1, pixels[1]*0.1]

    def execute(self):
        # Set up
        nodes = np.load(self.maps_path+"/outputs/willow-full_lean.npy")

        # Create robots
        print(nodes[0])
        robot0 = Robot(0, nodes[0], 0)
        robot1 = Robot(1, nodes[0], 0)
        robot2 = Robot(2, nodes[0], 0)
        robots = [robot0, robot1, robot2]
        # Create environment
        adj = np.load(
            self.package_path+'/src/robosar_task_allocator/{}.npy'.format(self._graph_name))
        env = Environment(nodes[:len(adj), :], adj, robots)

        solver = TA_greedy()
        solver.init(env)

        # Create transmitter object
        transmitter = TaskTxMoveBase(robots)

        rate = rospy.Rate(10)  # 10hz
        rospy.loginfo(
            'Task_Alloc_Greedy] Buckle up! Running greedy allocator!')

        while not rospy.is_shutdown():

            for robot in robots:
                status = transmitter.getStatus(robot.id)
                if (status == GoalStatus.SUCCEEDED):
                    solver.reached(robot.id, robot.next)
                    transmitter.setGoal(
                        robot.id, self.pixels_to_m(env.nodes[robot.next]))
                    print(env.visited)
                elif (status == GoalStatus.LOST):
                    solver.assign(robot.id, robot.prev)
                    transmitter.setGoal(
                        robot.id, self.pixels_to_m(env.nodes[robot.next]))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('task_commander', anonymous=False, log_level=rospy.DEBUG)

    try:
        tc = GreedyCommander()
        tc.execute()
    except rospy.ROSInterruptException:
        pass
