#!/usr/bin/env python3

# Created by Indraneel on 01/03/22

from cgi import test
import rospy
import actionlib

# from robosar_controller import RobosarControllerAction, RobosarControllerGoal

# from robosar_controller.srv import *
from robosar_controller.msg import *
from task_allocator.Robot import Robot

import time
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from robosar_messages.msg import *


class TaskListenerRobosarControl:
    def __init__(self, robots):
        # rospy.init_node('task_allocator_tx')
        time.sleep(1)
        self.status_map = {}
        # Create status subscribers for each robot
        for i in robots:

            print(i)
            # client = actionlib.SimpleActionClient(robots[i].name,RobosarControllerAction)

            status_keeper = self.ControllerStatusKeeper(i)

            self.status_map[i] = status_keeper

    def add_robot_listener(self, robot_id):
        status_keeper = self.ControllerStatusKeeper(robot_id)
        self.status_map[robot_id] = status_keeper

    def setBusyStatus(self, robot_id):
        if robot_id in self.status_map:
            print("Set busy for {}".format(robot_id))
            self.status_map[robot_id].setBusyStatus()
        else:
            rospy.logwarn(
                "[Task_Alloc_Tx] GetStat Missing client for {}".format(robot_id)
            )

    def getStatus(self, robot_id):
        if robot_id in self.status_map:
            return self.status_map[robot_id].getStatus()
        else:
            rospy.logwarn(
                "[Task_Alloc_Tx] GetStat Missing client for {}".format(robot_id)
            )

    def getGoalID(self, robot_id):
        if robot_id in self.status_map:
            return self.status_map[robot_id].getGoalID()
        else:
            rospy.logwarn(
                "[Task_Alloc_Tx] GetStat Missing client for {}".format(robot_id)
            )

    def wait_for_execution(self, robot_id):
        if robot_id in self.client_map:
            return self.client_map[robot_id].wait_for_result()
        else:
            rospy.logwarn("[Task_Alloc_Tx] Missing client for {}".format(robot_id))

    class ControllerStatusKeeper:
        def __init__(self, name):
            print("Creating a controller status keeper for {}".format(name))

            rospy.Subscriber(
                "/lazy_traffic_controller/" + name + "/status",
                controller_status,
                self.callback,
            )

            self.BUSY = 1
            self.IDLE = 2
            self.GOAL_SENT = 3
            self.status = self.IDLE
            self.goal_id = None

        def setBusyStatus(self):
            self.status = self.GOAL_SENT

        def getStatus(self):
            return self.status

        def getGoalID(self):
            return self.goal_id

        def callback(self, data):
            # print(data.data)
            status = data.data
            self.goal_id = data.goal_id
            if status == data.BUSY:
                self.status = self.BUSY
            elif self.status != self.GOAL_SENT and status == data.SUCCEEDED:
                self.status = self.IDLE


if __name__ == "__main__":
    rospy.init_node("task_listener_control", anonymous=True)
    robot0 = Robot("agent_0", [0, 0], 0)
    robots = [robot0]
    testObj = TaskListenerRobosarControl(robots)
    # time.sleep(1)
    # testObj.setGoal(1,[25.0,15.0])
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(testObj.getStatus("agent_0"))
        # pub.publish(hello_str)
        rate.sleep()
    print("End!")
