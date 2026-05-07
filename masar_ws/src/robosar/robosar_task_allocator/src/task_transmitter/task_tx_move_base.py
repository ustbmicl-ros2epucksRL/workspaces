#!/usr/bin/env python3

# Created by Indraneel on 01/03/22

from cgi import test
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

class TaskTxMoveBase:

    def __init__(self, robots):
        #rospy.init_node('task_allocator_tx')
        time.sleep(1)
        self.client_map = {}
        # Create action clients for each robot
        for i in range(len(robots)):
            
            # Create an action client called "move_base" with action definition file "MoveBaseAction"
            client = actionlib.SimpleActionClient(robots[i].name+'/move_base',MoveBaseAction)

            # Waits until the action server has started up and started listening for goals.
            if(client.wait_for_server(rospy.Duration(5))):
                # Add it to client map
                self.client_map[robots[i].name] = client
            else:
                rospy.logwarn("[Task_Alloc_Tx] Could not create client for {}".format(robots[i].name))

    def setGoal(self,robot_id,task):
        if robot_id in self.client_map:
            # Creates a new goal with the MoveBaseGoal constructor
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = task[0]
            goal.target_pose.pose.position.y = task[1]
            # No rotation of the mobile base frame w.r.t. map frame
            goal.target_pose.pose.orientation.w = 1.0

            # Sends the goal to the action server.
            self.client_map[robot_id].send_goal(goal)
            rospy.loginfo('[Task_Alloc_Tx] Goal sent to {}'.format(robot_id))
        else:
            rospy.logwarn("[Task_Alloc_Tx] Missing client for {}".format(robot_id))

    def getStatus(self,robot_id):
        if robot_id in self.client_map:
            return self.client_map[robot_id].get_state()
        else:
            rospy.logwarn("[Task_Alloc_Tx] Missing client for {}".format(robot_id))

    def wait_for_execution(self,robot_id):
        if robot_id in self.client_map:
            return self.client_map[robot_id].wait_for_result()
        else:
            rospy.logwarn("[Task_Alloc_Tx] Missing client for {}".format(robot_id))

# if __name__ == "__main__":
#     robot0 = Robot(0, [0,0], 0)
#     robot1 = Robot(1, [0,0], 0)
#     robot2 = Robot(2, [0,0], 0)
#     robots = [robot0, robot1, robot2]
#     testObj = TaskTxMoveBase(robots)
#     time.sleep(1)
#     testObj.setGoal(1,[25.0,15.0])
#     print('End!')