#!/usr/bin/env python3

"""
Synthetic AprilTag/victim simulator for search-and-rescue tasks.

This node is used to emulate fixed victim locations and publish synthetic
AprilTag detections plus visualization markers. It is not an odometry source.
"""

import actionlib
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from robosar_messages.msg import *
from robosar_messages.srv import *
import sched
import time
from visualization_msgs.msg import Marker
s = sched.scheduler(time.time, time.sleep)
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection


class AprilTagSim:

    def __init__(self):
        rospy.init_node("generate_april_tags_node")
        self.num_april_tags = 12
        self.listener = tf.TransformListener()
        self.distance_threshold = 1.5
        self.rate = rospy.Rate(0.5)
        self.detected_april_tags = []
        self.victim_pub = rospy.Publisher(
            "/slam_toolbox/victim_markers", Marker, queue_size=10)
        self.apriltag_feedback_pub = {}

    def get_map_info(self):
        """
        Gets map message
        """
        rospy.logdebug("Waiting for map")
        map_msg = rospy.wait_for_message("/sim_map", OccupancyGrid)
        rospy.logdebug("Map received")
        scale = map_msg.info.resolution
        origin = [map_msg.info.origin.position.x,
                  map_msg.info.origin.position.y]
        rospy.logdebug("map origin: {}".format(origin))
        data = np.reshape(
            map_msg.data, (map_msg.info.height, map_msg.info.width))
        free_space = 0
        for cell in map_msg.data:
            if cell >= 0:
                free_space += 1
        area = free_space * scale * scale
        rospy.logdebug("Map Area: {}".format(area))
        print("Map received")
        return map_msg, data, scale, origin, area

    def generate_random_poses(self):
        self.map_msg, self.map_data, self.scale, self.origin, _ = self.get_map_info()
        self.april_tag_poses = [[3,4.6], [3.7,8.75], [5.75,5.13], [6.3,15.5], [7.14,11.4], [8.6,16.6], 
                [9.8,9.5],[10.9,6.3], [12.1,15.6], [14.2,12.3], [14.7,8.9], [14.8,15.8]]
        # i = 0
        # while (i < self.num_april_tags):
        #     x = np.random.randint(self.origin[0], self.map_msg.info.width)
        #     y = np.random.randint(self.origin[1], self.map_msg.info.height)
        #     if self.map_data[y, x] == 0:
        #         self.april_tag_poses.append([x*self.scale, y*self.scale])
        #         i += 1
        print("The random generated poses are: ", self.april_tag_poses)

    def publish_ground_truth_victims(self, tag_id, pose):
        m = Marker()
        m.header.frame_id = "map"
        m.pose.position.x = pose[0]
        m.pose.position.y = pose[1]
        m.id = tag_id + 100
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 0.2
        m.color.g = 0
        m.color.b = 1
        m.color.a = 1
        self.victim_pub.publish(m)

    def get_active_agents(self):
        # Get active agents
        self.agent_active_status = {}
        print("calling agent status service")
        rospy.wait_for_service("/robosar_agent_bringup_node/agent_status")
        try:
            get_status = rospy.ServiceProxy(
                "/robosar_agent_bringup_node/agent_status", agent_status
            )
            resp1 = get_status()
            active_agents = resp1.agents_active
            for a in active_agents:
                self.agent_active_status[a] = True
                # Initiate feedback publisher
                self.apriltag_feedback_pub[a] = rospy.Publisher(
                    "/robosar_agent_bringup_node/" + str(a) + "/feedback/apriltag", AprilTagDetectionArray, queue_size=10)

            print("{} agents active".format(len(self.agent_active_status)))
            assert len(self.agent_active_status) > 0
        except rospy.ServiceException as e:
            print("Agent status service call failed: %s" % e)
            raise Exception("Agent status service call failed")
    
    #换gazebo后，会出现rospy.Time.now() 获取的时间与 TF 缓存中的实际时间有偏差，stage没出现上诉问题。
    """
    def get_agent_position(self):
        robot_pos = []

        self.listener.waitForTransform(
            "map",
            list(self.agent_active_status.keys())[0] + "/base_link",
            rospy.Time(),
            rospy.Duration(5.0),
        )
        for name in self.agent_active_status:
            now = rospy.Time.now()
            self.listener.waitForTransform(
                "map", name + "/base_link", now, rospy.Duration(5.0)
            )
            (trans, rot) = self.listener.lookupTransform(
                "map", name + "/base_link", now)
            robot_pos.append([trans[0], trans[1]])
        return robot_pos
    """
    def get_agent_position(self):
        robot_pos = {}
        for name in self.agent_active_status:
            try:
                self.listener.waitForTransform(
                    "map", name + "/base_link", rospy.Time(0), rospy.Duration(5.0)
                )
                (trans, rot) = self.listener.lookupTransform(
                    "map", name + "/base_link", rospy.Time(0))
                yaw = tf.transformations.euler_from_quaternion(rot)[2]
                robot_pos[name] = {
                    "position": [trans[0], trans[1]],
                    "yaw": yaw,
                }
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"[TF警告] 获取 {name} 的位置失败：{e}")
                continue
        return robot_pos

    def compute_relative_detection_pose(self, robot_pose, tag_pose):
        dx = tag_pose[0] - robot_pose["position"][0]
        dy = tag_pose[1] - robot_pose["position"][1]
        yaw = robot_pose["yaw"]
        rel_x = np.cos(yaw) * dx + np.sin(yaw) * dy
        rel_y = -np.sin(yaw) * dx + np.cos(yaw) * dy
        return [rel_x, rel_y]

    def publish_victims(self, tag_pose, detection_pose, agent_name, tag_id):
        print("April Tag id {} detected by {}".format(tag_id, agent_name))
        m = Marker()
        m.header.frame_id = "map"
        m.pose.position.x = tag_pose[0]
        m.pose.position.y = tag_pose[1]
        m.ns = agent_name
        m.id = tag_id
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 1
        m.color.g = 0
        m.color.b = 1
        m.color.a = 1
        self.victim_pub.publish(m)

        detection_array = AprilTagDetectionArray()
        detection_array.header.frame_id = agent_name + "/base_link"
        detection_array.header.stamp = rospy.Time.now()
        detection = AprilTagDetection()
        detection.id = [tag_id]
        detection.pose.pose.pose.position.x = detection_pose[0]
        detection.pose.pose.pose.position.y = detection_pose[1]
        detection.pose.pose.pose.position.z = 0
        detection.pose.pose.pose.orientation.x = 0
        detection.pose.pose.pose.orientation.y = 0
        detection.pose.pose.pose.orientation.z = 0
        detection.pose.pose.pose.orientation.w = 1
        detection_array.detections.append(detection)
        self.apriltag_feedback_pub[agent_name].publish(detection_array)

    def check_agent_poses(self):
        robot_poses = self.get_agent_position()
        #Comparing distance all the robot poses with all april tag poses
        for agent_name, robot_pose in robot_poses.items():
            for j in range(len(self.april_tag_poses)):
                dist = np.linalg.norm(
                    np.array(robot_pose["position"]) - np.array(self.april_tag_poses[j])
                )
                if dist < self.distance_threshold:
                    #check if the april tag is already detected
                    if j not in self.detected_april_tags:
                        self.detected_april_tags.append(j)
                        detection_pose = self.compute_relative_detection_pose(
                            robot_pose,
                            self.april_tag_poses[j],
                        )
                        self.publish_victims(
                            self.april_tag_poses[j],
                            detection_pose,
                            agent_name,
                            j,
                        )

    def simulate_aptil_tag(self):
        self.generate_random_poses()
        self.get_active_agents()

        for tag_id, pose in enumerate(self.april_tag_poses):
            self.publish_ground_truth_victims(tag_id, pose)

        while not rospy.is_shutdown():
            for tag_id, pose in enumerate(self.april_tag_poses):
                self.publish_ground_truth_victims(tag_id, pose)
            self.check_agent_poses()
            self.rate.sleep()

# Create new node which subscribes to map, agent poses and publishes to gui
if __name__ == "__main__":
    try:
        apriltag = AprilTagSim()
        apriltag.simulate_aptil_tag()
    except rospy.ROSInterruptException:
        pass
