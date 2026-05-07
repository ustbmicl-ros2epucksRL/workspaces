#!/usr/bin/env python3

from copy import copy

import actionlib
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from robosar_messages.msg import *
from robosar_messages.srv import *
from sklearn.cluster import MeanShift
from visualization_msgs.msg import Marker

import functions
from functions import gridValue, informationGain


class FrontierFilter:
    def __init__(self) -> None:
        self.mapData = OccupancyGrid()
        self.filtered_frontiers = []

        # fetching all parameters
        ns = rospy.get_name()
        rospy.loginfo(f"Current node namespace: {ns}")
        self.occ_threshold = rospy.get_param("~costmap_clearing_threshold", 70)
        self.info_threshold = rospy.get_param("~info_gain_threshold", 0.2)
        self.pose_graph_proximity_threshold = rospy.get_param("~pose_graph_proximity_threshold", 0.3)
        self.cluster_bandwidth = rospy.get_param("~cluster_bandwidth", 1.0)
        # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
        self.info_radius = rospy.get_param("~info_radius", 0.5)
        self.goals_topic = rospy.get_param("~goals_topic", "/detected_points")
        self.geofence = rospy.get_param("~geofence", [-0.5, 12.0, -10.0, 2.0])  # x_min, x_max, y_min, y_max
        
        self.filter_as = actionlib.SimpleActionServer(
            "frontier_filter_srv",
            FrontierFilterAction,
            execute_cb=self.frontier_srv_callback,
            auto_start=False,
        )
        self.filter_as.start()
        self.frontier_marker_pub = rospy.Publisher(
            ns + "/frontier_centroids", Marker, queue_size=10
        )
        self.frontier_array_pub = rospy.Publisher(
            ns + "/filtered_frontiers", PointArray, queue_size=10
        )
        rospy.loginfo("Frontier filter is ready.")
        rospy.spin()

    def frontier_srv_callback(self, req):
        rospy.loginfo("frontier_srv_callback is run.")
        frontiers = []
        for p in req.frontiers:
            x = np.array([p.x, p.y])
            frontiers.append(x)
        frontiers = np.array(frontiers)

        self.mapData = req.map_data
        self.pose_graph = req.pose_graph

        self.filter(frontiers)
        self.filter_as.set_succeeded(FrontierFilterActionResult())
        

    def init_markers(self):
        points_clust = Marker()
        points_clust.header.frame_id = self.mapData.header.frame_id
        points_clust.header.stamp = rospy.Time.now()
        points_clust.ns = "cluster_markers"
        points_clust.id = 4

        points_clust.type = Marker.POINTS
        # Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points_clust.action = Marker.ADD
        points_clust.pose.orientation.w = 1.0
        points_clust.scale.x = 0.2
        points_clust.scale.y = 0.2
        points_clust.color.r = 0.0 / 255.0
        points_clust.color.g = 255.0 / 255.0
        points_clust.color.b = 0.0 / 255.0
        points_clust.color.a = 1
        points_clust.lifetime = rospy.Duration()

        return points_clust

    def in_geofence(self, node):
        if (
            node[0] > self.geofence[0]
            and node[0] < self.geofence[1]
            and node[1] > self.geofence[2]
            and node[1] < self.geofence[3]
        ):
            return True
        return False

    def check_edge_collision(self, xnear, xnew):
        rez = float(self.mapData.info.resolution)
        stepz = int(np.ceil(functions.Norm(xnew[0], xnew[1], xnear[0], xnear[1])) / rez)
        xi = xnear
        obs = 0
        unk = 0

        for _ in range(stepz):
            xi = functions.Steer(xi, xnew, rez)
            if functions.unvalid(self.mapData, xi):
                obs = 1
                break
            if gridValue(self.mapData, xi) >= self.occ_threshold:
                obs = 1
            if gridValue(self.mapData, xi) == -1:
                unk = 1
                break
        out = 0
        xnew = xi
        if unk == 1:
            out = -1  # unknown
        if obs == 1:
            out = 0  # occupied
        if obs != 1 and unk != 1:
            out = 1  # free

        return out

    def try_rrt_connect_client(self, point):
        rospy.wait_for_service("/robosar_task_generator/rrt_connect")
        try:
            rrt_connect_service = rospy.ServiceProxy(
                "/robosar_task_generator/rrt_connect", rrt_connect
            )
            point_msg = Point()
            point_msg.x = point[0]
            point_msg.y = point[1]
            resp = rrt_connect_service(point_msg)
            return resp.free
        except rospy.ServiceException as e:
            print("task graph getter service call failed: %s" % e)

    def filter(self, received_frontiers):
        centroid_markers = self.init_markers()

        centroid_point = PointStamped()
        centroid_point.header.frame_id = self.mapData.header.frame_id
        centroid_point.header.stamp = rospy.Time(0)
        centroid_point.point.z = 0.0

        # rospy.loginfo("Starting filter")
        centroids = []
        possible_frontiers = []
        # Add received frontiers
        for f in received_frontiers:
            possible_frontiers.append(f)

        # Filter out previous centroids by information gain
        for f in self.filtered_frontiers:
            info_gain = informationGain(
                self.mapData, [f[0], f[1]], self.info_radius, self.occ_threshold
            )
            if info_gain > self.info_threshold: #  and self.try_rrt_connect_client(f)
                possible_frontiers.append(f)

        # Clustering frontier points
        if len(possible_frontiers) > 1:
            ms = MeanShift(bandwidth=self.cluster_bandwidth)
            ms.fit(possible_frontiers)
            centroids = (
                ms.cluster_centers_
            )  # centroids array is the centers of each cluster
        if len(possible_frontiers) == 1:
            centroids = possible_frontiers

        # make sure centroid is not occupied, filter out by information gain
        centroids_filtered = []
        infoGain_filtered = []
        for idx, c in enumerate(centroids):
            centroid_point.point.x = c[0]
            centroid_point.point.y = c[1]
            x = np.array([centroid_point.point.x, centroid_point.point.y])
            infoGain = informationGain(
                self.mapData, [x[0], x[1]], self.info_radius, self.occ_threshold
            )
            if (
                gridValue(self.mapData, x) < self.occ_threshold
                and infoGain > self.info_threshold/2.0
                and self.in_geofence(x)
            ):
                centroids_filtered.append(c)
                infoGain_filtered.append(infoGain)
        self.filtered_frontiers = copy(centroids_filtered)

        # Filter out frontiers too close to the pose graph
        final_frontiers = []
        for f in centroids_filtered:
            # get min distance to pose graph
            min_dist = 1000000
            for node in self.pose_graph:
                dist = functions.Norm(f[0], f[1], node.position.x, node.position.y)
                if dist < min_dist:
                    min_dist = dist
            # filter out frontiers too close to the pose graph
            if min_dist > self.pose_graph_proximity_threshold:
                final_frontiers.append(f)
        centroids_filtered = final_frontiers
        
        # publishing
        arraypoints = PointArray()
        arraypoints.points = []
        arraypoints.infoGain = []
        for i, cen in enumerate(centroids_filtered):
            published_point = Point()
            published_point.z = 0.0
            published_point.x = cen[0]
            published_point.y = cen[1]
            arraypoints.points.append(published_point)
            arraypoints.infoGain.append(min(infoGain_filtered[i]/(self.info_threshold/2), 1.0))
        self.frontier_array_pub.publish(arraypoints)
        pp = []
        for q in range(0, len(centroids_filtered)):
            p = Point()
            p.z = 0
            p.x = centroids_filtered[q][0]
            p.y = centroids_filtered[q][1]
            pp.append(p)
        centroid_markers.points = pp
        self.frontier_marker_pub.publish(centroid_markers)


if __name__ == "__main__":

    rospy.init_node("frontier_filter", anonymous=False)
    try:
        rospy.loginfo("Initializing FrontierFilter node...")
        ff = FrontierFilter()
    except rospy.ROSInterruptException:
        pass
