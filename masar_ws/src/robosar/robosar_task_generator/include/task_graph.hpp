// Created by Indraneel on 22/10/22

#ifndef TASK_GRAPH_H
#define TASK_GRAPH_H

#include <mutex>
#include <string>
#include <queue>
#include <vector>
#include <thread>
#include <condition_variable>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include "robosar_messages/task_graph_getter.h"
#include "robosar_messages/task_graph_setter.h"
#include "robosar_messages/rrt_connect.h"
#include "functions.h"
#include "mtrand.h"
#include "rrt_utils/RRT.h"

// Assumed 
// ->  Should not have more than 10000 nodes per tree else id calculations need to be changed
class TaskGraph
{

public:
    TaskGraph();
    ~TaskGraph();

    class TaskVertex
    {
    public:
        TaskVertex() : pose_(), neighbors_(), rrt_() {}
        TaskVertex(int id, geometry_msgs::Pose pose, float info_gain)
            : id_(id), pose_(pose), neighbors_(), info_updated_(true), new_vertex_(true), rrt_(pose_,info_gain) {}

        std::pair<float, float> steerVertex(int nearest_node_id, std::pair<float, float> x_rand, float eta);
        int id_;
        geometry_msgs::Pose pose_;
        std::vector<int> neighbors_;
        bool info_updated_, new_vertex_;
        RRT rrt_;
    };

private:
    // ROS interface 
    bool taskGraphServiceCallback(robosar_messages::task_graph_getter::Request &req,
                                  robosar_messages::task_graph_getter::Response &res);

    bool taskGraphSetterServiceCallback(robosar_messages::task_graph_setter::Request &req,
                                        robosar_messages::task_graph_setter::Response &res);
    bool rrtConnectServiceCallback(robosar_messages::rrt_connect::Request &req, robosar_messages::rrt_connect::Response &res);

    void callFrontierFilterService();
    void incomingGraph(const visualization_msgs::MarkerArrayConstPtr &new_graph);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    std::tuple<TaskGraph::TaskVertex *, int> findNearestRRTVertex(std::pair<float, float> &x_rand);
    TaskVertex *findNearestPoseVertex(std::pair<float, float> &x_rand);
    void initROSParams(void);
    bool isInsideGeofence(const std::pair<float, float> x_new);

    //bool isValidCoveragePoint(std::pair<float, float> x_new, float info_radius, int id);
    int gridValue(std::pair<float, float> &Xp);

    // Coverage task generator
    void taskGraphUpdater();
    float informationGain(std::pair<float, float> &x);
    void filterCoveragePoints(std::pair<float, float> x_new, float info_radius, int id);
    void intertreeCoverageFiltering(TaskVertex* vertexPtr);

    // RRT functions
    void expandRRT(const ros::TimerEvent &);
    std::pair<float, float> pixelsToMap(int x_pixel, int y_pixel);
    char ObstacleFree(std::pair<float, float> &xnear, std::pair<float, float> &xnew);
    void pruneRRT(RRT &rrt);
    void propagateEnable(RRT &rrt);

    // visualisation
    void initMarkers(void);
    void visualizeMarkers(void);
    void visualizeTree(void);

    visualization_msgs::Marker marker_points, marker_line, marker_coverage_area, marker_points_coverage;
    visualization_msgs::MarkerArray marker_coverage_area_array;
    ros::NodeHandle nh_;
    ros::Subscriber graph_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_coverage_area_pub_;
    ros::Publisher marker_pub_;
    nav_msgs::OccupancyGrid mapData_;
    ros::ServiceServer task_graph_service_;
    ros::ServiceServer task_setter_service_;
    ros::ServiceServer rrt_connect_service_;
    std_msgs::ColorRGBA color_coverage_, color_allocated_, color_visited_, color_frontier_;

    std::vector<float> geofence_vec_;
    std::vector<float> mc_geofence_vec_;
    std::map<int, int> id_to_index_;
    std::vector<TaskVertex> V_;
    std::vector<geometry_msgs::Point> frontiers_;
    std::mutex mtx;
    bool new_data_rcvd_;
    std::thread node_thread_;
    std::condition_variable cv_;
    std::string frame_id_;
    int filter_threshold_, occ_threshold_;
    float eta_;
    std::string map_topic_;

    // RRT related variables
    double rrt_expansion_period_s_;
    ros::Timer rrt_expansion_timer_;
    MTRand drand; // double in [0, 1) generator, already init
    int prune_counter_;
};

#endif // TASK_GRAPH_H