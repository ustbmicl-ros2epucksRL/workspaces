// Created by Indraneel on 22/10/22
#include <actionlib/client/simple_action_client.h>

#include "robosar_messages/FrontierFilterAction.h"
#include "task_graph.hpp"

// Assumption each tree should not have more than 10000 nodes
#define TO_TASK_ID(vertex_id, node_id) (vertex_id * 10000 + node_id)
#define TO_VERTEX_ID(task_id) (task_id / 10000)
#define TO_NODE_ID(task_id) (task_id % 10000)
#define MIN_INFO_GAIN 0.05
#define INFO_GAIN_DIST_THRESHOLD 2.0

TaskGraph::TaskGraph() : nh_(""), new_data_rcvd_(false), frame_id_("map"), prune_counter_(0)
{
  // TODO
  initROSParams();

  map_sub_ = nh_.subscribe(map_topic_, 100, &TaskGraph::mapCallBack, this);

  // Wait for map topic
  ROS_INFO("Waiting for map topic %s!", map_topic_.c_str());
  while (mapData_.data.size() < 1)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("recevied map");

  graph_sub_ = nh_.subscribe("/slam_toolbox/karto_graph_visualization", 1, &TaskGraph::incomingGraph, this);
  marker_coverage_area_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/task_graph/coverage_area", 10);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/task_graph/shapes", 10);

  // advertise task graph services
  task_graph_service_ = nh_.advertiseService("/robosar_task_generator/task_graph_getter", &TaskGraph::taskGraphServiceCallback, this);
  task_setter_service_ = nh_.advertiseService("/robosar_task_generator/task_graph_setter", &TaskGraph::taskGraphSetterServiceCallback, this);
  rrt_connect_service_ = nh_.advertiseService("/robosar_task_generator/rrt_connect", &TaskGraph::rrtConnectServiceCallback, this);

  initMarkers();

  ROS_INFO("Task Graphing!");
  node_thread_ = std::thread(&TaskGraph::taskGraphUpdater, this);

  // RRT expansion timer
  rrt_expansion_timer_ = nh_.createTimer(ros::Duration(rrt_expansion_period_s_), boost::bind(&TaskGraph::expandRRT, this, _1));
}

void TaskGraph::initROSParams(void) {
  std::string ns = ros::this_node::getName();
  ros::param::param<int>(ns + "/filter_threshold", filter_threshold_, 20);
  ros::param::param<float>(ns + "/eta", eta_, 1.0);
  ros::param::param<double>(ns + "/sampling_period", rrt_expansion_period_s_, 1.0 / 50);
  ros::param::param<std::string>(ns + "/map_topic", map_topic_, "/map");
  ros::param::param<int>(ns + "/occ_threshold", occ_threshold_, 70);
  ros::param::param<std::vector<float>>(ns + "/geofence", geofence_vec_, {0.5, 0.5});
  ros::param::param<std::vector<float>>(ns + "/mc_geofence", mc_geofence_vec_, {0.5, 0.5});
}

bool TaskGraph::taskGraphServiceCallback(robosar_messages::task_graph_getter::Request &req,
                                         robosar_messages::task_graph_getter::Response &res)
{
  std::lock_guard<std::mutex> guard(mtx);
  // TODO
  // Go through all the vertices and add unvisited coverage nodes to the response
  for (auto &v : V_)
  {
    for (auto& cov_node_id : v.rrt_.coverage_nodes_) {
      auto node_ptr = v.rrt_.get_node(cov_node_id);
      if (!node_ptr->is_visited_ && !node_ptr->is_disabled())
      {
        res.task_ids.push_back(TO_TASK_ID(v.id_,node_ptr->get_id()));
        geometry_msgs::Point p;
        p.x = node_ptr->get_x();
        p.y = node_ptr->get_y();
        p.z = 0.0;
        // calculate free space info gain heuristic
        std::pair<float,float> node_pos = {p.x, p.y};
        TaskVertex* pose_node = findNearestPoseVertex(node_pos);
        float dist = Norm(p.x, p.y, pose_node->pose_.position.x, pose_node->pose_.position.y);
        float info_gain_heur = std::max(std::min(dist/INFO_GAIN_DIST_THRESHOLD, 1.0), MIN_INFO_GAIN);
        res.points.push_back(p);
        res.task_types.push_back(robosar_messages::task_graph_getter::Response::COVERAGE);
        res.info_gains.push_back(info_gain_heur);
        node_ptr->is_allocated_ = true;
      }
    }
  }

  ROS_WARN("Task Graph Service Callback");
  // visualise coverage points
  visualizeMarkers();
  return true;
}

bool TaskGraph::taskGraphSetterServiceCallback(robosar_messages::task_graph_setter::Request &req,
                                               robosar_messages::task_graph_setter::Response &res)
{
  std::lock_guard<std::mutex> guard(mtx);
  // TODO
  // Go through all visited tasks and mark them as visited
  for (const auto &finished_task_ : req.finished_task_ids)
  {
    int vertex_id = TO_VERTEX_ID(finished_task_);
    int node_id = TO_NODE_ID(finished_task_);
    auto node_ptr = V_[id_to_index_[vertex_id]].rrt_.get_node(node_id);
    assert(node_ptr->is_coverage_node_);
    assert(node_ptr->is_allocated_);
    node_ptr->is_visited_ = true;
  }

  ROS_WARN("Task Graph Setter Service Callback");
  // visualise coverage points
  visualizeMarkers();
  return true;
}

bool TaskGraph::rrtConnectServiceCallback(robosar_messages::rrt_connect::Request &req, robosar_messages::rrt_connect::Response &res) {
  std::lock_guard<std::mutex> guard(mtx);
  std::pair<float, float> x = std::make_pair(req.point.x, req.point.y);
  auto output = findNearestRRTVertex(x);
  TaskVertex* tv = std::get<0>(output);
  std::pair<float, float> xnear = tv->rrt_.get_node(std::get<1>(output))->get_coord();
  char checking = ObstacleFree(xnear, x);
  res.free = true;
  if (checking == 0)
    res.free = false;
  return true;
}

void TaskGraph::callFrontierFilterService()
{
  actionlib::SimpleActionClient<robosar_messages::FrontierFilterAction> frontier_filter_client_("frontier_filter_srv", true);
  frontier_filter_client_.waitForServer();
  robosar_messages::FrontierFilterGoal req;
  req.frontiers = frontiers_;
  req.map_data = mapData_;
  // Go through all the vertices and add poses to the request
  for (auto &v : V_)
  {
    req.pose_graph.push_back(v.pose_);
  }

  frontier_filter_client_.sendGoal(req);
}

// Subscribers callback functions---------------------------------------
void TaskGraph::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  ROS_WARN("Received new map update! Seq: %d", msg->header.seq); // 添加这行
  std::lock_guard<std::mutex> guard(mtx);
  mapData_ = *msg;
}

TaskGraph::~TaskGraph()
{
  new_data_rcvd_ = true;
  cv_.notify_one();
  node_thread_.join();
}

inline bool AreSame(double a, double b)
{
  return fabs(a - b) < std::numeric_limits<double>::epsilon();
}

void TaskGraph::initMarkers()
{
  // visualizations  points and lines..
  marker_points.header.frame_id = frame_id_;
  marker_line.header.frame_id = frame_id_;
  marker_points.header.stamp = ros::Time(0);
  marker_line.header.stamp = ros::Time(0);

  marker_points.ns = marker_line.ns = "markers";
  marker_points.id = 0;
  marker_line.id = 1;

  marker_points.type = marker_points.POINTS;
  marker_line.type = marker_line.LINE_LIST;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_points.action = marker_points.ADD;
  marker_line.action = marker_line.ADD;
  marker_points.pose.orientation.w = 1.0;
  marker_line.pose.orientation.w = 1.0;
  marker_line.scale.x = 0.03;
  marker_line.scale.y = 0.03;
  marker_points.scale.x = 0.3;
  marker_points.scale.y = 0.3;

  //marker_line.color.r = 9.0 / 255.0;
  //marker_line.color.g = 91.0 / 255.0;
  //marker_line.color.b = 236.0 / 255.0;
  marker_points.color.a = 0.5;
  marker_line.color.a = 0.5;
  marker_points.lifetime = ros::Duration();
  marker_line.lifetime = ros::Duration();

  // marker for coverage points
  marker_points_coverage = marker_points;
  marker_points_coverage.color.r = 0.0;
  marker_points_coverage.color.g = 255.0 / 255.0;
  marker_points_coverage.color.b = 255.0 / 255.0;

  // marker for coverage area
  marker_coverage_area = marker_points;
  marker_coverage_area.type = marker_coverage_area.CYLINDER;
  marker_coverage_area.color.r = 0.0;
  marker_coverage_area.color.g = 0.0;
  marker_coverage_area.color.b = 255.0 / 255.0;
  marker_coverage_area.color.a = 0.1;
  marker_coverage_area.scale.z = 0.01;

  color_coverage_.r = 0.0;
  color_coverage_.g = 255.0 / 255.0;
  color_coverage_.b = 255.0 / 255.0;
  color_coverage_.a = 1.0;

  color_allocated_.r = 255.0 / 255.0;
  color_allocated_.g = 255.0 / 255.0;
  color_allocated_.b = 0.0 / 0.0;
  color_allocated_.a = 1.0;

  color_visited_.r = 255.0 / 255.0;
  color_visited_.g = 100.0;
  color_visited_.b = 0.0 / 255.0;
  color_visited_.a = 1.0;

  color_frontier_.r = 0.0 / 255.0;
  color_frontier_.g = 0.0;
  color_frontier_.b = 0.0 / 255.0;
  color_frontier_.a = 0.2;
}

void TaskGraph::incomingGraph(const visualization_msgs::MarkerArrayConstPtr &new_graph)
{
  std::lock_guard<std::mutex> guard(mtx);
  bool local_graph_update = false;

  // Go through all the markers in the array and add them to the graph
  for (auto marker : new_graph->markers)
  {
    // Check if the marker is a node
    if (marker.type == visualization_msgs::Marker::SPHERE)
    {
      // Check if the node already exists
      if (id_to_index_.find(marker.id) == id_to_index_.end())
      {
        // Add the node to the graph
        std::pair<float, float> vertex_pos = std::make_pair(marker.pose.position.x, marker.pose.position.y);
        float info_gain = informationGain(vertex_pos);
        TaskVertex new_vertex(marker.id, marker.pose, info_gain);

        // update coverage points
        // new_vertex.rrt_.update_coverage_nodes();

        V_.push_back(new_vertex);
        id_to_index_[marker.id] = V_.size() - 1;
        local_graph_update = true;
      }
      else
      {
        // Check if there is a change in the pose
        TaskVertex graph_vertex = V_[id_to_index_[marker.id]];
        if (!AreSame(graph_vertex.pose_.position.x, marker.pose.position.x) ||
            !AreSame(graph_vertex.pose_.position.y, marker.pose.position.y))
        {
          // Update the pose
          graph_vertex.pose_ = marker.pose;
          graph_vertex.info_updated_ = true;
          V_[id_to_index_[marker.id]] = graph_vertex;
          local_graph_update = true;
        }
      }
    }
    // Check if the marker is an edge
    else if (marker.type == visualization_msgs::Marker::ARROW)
    {
      ROS_WARN("Edge marker found");
    }
  }

  // Inform main thread
  if (local_graph_update)
  {
    ROS_WARN("New size of graph: %ld", V_.size());
    new_data_rcvd_ = true;
    cv_.notify_one();
  }
}

void TaskGraph::taskGraphUpdater()
{

  while (ros::ok())
  {
    std::unique_lock<std::mutex> lk(mtx);
    cv_.wait(lk, [this]
             { return new_data_rcvd_; });

    ROS_WARN("New pose graph received updating task graph");

    /** ==================== Iterate through all the vertices ,update information gain, update RRT ===================== */
    for (auto &vertex : V_)
    {
      if(vertex.new_vertex_) {
        
        vertex.rrt_.update_coverage_nodes();
        vertex.new_vertex_ = false;
      }
      else if (vertex.info_updated_)
      {
        ROS_WARN("Vertex %d has been updated", vertex.id_);

        vertex.rrt_.update_rrt(vertex.pose_);
        // vertex.info_updated_ = false;
        //  TODO if the tree shifts do we need to update the information gain of the vertices in the tree?
      }
    }

    /** ======================= Iterate again and update coverage points ================================ */
    for (auto &vertex : V_)
    {
      if(vertex.info_updated_) {
        
        // Since node to node distances are not updated, the tree is transformed as a whole
        // Intra tree coverage points should remain unaffected so only inter tree is called
        intertreeCoverageFiltering(&vertex);

        vertex.info_updated_ = false;
      }
    }

    // visualise coverage points
    //visualizeMarkers();

    new_data_rcvd_ = false;
    lk.unlock();
  }
}

float Norm(float x1, float y1, float x2, float y2)
{
  return pow((pow((x2 - x1), 2) + pow((y2 - y1), 2)), 0.5);
}

float TaskGraph::informationGain(std::pair<float, float> &x)
{

  // Local variables
  float info_gain_radius_m = 0.0;
  bool end_of_bfs = false;
  // BFS around this point to find distance to nearest obstacle

  // Local variables
  std::queue<std::pair<int, int>> queue;
  std::set<int> visited;

  // Convert to pixel and add it to queue
  std::pair<int, int> start_ind = std::make_pair((int)((x.first - mapData_.info.origin.position.x) / mapData_.info.resolution),
                                                 (int)((x.second - mapData_.info.origin.position.y) / mapData_.info.resolution));
  queue.push(start_ind);

  while (!queue.empty() && !end_of_bfs)
  {
    std::pair<int, int> current = queue.front();
    queue.pop();

    // if already visited, continue to next iteration
    if (visited.find((current.first + current.second * mapData_.info.width)) != visited.end())
      continue;
    // insert into visited
    else
      visited.insert(current.first + current.second * mapData_.info.width);

    // Check if reached end of BFS radius
    // convert to world coordinates
    std::pair<float, float> current_position = std::make_pair(mapData_.info.origin.position.x + mapData_.info.resolution * (float)(current.first),
                                                              mapData_.info.origin.position.y + mapData_.info.resolution * (float)(current.second));

    float dist = Norm(current_position.first, current_position.second,
                      x.first, x.second);
    if (dist > info_gain_radius_m)
    {
      info_gain_radius_m = dist;
    }
    if (dist > COV_MAX_INFO_GAIN_RADIUS_M)
    {
      // ROS_INFO("Outside radius. Exiting obstacle search");
      end_of_bfs = true;
    }

    // Check if obstacle
    if (gridValue(current_position) > 90 || gridValue(current_position) == -1)
    {
      end_of_bfs = true;
    }

    // expand node
    for (const auto &dir : bfs_prop_model)
    {
      std::pair<int, int> neighbour_ind = std::make_pair(current.first + dir[0], current.second + dir[1]);
      // Boundary checks
      if (neighbour_ind.first >= 0 && neighbour_ind.first < mapData_.info.width &&
          neighbour_ind.second >= 0 && neighbour_ind.second < mapData_.info.height)
      {
        queue.push(neighbour_ind);
      }
      else
        end_of_bfs = true;
    }
  }

  return info_gain_radius_m;
}

// gridValue function
int TaskGraph::gridValue(std::pair<float, float> &Xp)
{

  float resolution = mapData_.info.resolution;
  float Xstartx = mapData_.info.origin.position.x;
  float Xstarty = mapData_.info.origin.position.y;

  float width = mapData_.info.width;
  std::vector<signed char> Data = mapData_.data;

  // returns grid value at "Xp" location
  // map data:  100 occupied      -1 unknown       0 free
  float indx = (floor((Xp.second - Xstarty) / resolution) * width) + (floor((Xp.first - Xstartx) / resolution));
  int out;
  assert(int(indx) < Data.size());
  out = Data[int(indx)];
  return out;
}

void TaskGraph::visualizeMarkers(void)
{

  // visualization
  marker_points_coverage.points.clear();
  marker_points_coverage.colors.clear();
  marker_coverage_area_array.markers.clear();

  // Clear old Markers
  visualization_msgs::Marker delete_all;
  delete_all.action = visualization_msgs::Marker::DELETEALL;
  delete_all.id = 0;
  marker_coverage_area_array.markers.push_back(delete_all);
  marker_coverage_area_pub_.publish(marker_coverage_area_array);
  marker_coverage_area_array.markers.clear();

  for (auto j = V_.begin(); j != V_.end(); j++)
  {
    // Get coverage nodes
    std::vector<int> coverage_nodes_ = j->rrt_.coverage_nodes_;
    for (const auto& k : coverage_nodes_)
    {
      geometry_msgs::Point p;
      const auto node_ptr = j->rrt_.get_node(k);
      p.x = node_ptr->get_x();
      p.y = node_ptr->get_y();
      p.z = 0.0;

      marker_points_coverage.points.push_back(p);
      // Create coverage point marker
      if (node_ptr->is_visited_)
      {
        marker_points_coverage.colors.push_back(color_visited_);
      }
      else if (node_ptr->is_allocated_)
      {
        marker_points_coverage.colors.push_back(color_allocated_);
      }
      else
      {
        marker_points_coverage.colors.push_back(color_coverage_);
      }

      // Create area marker
      marker_coverage_area.id += 1;
      marker_coverage_area.pose.position.x = p.x;
      marker_coverage_area.pose.position.y = p.y;
      marker_coverage_area.scale.x = 2 * node_ptr->get_info_gain_radius();
      marker_coverage_area.scale.y = 2 * node_ptr->get_info_gain_radius();
      marker_coverage_area_array.markers.push_back(marker_coverage_area);
    }
    
  }

  if (marker_points_coverage.points.size() > 0)
  {
    marker_pub_.publish(marker_points_coverage);
  }
  marker_coverage_area_pub_.publish(marker_coverage_area_array);
}

void TaskGraph::visualizeTree(void)
{

  // visualization
  marker_line.points.clear();
  marker_line.colors.clear();

  for (auto k = V_.begin(); k != V_.end(); k++)
  {
    for (auto j = k->rrt_.nodes_.begin(); j != k->rrt_.nodes_.end(); j++)
    {
      if (j->second->get_parent() == -1)
        continue;
      geometry_msgs::Point p;
      p.x = j->second->get_x();
      p.y = j->second->get_y();
      p.z = 0.0;
      std_msgs::ColorRGBA color_marker; 
      if(j->second->is_disabled())
      {
        color_marker.r = 0.0 / 255.0;
        color_marker.g = 0.0 / 255.0;
        color_marker.b = 0.0 / 255.0;
        color_marker.a = 0.3;
      }
      else {
        color_marker.r = 0.0 / 255.0;
        color_marker.g = 200.0 / 255.0;
        color_marker.b = 200.0 / 255.0;
        color_marker.a = 0.7;
      }
      marker_line.colors.push_back(color_marker);
      marker_line.points.push_back(p);
      p.x = k->rrt_.get_parent_node(j->second)->get_x();
      p.y = k->rrt_.get_parent_node(j->second)->get_y();
      p.z = 0.0;
      marker_line.colors.push_back(color_marker);
      marker_line.points.push_back(p);
    }
  }

  if (marker_line.points.size() > 0)
  {
    marker_pub_.publish(marker_line);
  }
}

std::pair<float, float> TaskGraph::pixelsToMap(int x_pixel, int y_pixel)
{
  std::pair<float, float> map_coords;
  float scale = mapData_.info.resolution;
  float x_origin = mapData_.info.origin.position.x;
  float y_origin = mapData_.info.origin.position.y;
  map_coords = {x_pixel * scale + x_origin, y_pixel * scale + y_origin};
  return map_coords;
}

bool TaskGraph::isInsideGeofence(const std::pair<float, float> x_new) {

  if(x_new.first > geofence_vec_[0] && x_new.first < geofence_vec_[1] && x_new.second > geofence_vec_[2] && x_new.second < geofence_vec_[3])
  {
    if (mc_geofence_vec_.size() < 4)
      return true;
    if (!(x_new.first > mc_geofence_vec_[0] && x_new.first < mc_geofence_vec_[1] && x_new.second > mc_geofence_vec_[2] && x_new.second < mc_geofence_vec_[3]))
      return true;
  }
  return false;
}

void TaskGraph::expandRRT(const ros::TimerEvent &)
{

  std::lock_guard<std::mutex> guard(mtx);

  // Cannot expand RRT if no pose graph
  if (V_.size() == 0)
  {
    return;
  }

  if (prune_counter_ == 500)
  {
    ROS_INFO("Pruning");
    for (auto &vertex : V_) {
       pruneRRT(vertex.rrt_);
     }
    prune_counter_ = 0;

    visualizeMarkers();
  }

  // Local variables
  float xr, yr;
  std::pair<float, float> x_rand, x_nearest;

  // Sample free
  //ROS_INFO("=== RRT Expansion Start ===");
  int xp_r = drand() * mapData_.info.width;
  int yp_r = drand() * mapData_.info.height;
  //ROS_INFO("Sampling pixel coordinates: [%d, %d, %d, %d]", xp_r, yp_r,mapData_.info.width,mapData_.info.height);
  std::pair<float, float> map_coords = pixelsToMap(xp_r, yp_r);
  xr = map_coords.first + drand() * 0.2;
  yr = map_coords.second + drand() * 0.2;

  x_rand = {xr, yr};
  //ROS_INFO("World coordinates: [%.2f, %.2f]", xr, yr);
  // find nearest taskgraph vertex
  TaskVertex* vertexPtr = findNearestPoseVertex(x_rand);
  int nearest_node_id = vertexPtr->rrt_.nearest(x_rand.first, x_rand.second);
  //ROS_INFO("Found nearest vertex ID: %d", vertexPtr->id_);
  // grow vertex tree
  std::pair<float, float> x_new = vertexPtr->steerVertex(nearest_node_id, x_rand, eta_);

  // Check if connection is valid
  x_nearest = vertexPtr->rrt_.get_node(nearest_node_id)->get_coord();
  int connection_type = ObstacleFree(x_nearest, x_new);
  //ROS_INFO("Connection check result: %d (1=free, 0=obstacle, -1=frontier)", connection_type);
  // if can't connect to nearest taskgraph vertex, try to connect to nearest rrt node
  if (connection_type == 0) {
    auto output = findNearestRRTVertex(x_rand);
    vertexPtr = std::get<0>(output);
    nearest_node_id = std::get<1>(output);
    x_new = vertexPtr->steerVertex(nearest_node_id, x_rand, eta_);
    x_nearest = vertexPtr->rrt_.get_node(nearest_node_id)->get_coord();
    connection_type = ObstacleFree(x_nearest, x_new);
  }

  // frontier detected
  if (connection_type == -1)
  {
    // ROS_WARN("Frontier detected!");
    ROS_WARN("Frontier detected at [%.2f, %.2f]", x_nearest.first, x_nearest.second);
    geometry_msgs::Point p;
    p.x = x_nearest.first;
    p.y = x_nearest.second;
    p.z = 0.0;
    frontiers_.push_back(p);
    marker_points_coverage.points.push_back(p);
    marker_points_coverage.colors.push_back(color_frontier_);
    marker_pub_.publish(marker_points_coverage);
  }
  // valid connection
  else if (connection_type == 1 && isInsideGeofence(x_new))
  {
    //ROS_INFO("Adding new node at [%.2f, %.2f]", x_new.first, x_new.second);
    // Calculate information gain
    float info_gain = informationGain(x_new);
    vertexPtr->rrt_.add_node(x_new.first, x_new.second, nearest_node_id, info_gain);

    // update intra tree coverage nodes
    vertexPtr->rrt_.update_coverage_nodes();

    // filter intra tree coverage nodes
    vertexPtr->rrt_.filter_coverage_nodes();

    intertreeCoverageFiltering(vertexPtr);
  }

  // visualise
  visualizeTree();

  // if enough frontiers, call frontier_filter service
  if (frontiers_.size() > filter_threshold_)
  {
    ROS_INFO("Triggering frontier filter with %d points", (int)frontiers_.size());
    callFrontierFilterService();
    frontiers_.clear();
  }

  prune_counter_++;
}

std::tuple<TaskGraph::TaskVertex *, int> TaskGraph::findNearestRRTVertex(std::pair<float, float> &x_rand)
{
  float min_dist = std::numeric_limits<float>::max();
  TaskVertex *nearest_vertex = NULL;
  int nearest_node_id = -1;
  for (auto j = V_.begin(); j != V_.end(); j++)
  {
    int curr_nearest_id = j->rrt_.nearest(x_rand.first, x_rand.second);
    auto closest_node = j->rrt_.get_node(curr_nearest_id);
    float dist = Norm(closest_node->get_x(), closest_node->get_y(), x_rand.first, x_rand.second);
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_vertex = &(*j);
      nearest_node_id = curr_nearest_id;
    }
  }
  return std::make_tuple(nearest_vertex, nearest_node_id);
}

TaskGraph::TaskVertex *TaskGraph::findNearestPoseVertex(std::pair<float, float> &x_rand)
{
  float min_dist = 1000000;
  TaskVertex *nearest_vertex = NULL;
  for (auto j = V_.begin(); j != V_.end(); j++)
  {
    float dist = Norm(j->pose_.position.x, j->pose_.position.y, x_rand.first, x_rand.second);
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_vertex = &(*j);
    }
  }
  return nearest_vertex;
}

std::pair<float, float> TaskGraph::TaskVertex::steerVertex(int nearest_node_id, std::pair<float, float> x_rand, float eta)
{
  assert(nearest_node_id != -1);
  std::pair<float, float> x_nearest = rrt_.get_node(nearest_node_id)->get_coord();
  std::pair<float, float> x_new = Steer(x_nearest, x_rand, eta);
  return x_new;
}

char TaskGraph::ObstacleFree(std::pair<float, float> &xnear, std::pair<float, float> &xnew)
{
  float rez = float(mapData_.info.resolution);
  int stepz = int(ceil(Norm(xnew.first, xnew.second, xnear.first, xnear.second)) / rez);
  std::pair<float, float> xi = xnear;
  char obs = 0;
  char unk = 0;

  geometry_msgs::Point p;
  for (int c = 0; c < stepz; c++)
  {
    xi = Steer(xi, xnew, rez);

    if (gridValue(xi) > occ_threshold_)
      obs = 1;
    if (gridValue(xi) == -1)
    {
      unk = 1;
      break;
    }
  }
  char out = 0;
  xnew = xi;
  if (unk == 1)
    out = -1; // frontier: reached unknown space
  else if (obs == 1)
    out = 0; // occupied: blocked before reaching unknown
  else
    out = 1; // free

  return out;
}

void TaskGraph::pruneRRT(RRT &rrt)
{
  std::vector<int> to_remove;
  for (auto j = rrt.nodes_.begin(); j != rrt.nodes_.end(); j++)
  {
    if (j->second->get_parent() == -1 || j->second->is_disabled())
      continue;
    std::pair<float, float> x_child = j->second->get_coord();
    std::pair<float, float> x_parent = rrt.get_parent_node(j->second)->get_coord();
    char checking = ObstacleFree(x_parent, x_child);
    if (checking == 0)
    {
      to_remove.push_back(j->first);
    }
  }
  for (int id : to_remove)
  {
    if (id != 0) // never remove root node
      rrt.disable_node(id);
  }

  propagateEnable(rrt);
}

void TaskGraph::propagateEnable(RRT &rrt) {
  std::unordered_set<int> disabled_nodes = rrt.get_disabled_node_ids();
  // nodes to be try and enabled
  std::queue<int> q;
  for (int dr : disabled_nodes) {
    q.push(dr);
  }
  while (!q.empty()) {
    int node_id = q.front();
    q.pop();
    auto node = rrt.get_node(node_id);
    std::pair<float, float> x_child = node->get_coord();
    std::pair<float, float> x_parent = rrt.get_parent_node(node)->get_coord();
    char checking = ObstacleFree(x_parent, x_child);
    // renable node if edge is collision free
    if (checking == 1) {
      rrt.enable_node(node_id);
      // add disabled children to queue
      for (int child_id : node->get_children()) {
        if (rrt.get_node(child_id)->is_disabled())
          q.push(child_id);
      }
    }
  }
}

// returns true if coverage points are redundant
bool intertreeCoverageFilter(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2)
{

  // check overlap
  // TODO should we check if it is a coverage node?
  float inter_node_dist = Norm(node1->get_x(), node1->get_y(),
                               node2->get_x(), node2->get_y());
  // If there is an overlap (either of the centers is inside the other circle)
  if ((node1->get_info_gain_radius() > inter_node_dist || node2->get_info_gain_radius() > inter_node_dist))
  {
    return true;
  }

  // check node to node distance
  if (inter_node_dist < COV_MIN_DIST_BETWEEN_COVERAGE_POINTS)
    return true;

  return false;
}

void TaskGraph::intertreeCoverageFiltering(TaskVertex* vertexPtr) {

  // Check if all coverage nodes in this tree are valid
  // if not remove them
  for (auto my_node_id_ptr = vertexPtr->rrt_.coverage_nodes_.begin();
            my_node_id_ptr != vertexPtr->rrt_.coverage_nodes_.end();) {
    
    bool redundant = false;
    std::vector<int> to_remove_task_ids;
    to_remove_task_ids.clear();
    auto my_node_ptr = vertexPtr->rrt_.get_node(*my_node_id_ptr);

    // dont filter allocated or visited coverage points
    if (my_node_ptr->is_allocated_ || my_node_ptr->is_visited_) {
      my_node_id_ptr++;
      continue;
    }

    for (auto j = V_.begin(); j != V_.end(); j++) {
      // dont compare with same tree
      if (j->id_ == vertexPtr->id_)
        continue;

      for(auto other_node_id_ptr = j->rrt_.coverage_nodes_.begin(); 
                other_node_id_ptr != j->rrt_.coverage_nodes_.end();) {
        auto other_node_ptr = j->rrt_.get_node(*other_node_id_ptr);

        if(intertreeCoverageFilter(my_node_ptr, other_node_ptr)) {
          
          // filter out smaller coverage node
          if (other_node_ptr->is_allocated_ || other_node_ptr->is_visited_ || my_node_ptr->get_info_gain_radius() < other_node_ptr->get_info_gain_radius())
          {
            // ROS_INFO("Removing my coverage node %d from tree %d for cov node %d from tree %d", my_node_ptr->get_id(), vertexPtr->id_,
            //                                                                   other_node_ptr->get_id(), j->id_);  
            redundant = true;
            break;
          }
          else {
            to_remove_task_ids.push_back(TO_TASK_ID(j->id_, other_node_ptr->get_id()));
          }
        }

        other_node_id_ptr++;
      }

      if (redundant)
        break;
    }
    
    // delete if redundant
    if(redundant) {
      my_node_ptr->is_coverage_node_ = false;
      my_node_id_ptr = vertexPtr->rrt_.coverage_nodes_.erase(my_node_id_ptr);
    }
    else {
      // Process to remove task ids
      for(auto& task_id:to_remove_task_ids) {
        int vertex_id = TO_VERTEX_ID(task_id);
        int node_id = TO_NODE_ID(task_id);
        assert(id_to_index_.find(vertex_id) != id_to_index_.end());
        assert(id_to_index_[vertex_id]<V_.size());
        auto& vertex = V_[id_to_index_[vertex_id]];
        assert(std::find(vertex.rrt_.coverage_nodes_.begin(),vertex.rrt_.coverage_nodes_.end(),node_id)  != vertex.rrt_.coverage_nodes_.end());
        vertex.rrt_.coverage_nodes_.erase(std::find(vertex.rrt_.coverage_nodes_.begin(),vertex.rrt_.coverage_nodes_.end(),node_id));
        vertex.rrt_.get_node(node_id)->is_coverage_node_ = false;

        // ROS_INFO("Removing other coverage node %d from tree %d for cov node %d from tree %d", node_id, vertex_id,
        //                                                                       my_node_ptr->get_id(), vertexPtr->id_);
      }
      my_node_id_ptr++;
    }
  }
}
