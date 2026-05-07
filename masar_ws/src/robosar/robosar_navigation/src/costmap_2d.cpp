// Created by Indraneel on 7/03/21
// 2D 代价图：订阅地图、膨胀障碍物、发布供规划使用的 costmap

#include "costmap_2d.hpp"
#include <ros/console.h>

char *Costmap2D::cost_translation_table_ = NULL;

Costmap2D::Costmap2D() : nh_(""), size_width(0), size_height(0), track_unknown_space(true),
                         trinary_costmap(true), costmap_(NULL),
                         inflation_radius(3),      // 膨胀半径(m)：障碍物周围多远的格子参与膨胀
                         inscribed_radius(0.05),     // 内切圆半径(m)：机器人几何模型
                         inflation_cost_scaling_factor(5.0)  // 膨胀代价随距离的衰减系数
{

  if (cost_translation_table_ == NULL)
  {
    create_translation_table();
  }

  // Keep the latest costmap available for late subscribers such as task_graph.
  costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("robosar_navigation/costmap", 1, true);

  ROS_INFO("Requesting the map...");
  map_sub_ = nh_.subscribe("map", 1, &Costmap2D::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while (!map_received_ && nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Map Received!");
}

Costmap2D::~Costmap2D()
{

  // Do we need a lock here?
  delete[] costmap_;
  costmap_ = NULL;

  delete[] cost_translation_table_;
  cost_translation_table_ = NULL;
}

// 内部代价 0-255 转成 OccupancyGrid 的 -1~100
void Costmap2D::create_translation_table()
{
  cost_translation_table_ = new char[256];

  cost_translation_table_[0] = 0;     // 空闲
  cost_translation_table_[253] = 99;  // 膨胀区内（inscribed）
  cost_translation_table_[254] = 100; // 致命障碍
  cost_translation_table_[255] = -1;  // 未知

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++)
  {
    cost_translation_table_[i] = char(1 + (97 * (i - 1)) / 251);
  }
}

// 收到新地图：解析栅格、膨胀障碍、发布 costmap
void Costmap2D::incomingMap(const nav_msgs::OccupancyGridConstPtr &new_map)
{
  std::lock_guard<std::mutex> guard(map_update_mtx_);
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  //ROS_INFO("Received a %d X %d map at %f m/pix with origin %f %f", size_x, size_y,
  //         new_map->info.resolution, new_map->info.origin.position.x, new_map->info.origin.position.y);

  // Recreate map if physical dimensions change
  if (size_width != size_x || size_height != size_y)
  {
    // Clearing previous allocation
    if (costmap_ != NULL)
    {
      delete[] costmap_;
      costmap_ = NULL;
    }
    costmap_ = new unsigned char[size_x * size_y];

    // Update all parameters
    map_frame_ = new_map->header.frame_id;

    origin_y = new_map->info.origin.position.y;
    origin_x = new_map->info.origin.position.x;
    resolution = new_map->info.resolution;
    size_width = size_x;
    size_height = size_y;

    inflator_.initialiseInflator(size_width, size_height, cellDistance(inflation_radius), resolution, inscribed_radius, inflation_cost_scaling_factor);
  }

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  // inflate near obstacles
  inflator_.inflateCosts(costmap_);

  map_received_ = true;
  has_updated_data_ = true;

  // publish inflated costmap
  // ROS_INFO("Preparing grid.");
  prepareGrid();
  // ROS_INFO("Publishing inflated costmap.");
  costmap_pub_.publish(grid_);
}

// 将 OccupancyGrid 单格取值转为内部代价（未知/空闲/障碍/渐变）
unsigned char Costmap2D::interpretValue(unsigned char value)
{
  unsigned char unknown_cost_value_ = -1;
  unsigned char lethal_threshold_ = 100;

  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap)
    return FREE_SPACE;

  double scale = (double)value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

// 新订阅者时单次发布（当前由 incomingMap 持续发布，此函数未用）
void Costmap2D::onNewSubscription(const ros::SingleSubscriberPublisher &pub)
{
  prepareGrid();
  pub.publish(grid_);
}

// 用转换表把内部 costmap 填到 grid_，供发布
void Costmap2D::prepareGrid()
{
  grid_.header.frame_id = map_frame_;
  grid_.header.stamp = ros::Time::now();
  grid_.info.resolution = resolution;

  grid_.info.width = size_width;
  grid_.info.height = size_height;

  grid_.info.origin.position.x = origin_x;
  grid_.info.origin.position.y = origin_y;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;

  grid_.data.resize(grid_.info.width * grid_.info.height);

  unsigned char *data = costmap_;
  for (unsigned int i = 0; i < grid_.data.size(); i++)
  {
    grid_.data[i] = cost_translation_table_[data[i]];
  }
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
  double cells_dist = std::max(0.0, ceil(world_dist / resolution));
  return (unsigned int)cells_dist;
}