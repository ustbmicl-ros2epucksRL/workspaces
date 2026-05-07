// Created by Indraneel on 3/11/22

#include <ros/ros.h>
#include "task_graph.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_task_graph_node");
  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  //move_base::MoveBase move_base( buffer );
  
  TaskGraph task_graph_session;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}