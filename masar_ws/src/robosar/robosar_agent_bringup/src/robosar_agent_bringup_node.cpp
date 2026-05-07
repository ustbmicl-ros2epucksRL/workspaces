// Created by Indraneel on 22/01/21

#include <ros/ros.h>
#include "config_parser.hpp"

int main(int argc, char **argv)
{
  ROS_INFO("333");
  ros::init(argc, argv, "robosar_agent_bringup_node");

  //move_base::MoveBase move_base( buffer );

  ConfigParser new_agent_bringup_session;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}