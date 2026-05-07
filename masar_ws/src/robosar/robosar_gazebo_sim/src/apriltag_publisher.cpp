#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

int main(int argc, char** argv){
  // ROS setup
  ros::init(argc, argv, "apriltag_publisher_gazebo");
  ros::NodeHandle nh("~");
  ros::Duration(3.0).sleep(); // Delay startup for simulator
  ros::Rate rate(10.0);
  // Get params
  // Agent info
  std::vector<std::string> agent_names;
  if(!nh.getParam("agent_names", agent_names))
    ROS_FATAL("agent_names param not defined!");
  std::string base_frame_name;
  if(!nh.getParam("base_frame_name", base_frame_name))
    ROS_FATAL("base_frame_name param not defined!");
  // Tag info
  std::string tag_name;
  if(!nh.getParam("tag_name", tag_name))
    ROS_FATAL("tag_name param not defined!");
  int tag_amount;
  if(!nh.getParam("tag_amount", tag_amount))
    ROS_FATAL("tag_amount param not defined!");
  // Topic info
  std::string topic_name_prefix;
  if(!nh.getParam("topic_name_prefix", topic_name_prefix))
    ROS_FATAL("topic_name_prefix param not defined!");
  std::string topic_name_suffix;
  if(!nh.getParam("topic_name_suffix", topic_name_suffix))
    ROS_FATAL("topic_name_suffix param not defined!");

  // Set up publishers
  std::vector<ros::Publisher> publishers;
  for(size_t agent_idx = 0; agent_idx < agent_names.size(); agent_idx++)
  {
    const std::string topic_name = topic_name_prefix + agent_names.at(agent_idx) + topic_name_suffix;
    publishers.push_back(nh.advertise<apriltag_ros::AprilTagDetectionArray>(topic_name, 10));
  }

  // Set up tf buffer and listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  while (nh.ok()){
    for(size_t agent_idx = 0; agent_idx < agent_names.size(); agent_idx++)
    {
      // Generate new message for each agent
      apriltag_ros::AprilTagDetectionArray msg;
      
      // Iterate through all tag IDs
      for(size_t tag_id = 0; tag_id < tag_amount; tag_id++)
      {
        // Set up names
        const std::string base_frame = agent_names.at(agent_idx) + "/" + base_frame_name;
        const std::string tag_frame = tag_name + std::to_string(tag_id) + "/" + agent_names.at(agent_idx);
        
        // Try to get transform
        geometry_msgs::TransformStamped transformStamped;
        bool transform_found = false;
        try{
          transformStamped = tfBuffer.lookupTransform(base_frame, tag_frame, ros::Time(0));
          double secs = (ros::Time::now()-transformStamped.header.stamp).toSec();
          if(secs < 1.0)
            transform_found = true;
        }
        catch (tf2::TransformException &ex) {
          transform_found = false;
        }

        // Publish messasge
        if(transform_found){
          msg.header = transformStamped.header;

          apriltag_ros::AprilTagDetection detection;
          detection.id.push_back(tag_id);
          detection.size.push_back(0);
          detection.pose.pose.pose.position.x = transformStamped.transform.translation.x;
          detection.pose.pose.pose.position.y = transformStamped.transform.translation.y;
          detection.pose.pose.pose.position.z = transformStamped.transform.translation.z;
          msg.detections.push_back(detection);
        }
      }
      // publish if data available
      if(msg.detections.size())
        publishers.at(agent_idx).publish(msg);
    }
    rate.sleep();
  }
  return 0;
};
