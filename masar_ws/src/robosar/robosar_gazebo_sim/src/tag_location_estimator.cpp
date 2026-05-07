#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

struct tag_coord
{
  float x = 0;
  float y = 0;
  float z = 0;
  bool is_new = false;
};

const std::string fixed_frame = "map";
const uint num_robots = 2;
const std::string robot_prefix = "agent";
const uint num_tags = 14;
const std::string tag_prefix = "tag_";

int main(int argc, char** argv){
  ros::init(argc, argv, "tag_location_estimator");
  ros::NodeHandle nh;
  ros::Publisher pub_mk = nh.advertise<visualization_msgs::Marker>("tag_location", 10);
  tf::TransformListener listener;
  ros::Duration(3.0).sleep();
  ros::Rate rate(10.0);
  
  while (nh.ok()){
    tf::StampedTransform transform;
    bool transform_found = true;
    // Try all possible transform combinations of tags and agents
    for(uint robot_idx = 0; robot_idx < num_robots; robot_idx++){
      for(uint tag_idx = 0; tag_idx < num_tags; tag_idx++){
        std::string cur_robot_name = robot_prefix + std::to_string(robot_idx);
        std::string cur_tag_name = tag_prefix + std::to_string(tag_idx);
        try{
          listener.lookupTransform("/" + fixed_frame, "/" + cur_tag_name + "/" + cur_robot_name, ros::Time(0), transform);
          transform_found = true;
        }
        catch (tf::TransformException ex){
          transform_found = false;
        }
        if(transform_found){
          // Found transform; generate and publish message
          visualization_msgs::Marker m;
          m.header.frame_id = fixed_frame;
          // label
          m.ns = tag_prefix;
          m.id = tag_idx;
          // shape
          m.type = 2; // Sphere
          // what to do
          m.action = 0; // Add/modify
          // pose
          m.pose.position.x = transform.getOrigin().x();
          m.pose.position.y = transform.getOrigin().y();
          m.pose.position.z = transform.getOrigin().z();
          m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0;
          m.pose.orientation.w = 1;
          // size
          m.scale.x = 0.3;
          m.scale.y = 0.3;
          m.scale.z = 0.3;
          // color
          m.color.r = 0.2;
          m.color.g = 0.2;
          m.color.b = 1;
          m.color.a = 1;
          // lifetime
          m.lifetime = ros::Duration(0); // forever
          // publish
          pub_mk.publish(m);
          rate.sleep();
        }
      }
    }
  }
  return 0;
};
