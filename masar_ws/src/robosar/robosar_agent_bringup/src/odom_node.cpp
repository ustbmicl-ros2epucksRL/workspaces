#include "odom_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include "std_msgs/Int16.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;
#define EPSILON 10e-5

// Constructor
OdomNode::OdomNode(const std::string robot_id, ros::Publisher &odom_data_pub_euler, ros::Publisher &odom_data_pub_quat):
  robot_id_(robot_id),
  odom_data_pub_euler_(odom_data_pub_euler),
  odom_data_pub_quat_(odom_data_pub_quat)
{
  // Initialize odomNew_
  odomNew_.header.frame_id = robot_id_+"/odom";
  odomNew_.child_frame_id = robot_id_+"/base_link";
  reset();
}

void OdomNode::reset(){
  // Reset odomNew_
  odomNew_.pose.pose.position.z = 0;
  odomNew_.pose.pose.orientation.x = 0;
  odomNew_.pose.pose.orientation.y = 0;
  odomNew_.twist.twist.linear.x = 0;
  odomNew_.twist.twist.linear.y = 0;
  odomNew_.twist.twist.linear.z = 0;
  odomNew_.twist.twist.angular.x = 0;
  odomNew_.twist.twist.angular.y = 0;
  odomNew_.twist.twist.angular.z = 0;
  // Reset odomOld_
  odomOld_.pose.pose.position.x = 0;
  odomOld_.pose.pose.position.y = 0;
  odomOld_.pose.pose.orientation.z = 0;
}
// Update encoder values
void OdomNode::update_encoders(int enc_left, int enc_right){
  pos_left_ = enc_left;
  pos_right_ = enc_right;
}

// Update odomNew, odomOld
void OdomNode::update_odom()
{

  long delta_pos_left, delta_pos_right;
  float delta_left, delta_right, delta_theta, theta2;
  float delta_x, delta_y;

  // Change in encoder values
  delta_pos_left = pos_left_ - pos_left_prev_;
  delta_pos_right = pos_right_ - pos_right_prev_;
  // Unit conversion
  delta_left = delta_pos_left * wheel_conversion_left;
  delta_right = delta_pos_right * wheel_conversion_right;
  // Change in pose
  delta_theta = (delta_right - delta_left) / wheel_distance;
  theta2 = odomNew_.pose.pose.orientation.z + delta_theta * 0.5;
  delta_x = (delta_left + delta_right) * 0.5 * cosf(theta2);
  delta_y = (delta_left + delta_right) * 0.5 * sinf(theta2);

  // Calculate the new pose (x, y, and theta)
  odomNew_.pose.pose.position.x = odomOld_.pose.pose.position.x + delta_x;
  odomNew_.pose.pose.position.y = odomOld_.pose.pose.position.y + delta_y;
  odomNew_.pose.pose.orientation.z = odomOld_.pose.pose.orientation.z + delta_theta;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew_.pose.pose.position.x) || isnan(odomNew_.pose.pose.position.y) || isnan(odomNew_.pose.pose.position.z))
  {
    odomNew_.pose.pose.position.x = odomOld_.pose.pose.position.x;
    odomNew_.pose.pose.position.y = odomOld_.pose.pose.position.y;
    odomNew_.pose.pose.orientation.z = odomOld_.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew_.pose.pose.orientation.z + EPSILON > PI)
  {
    odomNew_.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew_.pose.pose.orientation.z < -PI + EPSILON)
  {
    odomNew_.pose.pose.orientation.z += 2 * PI;
  }

  // Compute the velocity
  odomNew_.header.stamp = ros::Time::now();
  odomNew_.twist.twist.linear.x = 0.5 * (delta_left + delta_right) / (odomNew_.header.stamp.toSec() - odomOld_.header.stamp.toSec());
  odomNew_.twist.twist.angular.z = delta_theta / (odomNew_.header.stamp.toSec() - odomOld_.header.stamp.toSec());

  // Save the pose data for the next cycle
  odomOld_.pose.pose.position.x = odomNew_.pose.pose.position.x;
  odomOld_.pose.pose.position.y = odomNew_.pose.pose.position.y;
  odomOld_.pose.pose.orientation.z = odomNew_.pose.pose.orientation.z;
  odomOld_.header.stamp = odomNew_.header.stamp;

  // Update encoder values
  pos_left_prev_ = pos_left_;
  pos_right_prev_ = pos_right_;
}

// Publish the odometry message, euler
void OdomNode::publish_euler(){
  odom_data_pub_euler_.publish(odomNew_);
}

// Publish the odometry message, quat
void OdomNode::publish_quat()
{

  tf2::Quaternion q;

  q.setRPY(0, 0, odomNew_.pose.pose.orientation.z);

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew_.header.stamp;
  quatOdom.header.frame_id = robot_id_+"/odom";
  quatOdom.child_frame_id = robot_id_+"/base_link";
  quatOdom.pose.pose.position.x = odomNew_.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew_.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew_.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew_.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew_.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew_.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew_.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew_.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew_.twist.twist.angular.z;

  for (int i = 0; i < SizeOfCovarianceMatrix; i++)
  {
    if (i == 0 || i == 7 || i == 14)
    {
      // diagonal elements
      quatOdom.pose.covariance[i] = .01;
    }
    else if (i == 21 || i == 28 || i == 35)
    {
      // roll pitch and yaw
      quatOdom.pose.covariance[i] += 0.1;
    }
    else
    {
      quatOdom.pose.covariance[i] = 0;
    }
  }

  odom_data_pub_quat_.publish(quatOdom);
}

nav_msgs::Odometry OdomNode::get_odom(){
  return odomNew_;
}

