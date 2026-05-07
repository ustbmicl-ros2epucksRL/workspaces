/*
 * snap_utils
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX_POSE_UTILS_H_
#define SLAM_TOOLBOX_POSE_UTILS_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "slam_toolbox/toolbox_types.hpp"
#include "karto_sdk/Mapper.h"

namespace pose_utils
{

// helper to get the robots position
class GetPoseHelper
{
public:
  GetPoseHelper(tf2_ros::Buffer* tf,
    const std::string& base_frame,
    const std::string& odom_frame)
  : tf_(tf), base_frame_(base_frame), odom_frame_(odom_frame)
  {
  };

  bool getOdomPose(karto::Pose2& karto_pose,
    const ros::Time& t,
    std::string ref_frame = std::string(""))//
  {
    auto normalizeFrame = [](const std::string& frame) -> std::string
    {
      if (!frame.empty() && frame.front() == '/')
      {
        return frame.substr(1);
      }
      return frame;
    };

    const std::string normalized_ref_frame = normalizeFrame(ref_frame);
    const std::string normalized_base_frame = normalizeFrame(base_frame_);

    // Only succeed if ref_frame matches base_frame_ since using wrong pose helper.
    if (!normalized_ref_frame.empty() &&
        normalized_ref_frame != normalized_base_frame)
    {
      return false;
    }

    geometry_msgs::TransformStamped odom_pose;

    try
    {
      odom_pose = tf_->lookupTransform(
        odom_frame_, base_frame_, t, ros::Duration(0.1));
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
      return false;
    }


    const double yaw = tf2::getYaw(odom_pose.transform.rotation);
    karto_pose = karto::Pose2(odom_pose.transform.translation.x,
      odom_pose.transform.translation.y, yaw);

    return true;
  };

private:
  tf2_ros::Buffer* tf_;
  std::string base_frame_, odom_frame_;
};

} // end namespace

#endif //SLAM_TOOLBOX_POSE_UTILS_H_
