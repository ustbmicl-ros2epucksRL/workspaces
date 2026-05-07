/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
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

#include "slam_toolbox/slam_toolbox_sync.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
SynchronousSlamToolbox::SynchronousSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  int scan_queue_size_limit = 30;
  double scan_queue_max_delay = 2.0;
  nh.param("sync_queue_size_limit", scan_queue_size_limit, scan_queue_size_limit);
  nh.param("sync_queue_max_delay", scan_queue_max_delay, scan_queue_max_delay);
  scan_queue_size_limit_ = scan_queue_size_limit < 0 ? 0u :
    static_cast<size_t>(scan_queue_size_limit);
  scan_queue_max_delay_ = ros::Duration(std::max(0.0, scan_queue_max_delay));

  ROS_INFO("[Debug] Sync mode initialized");
  ROS_INFO("SynchronousSlamToolbox queue guard: size_limit=%zu max_delay=%.3fs",
    scan_queue_size_limit_, scan_queue_max_delay_.toSec());
  ssClear_ = nh.advertiseService("clear_queue",
    &SynchronousSlamToolbox::clearQueueCallback, this);

  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&SynchronousSlamToolbox::run, this)));

  loadPoseGraphByParams(nh);
}

/*****************************************************************************/
void SynchronousSlamToolbox::run()
/*****************************************************************************/
{
  ros::Rate r(100);
  ROS_INFO("[Debug] Sync mode run");
  while(ros::ok())
  {
    if (!isPaused(PROCESSING))
    {
      //ROS_INFO("[Debug] run");
      PosedScan scan_w_pose(nullptr, karto::Pose2()); // dummy, updated in critical section
      bool queue_empty = true;
      {
        boost::mutex::scoped_lock lock(q_mutex_);
        queue_empty = q_.empty();
        if(!queue_empty)
        {
          scan_w_pose = q_.front();
          q_.pop();

          if (q_.size() > 10)
          {
            ROS_WARN_THROTTLE(10., "Queue size has grown to: %i. "
              "Recommend stopping until message is gone if online mapping.",
              (int)q_.size());
          }
        }
      }
      if(!queue_empty){//
        const ros::Time now = ros::Time::now();
        if (scan_queue_max_delay_ > ros::Duration(0.0) &&
            !scan_w_pose.scan->header.stamp.isZero() &&
            !now.isZero() &&
            now > scan_w_pose.scan->header.stamp &&
            (now - scan_w_pose.scan->header.stamp) > scan_queue_max_delay_)
        {
          ROS_WARN_THROTTLE(5.,
            "Dropping stale queued scan for %s delayed by %.3fs",
            scan_w_pose.scan->header.frame_id.c_str(),
            (now - scan_w_pose.scan->header.stamp).toSec());
          continue;
        }
        // Process scan with pose
        //ROS_INFO("[Debug] scan");
        karto::LocalizedRangeScan* karto_scan = addScan(getLaser(scan_w_pose.scan), scan_w_pose);
        // Tie this agent's pose with this agent's apriltag array detections
        if (karto_scan)
        {
          // Get this agent's name
          std::stringstream ss(scan_w_pose.scan->header.frame_id);
          std::string frame_id;
          std::getline(ss, frame_id, '/');
          boost::mutex::scoped_lock lock(apriltag_q_mutex_);
          // Get this agent's queue
          std::queue<apriltag_ros::AprilTagDetectionArray::ConstPtr>& cur_queue = agent_apriltags_q_m_[frame_id];
          // Go through entire queue
          while(!cur_queue.empty()){
            addTag(cur_queue.front(), karto_scan);
            cur_queue.pop();
          }
        }
      }
      //ROS_INFO("[Debug] not scan");
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SynchronousSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  const ros::Time now = ros::Time::now();
  if (scan_queue_max_delay_ > ros::Duration(0.0) &&
      !scan->header.stamp.isZero() &&
      !now.isZero() &&
      now > scan->header.stamp &&
      (now - scan->header.stamp) > scan_queue_max_delay_)
  {
    ROS_WARN_THROTTLE(5.,
      "Dropping stale incoming scan for %s delayed by %.3fs before queueing",
      scan->header.frame_id.c_str(),
      (now - scan->header.stamp).toSec());
    return;
  }

  // no odom info on any pose helper
  //ROS_INFO("[Debug] laserCallback run");
  karto::Pose2 pose;//
  bool found_odom = false;
  for(size_t idx = 0; idx < pose_helpers_.size(); idx++)
  {
    found_odom = pose_helpers_[idx]->getOdomPose(pose, scan->header.stamp, scan->header.frame_id);
    if(found_odom){
      //ROS_INFO("[Debug] found");
      break;
    }

  }
  if(!found_odom){
    ROS_INFO("[Debug] not found");
    return;
  }//
  // ensure the laser can be used
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    ROS_INFO("[Debug] laser cannot be used");
    return;
  }

  // if sync and valid, add to queue
  if (shouldProcessScan(scan, pose))
  {
    //ROS_INFO("[Debug] adding to queue");
    boost::mutex::scoped_lock lock(q_mutex_);
    if (scan_queue_size_limit_ > 0 && q_.size() >= scan_queue_size_limit_)
    {
      ROS_WARN_THROTTLE(5.,
        "Sync queue reached limit (%zu), dropping oldest scan to keep latency bounded",
        scan_queue_size_limit_);
      q_.pop();
    }
    q_.push(PosedScan(scan, pose));
  }

  return;
}

/*****************************************************************************/
void SynchronousSlamToolbox::apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& apriltags) {//
/*****************************************************************************/
  boost::mutex::scoped_lock lock(apriltag_q_mutex_);
  std::stringstream ss(apriltags->header.frame_id);
  std::string frame_id;
  std::getline(ss, frame_id, '/');
  // Put non-empty apriltag array into map
  if (apriltags->detections.size() > 0 && apriltags->detections[0].id.size() > 0) {
    if (agent_apriltags_q_m_.find(frame_id) == agent_apriltags_q_m_.end())
      agent_apriltags_q_m_[frame_id] = std::queue<apriltag_ros::AprilTagDetectionArray::ConstPtr>();
    // Push to queue if size permits
    // Queue reflects apriltags detected by this agent over time
    if(agent_apriltags_q_m_[frame_id].size() < 10)
      agent_apriltags_q_m_[frame_id].push(apriltags);
  }
}

/*****************************************************************************/
bool SynchronousSlamToolbox::clearQueueCallback(
  slam_toolbox_msgs::ClearQueue::Request& req,
  slam_toolbox_msgs::ClearQueue::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("SynchronousSlamToolbox: Clearing all queued scans to add to map.");
  while(!q_.empty())
  {
    q_.pop();
  }
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request& req,
  slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if (req.match_type == procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

/*****************************************************************************/
/*bool SynchronousSlamToolbox::resetCallback(
    slam_toolbox_msgs::Reset::Request& req,
    slam_toolbox_msgs::Reset::Response& resp)*/
/*****************************************************************************/
/*{
  {
    boost::mutex::scoped_lock lock(q_mutex_);
    // Clear the scan queue.
    while (!q_.empty()) {
      q_.pop();
    }
  }
  return SlamToolbox::resetCallback(req, resp);
}*/

} // end namespace
