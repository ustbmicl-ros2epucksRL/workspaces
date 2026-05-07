#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>

#include "add_agent.h"

namespace robosar_gui
{
  AddAgentTool::AddAgentTool()
  {
    shortcut_key_ = 'a';

    topic_property_ =
        new rviz::StringProperty("Topic", "agent_addition", "The topic on which to publish new agents.",
                                 getPropertyContainer(), SLOT(updateTopic()), this);
  }

  void AddAgentTool::onInitialize()
  {
    PoseTool::onInitialize();
    arrow_->setColor(1.0f, 0.0f, 1.0f, 1.0f);
    setName("Add Agent");
    updateTopic();
  }

  void AddAgentTool::updateTopic()
  {
    try
    {
      pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
    }
    catch (const ros::Exception &e)
    {
      ROS_ERROR_STREAM_NAMED("AddAgentTool", e.what());
    }
  }

  void AddAgentTool::onPoseSet(double x, double y, double theta)
  {
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation = tf2::toMsg(quat);
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.header.frame_id = fixed_frame;
    goal.header.stamp = ros::Time::now();
    ROS_INFO("Setting initial start: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = "
             "Angle: %.3f\n",
             fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
             goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
             goal.pose.orientation.w, theta);
    pub_.publish(goal);
  }

} // end namespace robosar_gui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robosar_gui::AddAgentTool, rviz::Tool)