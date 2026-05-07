#ifndef ADD_AGENT_TOOL_H
#define ADD_AGENT_TOOL_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;
}

namespace robosar_gui
{
class AddAgentTool : public rviz::PoseTool
{
Q_OBJECT
public:
  AddAgentTool();
  ~AddAgentTool() override
  {
  }
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
};

} // namespace robosar_gui
#endif // ADD_AGENT_TOOL_H