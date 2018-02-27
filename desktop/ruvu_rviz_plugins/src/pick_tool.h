#pragma once

#include <rviz/default_plugin/tools/pose_tool.h>

namespace rviz
{
class StringProperty;
}

namespace ruvu_rviz_plugins
{
class PickTool : public rviz::PoseTool
{
  Q_OBJECT
public:
  PickTool();

  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateName();
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
};
}
