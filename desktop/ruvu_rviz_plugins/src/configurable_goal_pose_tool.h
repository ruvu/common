#pragma once

#include <rviz/default_plugin/tools/pose_tool.h>

namespace rviz
{
class StringProperty;
}

namespace ruvu_rviz_plugins
{
class ConfigurableGoalPoseTool : public rviz::PoseTool
{
  Q_OBJECT
public:
  ConfigurableGoalPoseTool();

  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updatePublishers();

private:
  ros::NodeHandle nh_;
  rviz::StringProperty* namespace_;

  std::map<std::string, ros::Publisher> publisher_map_;

  int selected_index_;
};

}
