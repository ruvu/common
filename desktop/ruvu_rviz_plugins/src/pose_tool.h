//
// Copyright (c) 2018 RUVU Robotics
//
// @author Ramon Wijnands
//

#pragma once

#include <rviz/default_plugin/tools/pose_tool.h>
#include <string>

namespace rviz
{
class StringProperty;
}

namespace ruvu_rviz_plugins
{
class PoseTool : public rviz::PoseTool
{
  Q_OBJECT
public:
  PoseTool();
  ~PoseTool();

  void onInitialize();

protected:
  void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
};
}  // namespace ruvu_rviz_plugins
