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
class BasePoseTool : public rviz::PoseTool
{
  Q_OBJECT
public:
  explicit BasePoseTool(std::string name);
  virtual ~BasePoseTool();

  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
  std::string name;
};
}  // namespace ruvu_rviz_plugins
