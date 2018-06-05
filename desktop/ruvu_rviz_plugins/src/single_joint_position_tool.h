//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/tool.h>
#include <string>

// forward declare
namespace rviz
{
class StringProperty;
class FloatProperty;
}

namespace ruvu_rviz_plugins
{
class SingleJointPositionTool : public rviz::Tool
{
  Q_OBJECT

public:
  explicit SingleJointPositionTool();
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent();

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
  rviz::FloatProperty* position_property_;
};
}  // namespace ruvu_rviz_plugins
