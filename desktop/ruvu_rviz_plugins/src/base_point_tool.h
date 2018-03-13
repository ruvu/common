//
// Copyright (c) 2018 RUVU Robotics
//
// @author Ramon Wijnands
//

#pragma once

#include <rviz/tool.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <string>

// forward declare
namespace rviz
{
class StringProperty;
class BoolProperty;
}

namespace ruvu_rviz_plugins
{
class BasePointTool : public rviz::Tool
{
  Q_OBJECT
public:
  explicit BasePointTool(std::string name);
  virtual ~BasePointTool();
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

private Q_SLOTS:
  void updateTopic();
  void updateAutoDeactivate();

private:
  QCursor std_cursor_;
  QCursor hit_cursor_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* auto_deactivate_property_;

  std::string name;
};
}  // namespace ruvu_rviz_plugins
