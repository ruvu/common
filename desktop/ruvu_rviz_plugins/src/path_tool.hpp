//
// Copyright (c) 2021 RUVU Robotics
//
// @author Paul Verhoeckx
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <string>

#include <OgreVector3.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/geometry.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/viewport_mouse_event.h>

#include <ros/ros.h>
#include <rviz/tool.h>

namespace rviz
{
class Arrow;
class StringProperty;
}

namespace ruvu_rviz_plugins
{
class PathTool : public rviz::Tool
{
  Q_OBJECT
public:
  PathTool();
  ~PathTool();

  void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

protected:
  void onPathSet(double x1, double y1, double x2, double y2);

  rviz::Arrow* arrow_;

  enum State
  {
    Point1,
    Point2
  };
  State state_;

  Ogre::Vector3 pos1_;
  Ogre::Vector3 pos2_;

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
};
}  // namespace ruvu_rviz_plugins
