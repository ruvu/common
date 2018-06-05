//
// Copyright (c) 2018 RUVU Robotics
//
// @author Ramon Wijnands
//

#include "./point_tool.h"

#include <geometry_msgs/PointStamped.h>
#include <OgreVector3.h>
#include <rviz/display_context.h>
#include <rviz/load_resource.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <string>

namespace ruvu_rviz_plugins
{
PointTool::PointTool()
{
  // shortcut_key_ = 'l';
  this->topic_property_ = new rviz::StringProperty("Topic", "goal", "The topic on which to publish pick goals.",
                                                   getPropertyContainer(), SLOT(updateTopic()), this);

  auto_deactivate_property_ =
      new rviz::BoolProperty("Single click", true, "Switch away from this tool after one click.",
                             getPropertyContainer(), SLOT(updateAutoDeactivate()), this);
}

PointTool::~PointTool()
{
}

void PointTool::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = rviz::getDefaultCursor();
}

void PointTool::activate()
{
}

void PointTool::deactivate()
{
}

void PointTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PointStamped>(topic_property_->getStdString(), 1);
}

void PointTool::updateAutoDeactivate()
{
}

int PointTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  Ogre::Vector3 pos;
  bool success = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success)
  {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus(s.str().c_str());

    if (event.leftUp())
    {
      geometry_msgs::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      pub_.publish(ps);

      if (auto_deactivate_property_->getBool())
      {
        flags |= Finished;
      }
    }
  }
  else
  {
    setStatus("Move over an object to select the target point.");
  }

  return flags;
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::PointTool, rviz::Tool)
