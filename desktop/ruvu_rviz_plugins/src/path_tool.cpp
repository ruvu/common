//
// Copyright (c) 2021 RUVU Robotics
//
// @author Paul Verhoeckx
//

#include "./path_tool.hpp"

#include <ros/console.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ruvu_rviz_plugins
{
PathTool::PathTool()
  : arrow_(NULL)
{
  // shortcut_key_ = 'l';
  this->topic_property_ = new rviz::StringProperty("Topic", "path", "The topic on which to publish the path.",
                                                   getPropertyContainer(), SLOT(updateTopic()), this);
}

PathTool::~PathTool()
{
  delete arrow_;
}

void PathTool::onInitialize()
{
  arrow_ = new rviz::Arrow(scene_manager_, NULL, 1.0f, 0.02f, 0.0f, 0.0f);
  arrow_->setColor(0.0f, 0.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
  updateTopic();
}

void PathTool::activate()
{
  setStatus("Click and drag mouse to create path.");
  state_ = Point1;
}

void PathTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

void PathTool::updateTopic()
{
  pub_ = nh_.advertise<nav_msgs::Path>(topic_property_->getStdString(), 1);
}

int PathTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  if ( event.leftDown() )
  {
    ROS_ASSERT(state_ == Point1);

    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if ( rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                           ground_plane,
                                           event.x, event.y, pos1_) )
    {
      arrow_->setPosition(pos1_);

      state_ = Point2;
      flags |= Render;
    }
  }
  else if ( event.type == QEvent::MouseMove && event.left() )
  {
    if ( state_ == Point2 )
    {
      // compute angle in x-y plane
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if ( rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                             ground_plane,
                                             event.x, event.y, pos2_) )
      {
        double angle = atan2(pos2_.y - pos1_.y, pos2_.x - pos1_.x);

        arrow_->getSceneNode()->setVisible(true);

        // we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

        arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

        Ogre::Vector3 scale;
        double dist = sqrt(pow(pos2_.x - pos1_.x, 2) + pow(pos2_.y - pos1_.y, 2));
        scale.x = dist;
        scale.y = scale.z = 1;
        arrow_->setScale(scale);

        flags |= Render;
      }
    }
  }
  else if ( event.leftUp() )
  {
    if ( state_ == Point2 )
    {
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if ( rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                             ground_plane,
                                             event.x, event.y, pos2_) )
      {
        onPathSet(pos1_.x, pos1_.y, pos2_.x, pos2_.y);
        arrow_->getSceneNode()->setVisible(false);
        flags |= (Finished|Render);
      }
    }
  }

  return flags;
}

void PathTool::onPathSet(double x1, double y1, double x2, double y2)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = fixed_frame;

  double theta = atan2(pos2_.y - pos1_.y, pos2_.x - pos1_.x);
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, theta);
  geometry_msgs::Quaternion quat;
  quat = tf2::toMsg(quat_tf);

  geometry_msgs::PoseStamped p1;
  p1.header = header;
  p1.pose.orientation = quat;
  p1.pose.position.x = x1;
  p1.pose.position.y = y1;

  auto p2 = p1;
  p2.pose.position.x = x2;
  p2.pose.position.y = y2;

  nav_msgs::Path path;
  path.header = header;
  path.poses.push_back(p1);
  path.poses.push_back(p2);

  ROS_INFO("Setting path: Frame:%s, Position 1(%.3f, %.3f, %.3f), Position 2(%.3f, %.3f, %.3f)\n",
           fixed_frame.c_str(), path.poses.at(0).pose.position.x, path.poses.at(0).pose.position.y,
           path.poses.at(0).pose.position.z, path.poses.at(1).pose.position.x, path.poses.at(1).pose.position.y,
           path.poses.at(1).pose.position.z);
  pub_.publish(path);
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::PathTool, rviz::Tool)
