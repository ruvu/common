//
// Copyright (c) 2018 RUVU Robotics
//
// @author Ramon Wijnands
//

#include "./pose_tool.h"

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>
#include <string>

namespace ruvu_rviz_plugins
{
PoseTool::PoseTool()
{
  // shortcut_key_ = 'l';
  topic_property_ = new rviz::StringProperty("Topic", "goal", "The topic on which to publish pick goals.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
}

PoseTool::~PoseTool()
{
}

void PoseTool::onInitialize()
{
  rviz::PoseTool::onInitialize();
  updateTopic();
}

void PoseTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
}

void PoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n",
           fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
           goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  pub_.publish(goal);
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::PoseTool, rviz::Tool)
