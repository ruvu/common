#include "base_pose_tool.h"

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>

namespace ruvu_rviz_plugins
{
BasePoseTool::BasePoseTool(std::string name)
{
  // shortcut_key_ = 'l';
  topic_property_ = new rviz::StringProperty("Topic", "goal", "The topic on which to publish pick goals.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
  this->name = name;
}

BasePoseTool::~BasePoseTool()
{
}

void BasePoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName(this->name.c_str());
  updateTopic();
}

void BasePoseTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
}

void BasePoseTool::onPoseSet(double x, double y, double theta)
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
}
