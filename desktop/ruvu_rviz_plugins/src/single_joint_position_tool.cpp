//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./single_joint_position_tool.h"

#include <control_msgs/SingleJointPositionActionGoal.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <string>

namespace ruvu_rviz_plugins
{
SingleJointPositionTool::SingleJointPositionTool()
  : topic_property_(new rviz::StringProperty("Topic", "goal", "The topic on which to publish the joint position goal.",
                                             getPropertyContainer(), SLOT(updateTopic()), this))
  , position_property_(new rviz::FloatProperty("Joint position", 0, "Joint position setpoint", getPropertyContainer()))
{
}

void SingleJointPositionTool::onInitialize()
{
  updateTopic();
}

void SingleJointPositionTool::updateTopic()
{
  pub_ = nh_.advertise<control_msgs::SingleJointPositionActionGoal>(topic_property_->getStdString(), 1);
}

void SingleJointPositionTool::activate()
{
  control_msgs::SingleJointPositionActionGoal msg;
  msg.header.stamp = ros::Time::now();
  msg.goal.position = position_property_->getFloat();
  pub_.publish(msg);
  Q_EMIT(close());
}
void SingleJointPositionTool::deactivate()
{
}

int SingleJointPositionTool::processMouseEvent()
{
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::SingleJointPositionTool, rviz::Tool)
