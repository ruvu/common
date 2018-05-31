//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./base_single_joint_position_action_tool.h"

#include <control_msgs/SingleJointPositionActionGoal.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <string>

namespace ruvu_rviz_plugins
{
BaseSingleJointPositionActionTool::BaseSingleJointPositionActionTool(std::string name)
  : name_(name)
  , topic_property_(new rviz::StringProperty("Topic", "goal", "The topic on which to publish the joint position goal.",
                                             getPropertyContainer(), SLOT(updateTopic()), this))
  , position_property_(new rviz::FloatProperty("Joint position", 0, "Joint position setpoint", getPropertyContainer()))
{
}

void BaseSingleJointPositionActionTool::onInitialize()
{
  setName(name_.c_str());
  updateTopic();
}

void BaseSingleJointPositionActionTool::updateTopic()
{
  pub_ = nh_.advertise<control_msgs::SingleJointPositionActionGoal>(topic_property_->getStdString(), 1);
}

void BaseSingleJointPositionActionTool::activate()
{
  control_msgs::SingleJointPositionActionGoal msg;
  msg.header.stamp = ros::Time::now();
  msg.goal.position = position_property_->getFloat();
  pub_.publish(msg);
  Q_EMIT(close());
}
void BaseSingleJointPositionActionTool::deactivate()
{
}

int BaseSingleJointPositionActionTool::processMouseEvent()
{
}
}  // namespace ruvu_rviz_plugins
