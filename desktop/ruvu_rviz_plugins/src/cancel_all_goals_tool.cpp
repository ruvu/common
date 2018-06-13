//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./cancel_all_goals_tool.h"

#include <actionlib_msgs/GoalID.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <string>

namespace ruvu_rviz_plugins
{
CancelAllGoalsTool::CancelAllGoalsTool()
  : topic_property_(new rviz::StringProperty("Topic", "goal", "The topic on which to publish the cancel goal.",
                                             getPropertyContainer(), SLOT(updateTopic()), this))
{
}

void CancelAllGoalsTool::onInitialize()
{
  updateTopic();
}

void CancelAllGoalsTool::updateTopic()
{
  pub_ = nh_.advertise<actionlib_msgs::GoalID>(topic_property_->getStdString(), 1);
}

void CancelAllGoalsTool::activate()
{
  pub_.publish(actionlib_msgs::GoalID());
  Q_EMIT(close());
}
void CancelAllGoalsTool::deactivate()
{
}

int CancelAllGoalsTool::processMouseEvent()
{
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::CancelAllGoalsTool, rviz::Tool)
