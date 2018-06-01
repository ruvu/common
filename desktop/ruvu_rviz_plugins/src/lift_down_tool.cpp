//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./lift_down_tool.h"

namespace ruvu_rviz_plugins
{
LiftDownTool::LiftDownTool() : BaseSingleJointPositionActionTool("Lift down")
{
}
}  // namespace ruvu_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::LiftDownTool, rviz::Tool)
