//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./lateral_goal_tool.h"

namespace ruvu_rviz_plugins
{
LateralGoalTool::LateralGoalTool() : BasePoseTool("Lateral goal")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::LateralGoalTool, rviz::Tool)
