//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./longitudinal_goal_tool.h"

namespace ruvu_rviz_plugins
{
LongitudinalGoalTool::LongitudinalGoalTool() : BasePoseTool("Longitudinal goal")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::LongitudinalGoalTool, rviz::Tool)
