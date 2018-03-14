//
// Copyright (c) 2018 RUVU Robotics
//
// @author Ramon Wijnands
//

#include "./graph_goal_tool.h"

namespace ruvu_rviz_plugins
{
GraphGoalTool::GraphGoalTool() : BasePointTool("Graph goal")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::GraphGoalTool, rviz::Tool)
