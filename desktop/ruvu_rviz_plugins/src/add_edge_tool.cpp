//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./add_edge_tool.h"

namespace ruvu_rviz_plugins
{
AddEdgeTool::AddEdgeTool() : BasePointTool("Add edge")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::AddEdgeTool, rviz::Tool)
