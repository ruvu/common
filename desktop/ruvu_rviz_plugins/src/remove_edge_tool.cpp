//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./remove_edge_tool.h"

namespace ruvu_rviz_plugins
{
RemoveEdgeTool::RemoveEdgeTool() : BasePointTool("Remove edge")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::RemoveEdgeTool, rviz::Tool)
