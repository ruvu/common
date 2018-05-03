//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./add_node_tool.h"

namespace ruvu_rviz_plugins
{
AddNodeTool::AddNodeTool() : BasePoseTool("Add node")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::AddNodeTool, rviz::Tool)
