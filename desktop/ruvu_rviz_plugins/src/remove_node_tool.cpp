//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./remove_node_tool.h"

namespace ruvu_rviz_plugins
{
RemoveNodeTool::RemoveNodeTool() : BasePointTool("Remove node")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::RemoveNodeTool, rviz::Tool)
