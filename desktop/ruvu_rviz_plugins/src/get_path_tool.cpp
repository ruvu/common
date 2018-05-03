//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./get_path_tool.h"

namespace ruvu_rviz_plugins
{
GetPathTool::GetPathTool() : BasePointTool("Get path")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::GetPathTool, rviz::Tool)
