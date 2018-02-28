#include "dock_goal_tool.h"

namespace ruvu_rviz_plugins
{
DockGoalTool::DockGoalTool() : BasePoseTool("Dock goal")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::DockGoalTool, rviz::Tool)
