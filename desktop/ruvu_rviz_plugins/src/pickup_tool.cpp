#include "pickup_tool.h"

namespace ruvu_rviz_plugins
{
PickupTool::PickupTool() : BasePoseTool("Pickup")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::PickupTool, rviz::Tool)
