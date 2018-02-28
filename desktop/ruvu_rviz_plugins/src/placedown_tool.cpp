#include "placedown_tool.h"

namespace ruvu_rviz_plugins
{
PlacedownTool::PlacedownTool() : BasePoseTool("Placedown")
{
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_rviz_plugins::PlacedownTool, rviz::Tool)
