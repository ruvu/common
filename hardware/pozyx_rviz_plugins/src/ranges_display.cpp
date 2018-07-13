#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "ranges_display.h"
#include "ranges_visual.h"

namespace pozyx_rviz_plugins
{

RangesDisplay::RangesDisplay()
{
  updateProperties();
}

void RangesDisplay::updateProperties()
{
}

void RangesDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

RangesDisplay::~RangesDisplay()
{
}

void RangesDisplay::reset()
{
  MFDClass::reset();
}

void RangesDisplay::processMessage(const pozyx_msgs::Ranges::ConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  // Update the visual properties based on the incoming msg
  for (const pozyx_msgs::Range& range : msg->ranges)
  {
    // Initialize if the frame id does not exist yet
    if (frame_id_ranges_visual_map_.find(range.header.frame_id) == frame_id_ranges_visual_map_.end())
    {
      frame_id_ranges_visual_map_[range.header.frame_id] =
          std::shared_ptr<RangesVisual>(new RangesVisual(context_->getSceneManager(), scene_node_));
    }
  }

  // Update the visual properties based on the rviz state (fixed frame and properties)
  for (const auto& frame_id_ranges_visual_pair : frame_id_ranges_visual_map_)
  {
    const std::string& frame_id = frame_id_ranges_visual_pair.first;
    RangesVisual& visual = *frame_id_ranges_visual_pair.second;

    // Transform to rviz fixed frame
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(frame_id, now, position, orientation))
    {
      ROS_DEBUG("Error transforming from frame %s to frame %s", frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
    visual.updateVisual(position, orientation, now.toSec() - 1);
  }
}

}  // namespace pozyx_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pozyx_rviz_plugins::RangesDisplay, rviz::Display)
