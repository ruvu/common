//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <string>
#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "./ranges_display.h"
#include "./ranges_visual.h"

namespace pozyx_rviz_plugins
{
RangesDisplay::RangesDisplay()
{
  color_property_ = new rviz::ColorProperty("Label color", Qt::white, "Label color", this);
  character_height_property_ = new rviz::FloatProperty("Character height", 0.1, "Character height", this);

  offset_x_property_ = new rviz::FloatProperty("Offset x", 0, "Offset x", this);
  offset_y_property_ = new rviz::FloatProperty("Offset y", 0, "Offset y", this);
  offset_z_property_ = new rviz::FloatProperty("Offset z", 0, "Offset z", this);
  range_lifetime_property_ = new rviz::FloatProperty("Range lifetime", 1.0, "Range liftime in seconds", this);
}

RangesDisplay::~RangesDisplay()
{
}

void RangesDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void RangesDisplay::reset()
{
  MFDClass::reset();
}

void RangesDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  std::string topic = topic_property_->getTopicStd();
  if (!topic.empty())
  {
    array_sub_.shutdown();

    try
    {
      ROS_INFO("Subscribed to %s", topic.c_str());
      array_sub_ = update_nh_.subscribe(topic, 10, &RangesDisplay::processMessageArray, this);
      setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }
}

void RangesDisplay::unsubscribe()
{
  array_sub_.shutdown();
}

void RangesDisplay::processMessageArray(const pozyx_msgs::Ranges::ConstPtr& msg)
{
  ++messages_received_;
  ROS_DEBUG("Received %d messages", messages_received_);

  for (const pozyx_msgs::Range& range : msg->ranges)
  {
    tf_filter_->add(pozyx_msgs::Range::Ptr(new pozyx_msgs::Range(range)));
  }
}

void RangesDisplay::processMessage(const pozyx_msgs::Range::ConstPtr& msg)
{
  ROS_DEBUG("Processing range msg");

  // Update the visual properties based on the rviz state (fixed frame and properties)
  for (const auto& frame_id_ranges_visual_pair : frame_id_ranges_visual_map_)
  {
    const std::string& frame_id = frame_id_ranges_visual_pair.first;
    RangesVisual& visual = *frame_id_ranges_visual_pair.second;

    // Transform to rviz fixed frame
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(frame_id, msg->header.stamp, position, orientation))
    {
      ROS_WARN("Error transforming from frame %s to frame %s", frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
    Ogre::Vector3 offset_vector(offset_x_property_->getFloat(), offset_y_property_->getFloat(),
                                offset_z_property_->getFloat());

    visual.updateVisual(position, orientation, color_property_->getOgreColor(), character_height_property_->getFloat(),
                        offset_vector, msg->header.stamp.toSec() - range_lifetime_property_->getFloat());
  }

  // Initialize if the frame id does not exist yet
  if (frame_id_ranges_visual_map_.find(msg->header.frame_id) == frame_id_ranges_visual_map_.end())
  {
    ROS_DEBUG("Creating ranges visual for %s (network_id=%d)", msg->header.frame_id.c_str(), msg->network_id);
    frame_id_ranges_visual_map_[msg->header.frame_id] =
        std::shared_ptr<RangesVisual>(new RangesVisual(context_->getSceneManager(), scene_node_));
  }

  // Update the ranges visual
  frame_id_ranges_visual_map_[msg->header.frame_id]->updateRangeInfo(msg->remote_network_id, msg->distance, msg->RSS,
                                                                     msg->header.stamp.toSec());
}

}  // namespace pozyx_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pozyx_rviz_plugins::RangesDisplay, rviz::Display)
