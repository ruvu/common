//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#pragma once

#include <map>
#include <pozyx_msgs/Ranges.h>
#include <rviz/message_filter_display.h>
#include <string>

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace pozyx_rviz_plugins
{
class RangesVisual;

class RangesDisplay : public rviz::MessageFilterDisplay<pozyx_msgs::Range>
{
  Q_OBJECT
public:
  RangesDisplay();
  virtual ~RangesDisplay();

private:
  void onInitialize();
  void reset();

  void subscribe();
  void unsubscribe();

  void processMessageArray(const pozyx_msgs::Ranges::ConstPtr& msg);
  void processMessage(const pozyx_msgs::Range::ConstPtr& msg);

  std::map<std::string, std::shared_ptr<RangesVisual>> frame_id_ranges_visual_map_;

  ros::Subscriber array_sub_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* character_height_property_;
  rviz::FloatProperty* offset_x_property_;
  rviz::FloatProperty* offset_y_property_;
  rviz::FloatProperty* offset_z_property_;
  rviz::FloatProperty* range_lifetime_property_;
};
}  // namespace pozyx_rviz_plugins
