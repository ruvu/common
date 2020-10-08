//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#pragma once

#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <map>
#include <string>
#include <utility>

namespace Ogre
{
class Vector3;
class Quaternion;
class ColorValue;
}

namespace pozyx_rviz_plugins
{
class RangesVisual
{
public:
  RangesVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  virtual ~RangesVisual();

  void updateRangeInfo(int32_t remote_network_id, double distance, double rssi, double stamp);
  void updateVisual(const Ogre::Vector3& position, const Ogre::Quaternion& orientation, const Ogre::ColourValue& color,
                    float character_height, const Ogre::Vector3& offset, double stamp);

private:
  void removeRangesOlderThanTime(double stamp);

  std::shared_ptr<rviz::MovableText> text_visual_;
  std::map<int32_t, std::pair<double, std::string>> ranges_text_;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};

}  // namespace pozyx_rviz_plugins
