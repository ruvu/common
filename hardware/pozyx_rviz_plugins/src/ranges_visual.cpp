//
// Copyright (c) 2018 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <vector>

#include "./ranges_visual.h"

namespace pozyx_rviz_plugins
{
RangesVisual::RangesVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  text_visual_.reset(new rviz::MovableText("-"));
  frame_node_->attachObject(text_visual_.get());
}

RangesVisual::~RangesVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void RangesVisual::updateRangeInfo(int32_t remote_network_id, double distance, double rssi, double stamp)
{
  ranges_text_[remote_network_id].first = stamp;

  std::stringstream ss;
  ss << "" << remote_network_id << ": " << distance << "m";

  ranges_text_[remote_network_id].second = ss.str();
}

void RangesVisual::updateVisual(const Ogre::Vector3& position, const Ogre::Quaternion& orientation,
                                const Ogre::ColourValue& color, float character_height, const Ogre::Vector3& offset,
                                double stamp)
{
  removeRangesOlderThanTime(stamp);

  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);

  std::stringstream ss;
  ss << "Ranges [" << ranges_text_.size() << "]:\n\n";
  for (auto it : ranges_text_)
  {
    ss << it.second.second << "\n";
  }
  frame_node_->translate(offset);
  text_visual_->setCaption(ss.str());
  text_visual_->setColor(color);
  text_visual_->setCharacterHeight(character_height);
  text_visual_->setTextAlignment(rviz::MovableText::H_LEFT, rviz::MovableText::V_ABOVE);
}

void RangesVisual::removeRangesOlderThanTime(double stamp)
{
  // Check ids that are too old
  std::vector<int32_t> ids_to_be_removed;
  for (auto& it : ranges_text_)
  {
    if (it.second.first < stamp)
    {
      ids_to_be_removed.push_back(it.first);
    }
  }

  // Remove stale visuals
  for (auto id_to_be_removed : ids_to_be_removed)
  {
    ranges_text_.erase(ranges_text_.find(id_to_be_removed));
  }
}

}  // namespace pozyx_rviz_plugins
