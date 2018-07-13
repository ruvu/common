#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/movable_text.h>

#include "ranges_visual.h"

namespace pozyx_rviz_plugins
{

RangesVisual::RangesVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  text_visual_.reset(new rviz::MovableText(""));
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
  ss << "- " << remote_network_id << ": distance=" << distance << ", rssi=" << rssi << ", stamp=" << stamp;

  ranges_text_[remote_network_id].second = ss.str();
}

void RangesVisual::updateVisual(const Ogre::Vector3& position, const Ogre::Quaternion& orientation, double stamp)
{
  removeRangesOlderThanTime(stamp);

  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);

  std::stringstream ss;
  for (auto it : ranges_text_)
  {
    ss << it.second.second << "\n";
  }
  text_visual_->setCaption(ss.str());
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

} // namespace pozyx_rviz_plugins
