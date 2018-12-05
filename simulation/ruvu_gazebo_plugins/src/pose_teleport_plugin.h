//
// Copyright (c) 2018 RUVU Robotics
//
// @author Paul Verhoeckx
//

#pragma once

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <math.h>

namespace gazebo
{
//!
//! \brief The PoseTeleportPlugin class that alters a model based on a pose command
//!
class PoseTeleportPlugin : public ModelPlugin
{
public:
  //!
  //! \brief Load Register update function
  //! \param ptr Pointer to the world
  //!
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

protected:
  //!
  //! \brief Update Update function that needs to be implemented by the child class
  //! \param pose The current pose of the model (derived from the input)
  //! \param model Pointer to the model, can be used to update the model
  //!
  void Update(const math::Pose& pose, physics::ModelPtr model);

private:
  //!
  //! \brief model_ Pointer to the parent model
  //!
  physics::ModelPtr model_;

  //!
  //! \brief update_connection_ Connection that triggers the callback function of the model
  //!
  event::ConnectionPtr update_connection_;

  //!
  //! \brief mutex_ Mutex to make sure the physics thread and the ROS callback queue thread do not collide
  //!
  std::mutex mutex_;

  //!
  //! \brief rosnode_ Handle to the ROS node
  //!
  std::shared_ptr<ros::NodeHandle> rosnode_;

  //!
  //! \brief QueueThread Custom callback queue thread in order to not block the physics engine
  //!
  bool alive_;                           //! To ensure we shutdown the thread when the plugin dies
  ros::CallbackQueue queue_;             // The ROS callback queue
  boost::thread callback_queue_thread_;  // The ROS message handler thread
  void QueueThread();

  //!
  //! \brief poseCmdCallback Callback of the pose cmd
  //! \param pose_msg Pose command
  //!
  geometry_msgs::PoseConstPtr pose_msg_;  // Last received pose_msg
  ros::Subscriber pose_subscriber_;      // The ROS subcriber on the Pose msg
  void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg);

  //!
  //! \brief getCurrentPose Get the current pose based on the input pose and the last received time
  //!
  //! If the command is older than the pose timeout, we will return zero
  //!
  //! \param now The current time
  //! \return Return the current pose
  //!
  math::Pose getCurrentPose();

  //!
  //! \brief UpdateChild Called on every tick of the simulation; updates the position and velocity of the parent
  //!
  //! Also publishes the odometry if we exceed the step time of the odom
  //!
  void UpdateChild();

  //!
  //! \brief FiniChild Called on shutdown
  //!
  void FiniChild();
};
}  // namespace gazebo
