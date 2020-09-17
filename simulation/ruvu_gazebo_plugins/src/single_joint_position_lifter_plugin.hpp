//
// Copyright (c) 2017 RUVU Robotics
//
// @author Rein Appeldoorn
//
#pragma once

#include <gazebo/common/Plugin.hh>

#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>

#include <control_msgs/SingleJointPositionAction.h>
#include <actionlib/server/action_server.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
//!
//! \brief The SingleJointPositionLifterPlugin class that exposes an actionlib interface to lift bodies for moving
//!
//! It will attach a body to the model when lifting, and drop when dropping. Note that it is not moving the actual joint
//! but it will only attach the body on top of the current model (of the plugin).
//!
class SingleJointPositionLifterPlugin : public ModelPlugin
{
public:
  //!
  //! \brief Load Register update function
  //! \param ptr Pointer to the world
  //!
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  typedef actionlib::ActionServer<control_msgs::SingleJointPositionAction> SingleJointPositionActionServer;

  //!
  //! \brief model_ Pointer to the parent model
  //!
  physics::ModelPtr model_;

  //!
  //! \brief lift_model_ Pointer to the model we are lifting
  //!
  physics::ModelPtr lift_model_;
  ignition::math::Pose3d lift_world_pose_relative_to_model_;

  //!
  //! \brief joint_ Pointer to the joint that needs to be actuated
  //!
  physics::JointPtr joint_;

  //!
  //! \brief joint_ Joint position, we do not update the actual position
  //!
  double joint_position_;

  //!
  //! \brief state_publish_rate_ How often do we send out joint state
  //!
  double state_publish_rate_;
  sensor_msgs::JointState joint_state_msg_;
  ros::Publisher joint_state_publisher_;

  //!
  //! \brief update_connection_ Connection that triggers the callback function of the model
  //!
  event::ConnectionPtr update_connection_;

  //!
  //! \brief rosnode_ Handle to the ROS node
  //!
  std::shared_ptr<ros::NodeHandle> rosnode_;

  //!
  //! \brief goalCallback Action goal callback
  //! \param goal Incoming action goal
  //!
  std::shared_ptr<SingleJointPositionActionServer> action_server_;
  void goalCallback(SingleJointPositionActionServer::GoalHandle goal);

  //!
  //! \brief getModelAboveUs Returns the modelptr
  //! \return model above model
  //!
  physics::ModelPtr getModelAboveUs();

  //!
  //! \brief UpdateChild Called on every tick of the simulation; updates the position and velocity of the parent
  //!
  void UpdateChild();

  //!
  //! \brief FiniChild Called on shutdown
  //!
  void FiniChild();

  //!
  //! \brief mutex_ Thread safety for ROS and gazebo thread
  //!
  std::mutex mutex_;
};
}  // namespace gazebo
