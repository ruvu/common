//
// Copyright (c) 2018 RUVU Robotics
//
// @author Paul Verhoeckx
//

#include <string>

#include "./pose_teleport_plugin.h"
#include "./util.h"

#include <tf/transform_datatypes.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
void PoseTeleportPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  //! Store the model so that we can use it later
  model_ = model;

  //! Parse SDF for parameters
  std::string robot_namespace = getParameterFromSDF(sdf, "robotNamespace", std::string(""));
  std::string tf_prefix = robotNamespaceToTFPrefix(robot_namespace);
  std::string command_topic = getParameterFromSDF(sdf, "commandTopic", std::string("input_pose"));

  //! Make sure that if we finish, we also shutdown the callback thread
  alive_ = true;

  // Ensure that ROS has been initialized (required for using ROS COM)
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("pose_teleport", "PoseTeleportPlugin"
                                                << ": A ROS node for Gazebo has not been initialized, "
                                                << "unable to load plugin. Load the Gazebo system plugin "
                                                << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  rosnode_.reset(new ros::NodeHandle(robot_namespace));
  pose_msg_.reset(new geometry_msgs::Pose());

  // subscribe to the command topic
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
      command_topic, 1, boost::bind(&PoseTeleportPlugin::poseCallback, this, _1), ros::VoidPtr(), &queue_);
  pose_subscriber_ = rosnode_->subscribe(so);
  callback_queue_thread_ = boost::thread(boost::bind(&PoseTeleportPlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PoseTeleportPlugin::UpdateChild, this));
}

void PoseTeleportPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void PoseTeleportPlugin::poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_msg_ = pose_msg;
}

ignition::math::Pose3d PoseTeleportPlugin::getCurrentPose()
{
  ignition::math::Pose3d pose;
  if (pose_msg_)
  {
  }
  return pose;
}

void PoseTeleportPlugin::UpdateChild()
{
  std::lock_guard<std::mutex> lock(mutex_);

  ignition::math::Pose3d pose = pose;
  pose.pos = ignition::math::Vector3(pose_msg_->position.x, pose_msg_->position.y, pose_msg_->position.z);
  pose.rot = ignition::math::Quaterniond(pose_msg_->orientation.x, pose_msg_->orientation.y, pose_msg_->orientation.z,
                              pose_msg_->orientation.w);

  // Convert to from camera axis (x up, y front, z left) to model axis (x front, y, left, z up)
  tf::Quaternion q(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double camera_roll, camera_pitch, camera_yaw;
  camera_roll = -roll + M_PI / 2;
  camera_pitch = -yaw + M_PI / 2;
  camera_yaw = -pitch;

  tf::Quaternion q_camera = tf::createQuaternionFromRPY(camera_roll, camera_pitch, camera_yaw);
  pose.rot = ignition::math::Quaterniond(q_camera[0], q_camera[1], q_camera[2], q_camera[3]);

  // Update the model in gazebo
  model_->SetWorldPose(pose);
}

void PoseTeleportPlugin::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}
GZ_REGISTER_MODEL_PLUGIN(PoseTeleportPlugin)
}  // namespace gazebo
