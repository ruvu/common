//
// Copyright (c) 2018 RUVU Robotics
//
// @author Paul Verhoeckx
//

#include <string>

#include "./pose_plugin.h"
#include "./util.h"

namespace gazebo
{
void PosePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  //! Store the model so that we can use it later
  model_ = model;

  //! Parse SDF for parameters
  std::string robot_namespace = getParameterFromSDF(sdf, "robotNamespace", std::string(""));
  std::string tf_prefix = robotNamespaceToTFPrefix(robot_namespace);
  std::string command_topic = getParameterFromSDF(sdf, "commandTopic", std::string("input_pose"));
  publish_tf_ = getParameterFromSDF(sdf, "publishTF", false);
  cmd_timeout_ = getParameterFromSDF(sdf, "commandTimeout", 0.5);
  std::string odom_topic = getParameterFromSDF(sdf, "odometryTopic", std::string("odom"));
  odom_msg_.header.frame_id = tf_prefix + getParameterFromSDF(sdf, "odometryFrame", std::string("odom"));
  odometry_rate_ = getParameterFromSDF(sdf, "odometryRate", 20.0);
  odom_msg_.child_frame_id = tf_prefix + getParameterFromSDF(sdf, "robotFrame", std::string("base_link"));
  double pose_noise = getParameterFromSDF(sdf, "poseGaussianNoise", 0.0);
  pose_covariance_ = { pose_noise, 0, 0, 0, 0, 0, 0, pose_noise, 0, 0, 0, 0, 0, 0,  // NOLINT
                       pose_noise, 0, 0, 0, 0, 0, 0, pose_noise, 0, 0, 0, 0, 0, 0,
                       pose_noise, 0, 0, 0, 0, 0, 0, pose_noise };
  double velocity_noise = getParameterFromSDF(sdf, "velocityGaussianNoise", 0.0);
  velocity_covariance_ = { velocity_noise, 0, 0, 0, 0, 0, 0, velocity_noise, 0, 0, 0, 0, 0, 0,  // NOLINT
                           velocity_noise, 0, 0, 0, 0, 0, 0, velocity_noise, 0, 0, 0, 0, 0, 0,
                           velocity_noise, 0, 0, 0, 0, 0, 0, velocity_noise };
  transform_stamped_.header.frame_id = odom_msg_.header.frame_id;
  transform_stamped_.child_frame_id = odom_msg_.child_frame_id;

  //! Store last update, required for calculating the dt
  last_update_time_ = model_->GetWorld()->GetSimTime();

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

  // subscribe to the command topic
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
      command_topic, 1, boost::bind(&PosePlugin::poseCallback, this, _1), ros::VoidPtr(), &queue_);
  pose_subscriber_ = rosnode_->subscribe(so);
  callback_queue_thread_ = boost::thread(boost::bind(&PosePlugin::QueueThread, this));

  // Initialize odometry publisher
  odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odom_topic, 1);

  // Initialize odom state
  odom_pose_ = math::Pose::Zero;

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PosePlugin::UpdateChild, this));
}

void PosePlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void PosePlugin::poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_msg_ = pose_msg;
  last_cmd_time_ = model_->GetWorld()->GetSimTime();
}

math::Pose PosePlugin::getCurrentPose(const common::Time& now)
{
    math::Pose pose;
    if (pose_msg_)
    {
      pose.pos = math::Vector3(pose_msg_->position.x, pose_msg_->position.y, pose_msg_->position.z);
      pose.rot = math::Quaternion(pose_msg_->orientation.x, pose_msg_->orientation.y,
                                  pose_msg_->orientation.z, pose_msg_->orientation.w);
    }
    return pose;
}

void PosePlugin::UpdateChild()
{
  std::lock_guard<std::mutex> lock(mutex_);

  common::Time now = model_->GetWorld()->GetSimTime();
  common::Time dt = now - last_update_time_;
  geometry_msgs::Twist vel;
  math::Pose pose = getCurrentPose(now);

  updateOdometryPose(pose);

  if (odometry_rate_ > 0.0)
  {
    double seconds_since_last_update = (now - common::Time(odom_msg_.header.stamp.toSec())).Double();
    if (seconds_since_last_update > (1.0 / odometry_rate_))
    {
      publishOdometry(vel, now);
    }
  }

  Update(pose, model_);

  last_update_time_ = now;
}

void PosePlugin::updateOdometryPose(const math::Pose& pose)
{
  odom_pose_ = pose;
}

void PosePlugin::publishOdometry(const geometry_msgs::Twist& velocity, const common::Time& now)
{
  odom_msg_.pose.pose.position.x = odom_pose_.pos.x;
  odom_msg_.pose.pose.position.y = odom_pose_.pos.y;
  odom_msg_.pose.pose.position.z = odom_pose_.pos.z;

  odom_msg_.pose.pose.orientation.x = odom_pose_.rot.x;
  odom_msg_.pose.pose.orientation.y = odom_pose_.rot.y;
  odom_msg_.pose.pose.orientation.z = odom_pose_.rot.z;
  odom_msg_.pose.pose.orientation.w = odom_pose_.rot.w;

  for (int i = 0; i < odom_msg_.pose.covariance.size(); i++)
  {
    odom_msg_.pose.covariance[i] = pose_covariance_[i];
  }

  odom_msg_.twist.twist = velocity;
  for (int i = 0; i < odom_msg_.twist.covariance.size(); i++)
  {
    odom_msg_.twist.covariance[i] = velocity_covariance_[i];
  }

  odom_msg_.header.stamp = ros::Time(now.Double());
  odometry_publisher_.publish(odom_msg_);

  // Publish transform
  transform_stamped_.header.stamp = ros::Time(now.Double());
  transform_stamped_.transform.translation.x = odom_pose_.pos.x;
  transform_stamped_.transform.translation.y = odom_pose_.pos.y;
  transform_stamped_.transform.translation.z = odom_pose_.pos.z;
  transform_stamped_.transform.rotation = odom_msg_.pose.pose.orientation;

  if (publish_tf_)
  {
    odom_broadcaster_.sendTransform(transform_stamped_);
  }
}

void PosePlugin::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}
}  // namespace gazebo
