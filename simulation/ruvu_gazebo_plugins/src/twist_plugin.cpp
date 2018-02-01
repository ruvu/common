#include "twist_plugin.h"
#include "util.h"

namespace gazebo
{

void TwistPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  //! Store the model so that we can use it later
  model_ = model;

  //! Parse SDF for parameters
  std::string robot_namespace = getParameterFromSDF(sdf, "robotNamespace", std::string(""));
  std::string tf_prefix = robotNamespaceToTFPrefix(robot_namespace);
  std::string command_topic = getParameterFromSDF(sdf, "commandTopic", std::string("cmd_vel"));
  publish_tf_ = getParameterFromSDF(sdf, "publishTF", false);
  cmd_timeout_ = getParameterFromSDF(sdf, "commandTimeout", 0.5);
  std::string odom_topic = getParameterFromSDF(sdf, "odometryTopic",  std::string("odom"));
  odom_msg_.header.frame_id = tf_prefix + getParameterFromSDF(sdf, "odometryFrame",  std::string("odom"));
  odometry_rate_ = getParameterFromSDF(sdf, "odometryRate", 20.0);
  odom_msg_.child_frame_id = tf_prefix + getParameterFromSDF(sdf, "robotFrame",  std::string("base_link"));
  transform_stamped_.header.frame_id = odom_msg_.header.frame_id;
  transform_stamped_.child_frame_id = odom_msg_.child_frame_id;

  //! Store last update, required for calculating the dt
  last_update_time_ = model_->GetWorld()->GetSimTime();

  //! Make sure that if we finish, we also shutdown the callback thread
  alive_ = true;

  // Ensure that ROS has been initialized (required for using ROS COM)
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("twist_teleport", "TwistTeleportPlugin"
      << ": A ROS node for Gazebo has not been initialized, "
      << "unable to load plugin. Load the Gazebo system plugin "
      << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  rosnode_.reset(new ros::NodeHandle(robot_namespace));

  // subscribe to the command topic
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        command_topic, 1, boost::bind(&TwistPlugin::twistCallback, this, _1), ros::VoidPtr(), &queue_);
  twist_subscriber_ = rosnode_->subscribe(so);
  callback_queue_thread_ = boost::thread(boost::bind(&TwistPlugin::QueueThread, this));

  // Initialize odometry publisher
  odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odom_topic, 1);

  // Initialize odom state
  odom_pose_ = math::Pose::Zero;

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TwistPlugin::UpdateChild, this));
}

void TwistPlugin::QueueThread() {
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void TwistPlugin::twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_msg_ = cmd_msg;
  last_cmd_time_ = model_->GetWorld()->GetSimTime();
}

geometry_msgs::Twist TwistPlugin::getCurrentVelocity(const common::Time& now) {
  geometry_msgs::Twist velocity;
  if (cmd_msg_ && (now - last_cmd_time_).Double() < cmd_timeout_) {
    velocity = *cmd_msg_;
  }
  return velocity;
}

void TwistPlugin::UpdateChild()
{
  std::lock_guard<std::mutex> lock(mutex_);

  common::Time now = model_->GetWorld()->GetSimTime();
  common::Time dt = now - last_update_time_;
  math::Pose pose = model_->GetWorldPose();
  geometry_msgs::Twist vel = getCurrentVelocity(now);

  updateOdometryPose(pose, vel, dt);

  if (odometry_rate_ > 0.0) {
    double seconds_since_last_update = (now - common::Time(odom_msg_.header.stamp.toSec())).Double();
    if (seconds_since_last_update > (1.0 / odometry_rate_)) {
      publishOdometry(vel, now);
    }
  }

  // Calculate the velocities based on last pose
  math::Vector3 world_linear_velocity = pose.rot * math::Vector3(vel.linear.x, vel.linear.y, vel.linear.z);
  math::Vector3 world_angular_velocity = pose.rot * math::Vector3(vel.angular.x, vel.angular.y, vel.angular.z);

  Update(pose, vel, world_linear_velocity, world_angular_velocity, dt.Double(), model_);

  last_update_time_ = now;
}

void TwistPlugin::updateOdometryPose(const math::Pose& pose, const geometry_msgs::Twist& velocity, const common::Time& dt)
{
  odom_pose_ = pose;
}

void TwistPlugin::publishOdometry(const geometry_msgs::Twist& velocity, const common::Time& now)
{
  odom_msg_.pose.pose.position.x = odom_pose_.pos.x;
  odom_msg_.pose.pose.position.y = odom_pose_.pos.y;
  odom_msg_.pose.pose.position.z = odom_pose_.pos.z;

  odom_msg_.pose.pose.orientation.x = odom_pose_.rot.x;
  odom_msg_.pose.pose.orientation.y = odom_pose_.rot.y;
  odom_msg_.pose.pose.orientation.z = odom_pose_.rot.z;
  odom_msg_.pose.pose.orientation.w = odom_pose_.rot.w;

  odom_msg_.twist.twist = velocity;

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

void TwistPlugin::FiniChild() {
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

}
