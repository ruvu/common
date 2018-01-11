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
  cmd_timeout_ = getParameterFromSDF(sdf, "commandTimeout", 0.5);
  std::string odom_topic = getParameterFromSDF(sdf, "odometryTopic",  std::string("odom"));
  odom_msg_.header.frame_id = tf_prefix + getParameterFromSDF(sdf, "odometryFrame",  std::string("odom"));
  odometry_rate_ = getParameterFromSDF(sdf, "odometryRate", 20.0);
  odom_msg_.child_frame_id = tf_prefix + getParameterFromSDF(sdf, "robotFrame",  std::string("base_link"));

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
  x_ = 0;
  y_ = 0;
  yaw_ = 0;

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

  updateOdometryState(vel, now);

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

void TwistPlugin::updateOdometryState(const geometry_msgs::Twist& velocity, const common::Time& now)
{
  double dt = (now - last_update_time_).Double();
  double sin_yaw = sin(yaw_);
  double cos_yaw = cos(yaw_);

  // Calculate diffs in body fixed frame
  double dx = velocity.linear.x * dt;
  double dy = velocity.linear.y * dt;
  double dyaw = velocity.angular.z * dt;

  // Convert it to odom frame
  x_ += cos_yaw * dx - sin_yaw * dy;
  y_ += sin_yaw * dx + cos_yaw * dy;
  yaw_ += dyaw;
}

void TwistPlugin::publishOdometry(const geometry_msgs::Twist& velocity, const common::Time& now)
{
  math::Quaternion rotation = math::Quaternion(0, 0, yaw_);

  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;
  odom_msg_.pose.pose.position.z = 0;

  odom_msg_.pose.pose.orientation.x = rotation.x;
  odom_msg_.pose.pose.orientation.y = rotation.y;
  odom_msg_.pose.pose.orientation.z = rotation.z;
  odom_msg_.pose.pose.orientation.w = rotation.w;

  odom_msg_.twist.twist = velocity;

  odom_msg_.header.stamp = ros::Time(now.Double());
  odometry_publisher_.publish(odom_msg_);
}

void TwistPlugin::FiniChild() {
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

}
