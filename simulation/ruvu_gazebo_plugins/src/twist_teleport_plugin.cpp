#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "util.h"

namespace gazebo
{

//!
//! \brief The TwistTeleportPlugin class teleports an entity in the scene base on a Twist command.
//!
//! This plugin assumes that the physics are disabled and assumes infinite acceleration.
//!
class TwistTeleportPlugin : public ModelPlugin
{

public:

  //!
  //! \brief Load Register update function
  //! \param ptr Pointer to the world
  //!
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    //! Store the model so that we can use it later
    model_ = model;

    //! Parse SDF for parameters
    std::string command_topic = getParameterFromSDF(sdf, "commandTopic", std::string("cmd_vel"));
    cmd_timeout_ = getParameterFromSDF(sdf, "commandTimeout", 0.5);
    std::string odom_topic = getParameterFromSDF(sdf, "odometryTopic",  std::string("odom"));
    odom_msg_.header.frame_id = getParameterFromSDF(sdf, "odometryFrame",  std::string("odom"));
    odometry_rate_ = getParameterFromSDF(sdf, "odometryRate", 20.0);
    odom_msg_.child_frame_id = getParameterFromSDF(sdf, "robotFrame",  std::string("base_link"));

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
    rosnode_.reset(new ros::NodeHandle());

    // subscribe to the command topic
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
          command_topic, 1, boost::bind(&TwistTeleportPlugin::twistCallback, this, _1), ros::VoidPtr(), &queue_);
    twist_subscriber_ = rosnode_->subscribe(so);
    callback_queue_thread_ = boost::thread(boost::bind(&TwistTeleportPlugin::QueueThread, this));

    // Initialize odometry publisher
    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odom_topic, 1);

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TwistTeleportPlugin::UpdateChild, this));
  }

protected:

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
  bool alive_; //! To ensure we shutdown the thread when the plugin dies
  ros::CallbackQueue queue_; // The ROS callback queue
  boost::thread callback_queue_thread_; // The ROS message handler thread
  void QueueThread() {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  //!
  //! \brief twistCmdCallback Callback of the twist cmd
  //! \param cmd_msg Twist command
  //!
  geometry_msgs::TwistConstPtr cmd_msg_; // Last received cmd_msg
  common::Time last_cmd_time_; // Time last received cmd_msg
  double cmd_timeout_; // Base controller timeout
  ros::Subscriber twist_subscriber_; // The ROS subcriber on the Twist msg
  void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_msg_ = cmd_msg;
    last_cmd_time_ = model_->GetWorld()->GetSimTime();
  }

  //!
  //! \brief getCurrentVelocity Get the current velocity based on the input command and the last received time
  //!
  //! If the command is older than the command timeout, we will return zero
  //!
  //! \param now The current time
  //! \return Return the current velocity
  //!
  geometry_msgs::Twist getCurrentVelocity(const common::Time& now) {
    geometry_msgs::Twist velocity;
    if (cmd_msg_ && (now - last_cmd_time_).Double() < cmd_timeout_) {
      velocity = *cmd_msg_;
    }
    return velocity;
  }

  //!
  //! \brief UpdateChild Called on every tick of the simulation; updates the position and velocity of the parent
  //!
  //! Also publishes the odometry if we exceed the step time of the odom
  //!
  common::Time last_update_time_; // Keep track of the last update time in order to calculate the dt
  void UpdateChild()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    common::Time now = model_->GetWorld()->GetSimTime();
    common::Time dt = now - last_update_time_;
    math::Pose pose = model_->GetWorldPose();
    geometry_msgs::Twist vel = getCurrentVelocity(now);

    // Calculate the velocities based on last pose
    math::Vector3 world_linear_velocity(vel.linear.x * cosf(pose.rot.GetYaw()) - vel.linear.y * sinf(pose.rot.GetYaw()),
                                        vel.linear.y * cosf(pose.rot.GetYaw()) + vel.linear.x * sinf(pose.rot.GetYaw()),
                                        0);
    math::Vector3 world_angular_velocity(0, 0, vel.angular.z);

    // Calculate the updated pose
    pose.pos += dt.Double() * world_linear_velocity;
    pose.rot.SetFromEuler(0, 0, pose.rot.GetYaw() + dt.Double() * vel.angular.z);

    // Update the model in gazebo
    model_->SetLinearVel(world_linear_velocity);
    model_->SetAngularVel(world_angular_velocity);
    model_->SetWorldPose(pose);

    if (odometry_rate_ > 0.0) {
      double seconds_since_last_update = (now - common::Time(odom_msg_.header.stamp.toSec())).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(pose, vel, now);
      }
    }

    last_update_time_ = now;
  }

  //!
  //! \brief publishOdometry Publish the odometry via ROS
  //! \param pose Gazebo pose
  //! \param velocity Current velocity
  //! \param now Current time
  //!
  double odometry_rate_; // Rate of the odometry publisher
  ros::Publisher odometry_publisher_; // Odometry publisher
  nav_msgs::Odometry odom_msg_; // The odom message to be published
  void publishOdometry(const math::Pose& pose, const geometry_msgs::Twist& velocity, const common::Time& now)
  {
    odom_msg_.pose.pose.position.x = pose.pos.x;
    odom_msg_.pose.pose.position.y = pose.pos.y;

    odom_msg_.pose.pose.orientation.x = pose.rot.x;
    odom_msg_.pose.pose.orientation.y = pose.rot.y;
    odom_msg_.pose.pose.orientation.z = pose.rot.z;
    odom_msg_.pose.pose.orientation.w = pose.rot.w;

    odom_msg_.twist.twist = velocity;

    odom_msg_.header.stamp = ros::Time(now.Double());
    odometry_publisher_.publish(odom_msg_);
  }

  //!
  //! \brief FiniChild Called on shutdown
  //!
  void FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN  (TwistTeleportPlugin)

}
