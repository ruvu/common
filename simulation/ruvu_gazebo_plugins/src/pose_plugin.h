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
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <math.h>

namespace gazebo
{
//!
//! \brief The PosePlugin class that alters a model based on a pose command
//!
class PosePlugin : public ModelPlugin
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
  virtual void Update(const math::Pose& pose, physics::ModelPtr model) = 0;

  //!
  //! \brief odom_pose_ The state of the robot according to odometry
  //!
  math::Pose odom_pose_;

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
  common::Time last_cmd_time_;            // Time last received cmd_msg
  double cmd_timeout_;                    // Base controller timeout
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
  math::Pose getCurrentPose(const common::Time& now);

  //!
  //! \brief UpdateChild Called on every tick of the simulation; updates the position and velocity of the parent
  //!
  //! Also publishes the odometry if we exceed the step time of the odom
  //!
  common::Time last_update_time_;  // Keep track of the last update time in order to calculate the dt
  void UpdateChild();

  //!
  //! \brief updateOdometryState Update the odometry state variables (x, y, yaw)
  //! \param velocity Current (command) velocity
  //! \param dt Time delta since last update
  //!
  virtual void updateOdometryPose(const math::Pose& pose);

  //!
  //! \brief publishOdometry Publish the odometry via ROS
  //! \param velocity Current velocity
  //! \param now Current time
  //!
  bool publish_tf_;
  double odometry_rate_;               // Rate of the odometry publisher
  ros::Publisher odometry_publisher_;  // Odometry publisher
  nav_msgs::Odometry odom_msg_;        // The odom message to be published
  geometry_msgs::TransformStamped transform_stamped_;
  tf2_ros::TransformBroadcaster odom_broadcaster_;
  std::vector<double> pose_covariance_;
  std::vector<double> velocity_covariance_;
  void publishOdometry(const geometry_msgs::Twist& velocity, const common::Time& now);

  //!
  //! \brief FiniChild Called on shutdown
  //!
  void FiniChild();
};
}  // namespace gazebo
