//
// Copyright (c) 2020 RUVU Robotics
//
// @author Paul Verhoeckx
//

#pragma once

#include <gazebo/common/Plugin.hh>

#include <string>
#include <queue>
#include <assert.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
class AccerionTritonPlugin : public ModelPlugin
{
  /// \brief Constructor
  public: AccerionTritonPlugin();
  /// \brief Destructor
  public: virtual ~AccerionTritonPlugin();
  /// \brief Load the controller
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  /// \brief Update the controller
  protected: virtual void UpdateChild();
  private: physics::WorldPtr world_;
  private: physics::ModelPtr model_;
  /// \brief The parent Model
  private: physics::LinkPtr link_;
  private: physics::LinkPtr output_link_;
  /// \brief The body of the frame to display pose, twist
  private: physics::LinkPtr reference_link_;
  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;
  private: PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;
  /// \brief ros message
  private: nav_msgs::Odometry sensor_pose_msg_;
  private: nav_msgs::Odometry output_pose_msg_;
  /// \brief store bodyname
  private: std::string link_name_;
  private: std::string output_link_name_;
  /// \brief topic name
  private: std::string topic_name_;
  /// \brief frame transform name, should match link name
  /// FIXME: extract link name directly?
  private: std::string frame_name_;
  private: std::string tf_frame_name_;
  /// \brief allow specifying constant xyz and rpy offsets
  private: ignition::math::Pose3d offset_;
  /// \brief mutex to lock access to fields used in message callbacks
  private: boost::mutex lock;
  /// \brief save last_time
  private: common::Time last_loop_time_;
  private: common::Time last_publish_time_;
  // rate control
  private: double max_publish_rate_;
  // triton match parameters
  private: double grid_resolution_x_;
  private: double grid_resolution_y_;
  private: double grid_offset_x_;
  private: double grid_offset_y_;
  private: double threshold_xy_;
  // sensor delay
  private: double publish_delay_;
  private: std::queue<nav_msgs::Odometry> delay_queue_;
  /// \brief Gaussian noise
  private: double gaussian_noise_;
  /// \brief Gaussian noise generator
  private: double GaussianKernel(double mu, double sigma);
  /// \brief Pattern recognition checker
  private: bool CheckPatternDetection(nav_msgs::Odometry odom_msg);
  /// \brief pose msg creator
  private: nav_msgs::Odometry GetLinkPose(std::string link_name, physics::LinkPtr link, common::Time cur_time);
  /// \brief for setting ROS name space
  private: std::string robot_namespace_;
  private: ros::CallbackQueue triton_queue_;
  private: void TritonQueueThread();
  private: boost::thread callback_queue_thread_;
  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
  private: unsigned int seed;
  // ros publish multi queue, prevents publish() blocking
  private: PubMultiQueue pmq;
};
}  // namespace gazebo

