//
// Copyright (c) 2020 RUVU Robotics
//
// @author Paul Verhoeckx
//

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include "./accerion_triton_plugin.hpp"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(AccerionTritonPlugin);

////////////////////////////////////////////////////////////////////////////////
// Constructor
AccerionTritonPlugin::AccerionTritonPlugin()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
AccerionTritonPlugin::~AccerionTritonPlugin()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->triton_queue_.clear();
  this->triton_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void AccerionTritonPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("sensorFrameId"))
  {
    ROS_FATAL_NAMED("triton", "triton plugin missing <sensorFrameId>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("sensorFrameId")->Get<std::string>();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("triton", "accerion_triton_plugin error: sensor_frame_id: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("outputFrameId"))
  {
    ROS_FATAL_NAMED("triton", "triton plugin missing <outputFrameId>, cannot proceed");
    return;
  }
  else
    this->output_link_name_ = _sdf->GetElement("outputFrameId")->Get<std::string>();

  this->output_link_ = _parent->GetLink(this->output_link_name_);
  if (!this->output_link_)
  {
    ROS_FATAL_NAMED("triton", "accerion_triton_plugin error: outputFrameId: %s does not exist\n",
      this->output_link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("triton", "triton plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("parentFrameId"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <parentFrameId>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("parentFrameId")->Get<std::string>();

  if (!_sdf->HasElement("xyzOffset"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  }
  else
    this->offset_.Pos() = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();

  if (!_sdf->HasElement("rpyOffset"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
  }
  else
    this->offset_.Rot() = ignition::math::Quaterniond(_sdf->GetElement("rpyOffset")->Get<ignition::math::Vector3d>());

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

  if (!_sdf->HasElement("maxPublishRate"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <maxPublishRate>, defaults to 0.0 (as fast as possible)");
    this->max_publish_rate_ = 0;
  }
  else
  {
    this->max_publish_rate_ = _sdf->GetElement("maxPublishRate")->Get<double>();
  }

  if (!_sdf->HasElement("GridResolutionX"))
  {
    ROS_FATAL_NAMED("triton", "triton plugin missing <GridResolutionX>, cannot proceed");
    return;
  }
  else
  {
    this->grid_resolution_x_ = _sdf->GetElement("GridResolutionX")->Get<double>();
  }

  if (!_sdf->HasElement("GridResolutionY"))
  {
    ROS_FATAL_NAMED("triton", "triton plugin missing <GridResolutionY>, cannot proceed");
    return;
  }
  else
  {
    this->grid_resolution_y_ = _sdf->GetElement("GridResolutionY")->Get<double>();
  }

  if (!_sdf->HasElement("GridOffsetX"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <GridOffsetX>, defaults to 0");
    this->grid_offset_x_ = 0.0;
  }
  else
  {
    this->grid_offset_x_ = _sdf->GetElement("GridOffsetX")->Get<double>();
  }

  if (!_sdf->HasElement("GridOffsetY"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <GridOffsetY>, defaults to 0");
    this->grid_offset_y_ = 0.0;
  }
  else
  {
    this->grid_offset_y_ = _sdf->GetElement("GridOffsetY")->Get<double>();
  }

  if (!_sdf->HasElement("xyThreshold"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <xyThreshold>, defaults to 0.01");
    this->threshold_xy_ = 0.01;
  }
  else
    this->threshold_xy_ = _sdf->GetElement("xyThreshold")->Get<double>();

  if (!_sdf->HasElement("publishDelay"))
  {
    ROS_DEBUG_NAMED("triton", "triton plugin missing <publishDelay>, defaults to 0.0");
    this->publish_delay_ = 0.0;
  }
  else
    this->publish_delay_ = _sdf->GetElement("publishDelay")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("triton", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->tf_frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<nav_msgs::Odometry>();
    this->pub_ =
      this->rosnode_->advertise<nav_msgs::Odometry>(this->topic_name_, 1);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_loop_time_ = this->world_->SimTime();
  this->last_publish_time_ = this->world_->SimTime();
#else
  this->last_loop_time_ = this->world_->GetSimTime();
  this->last_publish_time_ = this->world_->GetSimTime();
#endif

  // if frameName specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (this->frame_name_ != "/world" &&
      this->frame_name_ != "world" &&
      this->frame_name_ != "/map" &&
      this->frame_name_ != "map")
  {
    this->reference_link_ = this->model_->GetLink(this->frame_name_);
    if (!this->reference_link_)
    {
      ROS_ERROR_NAMED("triton", "accerion_triton_plugin: frameName: %s does not exist, will"
                " not publish pose\n", this->frame_name_.c_str());
      return;
    }
  }

  // start custom queue for triton
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&AccerionTritonPlugin::TritonQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AccerionTritonPlugin::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void AccerionTritonPlugin::UpdateChild()
{
  if (!this->link_)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  // Send msgs with sufficient delay
  while (!this->delay_queue_.empty())
  {
    if (cur_time.Double() < this->delay_queue_.front().header.stamp.toSec() + this->publish_delay_)
      break;
    this->pub_Queue->push(this->delay_queue_.front(), this->pub_);
    this->delay_queue_.pop();
  }

  if (cur_time <= this->last_loop_time_)
  {
      ROS_WARN_NAMED("triton", "Negative or zero update time difference detected.");
      return;
  }

  // Update loop time
  this->last_loop_time_ = cur_time;
  // Rate control
  if (this->max_publish_rate_ > 0 &&
      (cur_time-this->last_publish_time_).Double() < (1.0/this->max_publish_rate_))
      return;

  if (this->pub_.getNumSubscribers() > 0)
  {
    this->lock.lock();
    if (this->topic_name_ != "")
    {
      // Get sensor pose
      this->sensor_pose_msg_ = this->GetLinkPose(this->link_name_, this->link_, cur_time);

      // check if sensor would match image
      if (!this->CheckPatternDetection(this->sensor_pose_msg_))
      {
        this->lock.unlock();
        return;
      }

      // Get output pose
      if (this->link_name_ == this->output_link_name_)
      {
        this->output_pose_msg_ = this->sensor_pose_msg_;
      }
      else
      {
        this->output_pose_msg_ = this->GetLinkPose(this->output_link_name_, this->output_link_, cur_time);
      }

      // Add msg to delay queue
      this->delay_queue_.push(this->output_pose_msg_);
    }

    this->lock.unlock();

    // save last time stamp
    this->last_publish_time_ = cur_time;
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double AccerionTritonPlugin::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void AccerionTritonPlugin::TritonQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->triton_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Check if sensor is triggered
bool AccerionTritonPlugin::CheckPatternDetection(nav_msgs::Odometry odom_msg)
{
  bool match_x = (this->grid_resolution_x_ > 0 &&
    std::abs(std::fmod(odom_msg.pose.pose.position.x - this->grid_offset_x_, this->grid_resolution_x_))
    <= this->threshold_xy_);
  bool match_y = (this->grid_resolution_y_ > 0 &&
    std::abs(std::fmod(odom_msg.pose.pose.position.y - this->grid_offset_y_, this->grid_resolution_y_))
    <= this->threshold_xy_);

  if (this->grid_resolution_x_ == 0 || this->grid_resolution_y_ == 0)
    return true;
  if (this->grid_resolution_x_ < 0 && match_y)
    return true;
  if (this->grid_resolution_y_ < 0 && match_x)
    return true;
  if ((this->grid_resolution_x_ > 0 && this->grid_resolution_y_ > 0) && (match_x || match_y))
    return true;
  return false;
}

// Get pose_msg from link
nav_msgs::Odometry AccerionTritonPlugin::GetLinkPose(std::string link_name, physics::LinkPtr link,
common::Time cur_time)
{
// copy data into pose message
    nav_msgs::Odometry pose_msg_;
    pose_msg_.header.frame_id = this->tf_frame_name_;
    pose_msg_.header.stamp.sec = cur_time.sec;
    pose_msg_.header.stamp.nsec = cur_time.nsec;
    pose_msg_.child_frame_id = link_name;

    ignition::math::Pose3d pose, frame_pose;
    ignition::math::Vector3d frame_vpos;
    ignition::math::Vector3d frame_veul;

    // get inertial Rates
    // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d vpos = link->WorldLinearVel();
    ignition::math::Vector3d veul = link->WorldAngularVel();
    pose = link->WorldPose();
#else
    ignition::math::Vector3d vpos = link->GetWorldLinearVel().Ign();
    ignition::math::Vector3d veul = link->GetWorldAngularVel().Ign();
    pose = link->GetWorldPose().Ign();
#endif
    // Apply Reference Frame
    if (this->reference_link_)
    {
      // convert to relative pose, rates
#if GAZEBO_MAJOR_VERSION >= 8
        frame_pose = this->reference_link_->WorldPose();
        frame_vpos = this->reference_link_->WorldLinearVel();
        frame_veul = this->reference_link_->WorldAngularVel();
  #else
        frame_pose = this->reference_link_->GetWorldPose().Ign();
        frame_vpos = this->reference_link_->GetWorldLinearVel().Ign();
        frame_veul = this->reference_link_->GetWorldAngularVel().Ign();
  #endif
        pose.Pos() = pose.Pos() - frame_pose.Pos();
        pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
        pose.Rot() *= frame_pose.Rot().Inverse();
        vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
        veul = frame_pose.Rot().RotateVector(veul - frame_veul);
    }

    // Apply Constant Offsets
    // apply xyz offsets and get position and rotation components
    pose.Pos() = pose.Pos() + this->offset_.Pos();
    // apply rpy offsets
    pose.Rot() = this->offset_.Rot()*pose.Rot();
    pose.Rot().Normalize();

    // Fill out messages
    pose_msg_.pose.pose.position.x    = pose.Pos().X();
    pose_msg_.pose.pose.position.y    = pose.Pos().Y();
    pose_msg_.pose.pose.position.z    = pose.Pos().Z();

    pose_msg_.pose.pose.orientation.x = pose.Rot().X();
    pose_msg_.pose.pose.orientation.y = pose.Rot().Y();
    pose_msg_.pose.pose.orientation.z = pose.Rot().Z();
    pose_msg_.pose.pose.orientation.w = pose.Rot().W();

    pose_msg_.twist.twist.linear.x  = vpos.X() + this->GaussianKernel(0, this->gaussian_noise_);
    pose_msg_.twist.twist.linear.y  = vpos.Y() + this->GaussianKernel(0, this->gaussian_noise_);
    pose_msg_.twist.twist.linear.z  = vpos.Z() + this->GaussianKernel(0, this->gaussian_noise_);
    // pass euler angular rates
    pose_msg_.twist.twist.angular.x = veul.X() + this->GaussianKernel(0, this->gaussian_noise_);
    pose_msg_.twist.twist.angular.y = veul.Y() + this->GaussianKernel(0, this->gaussian_noise_);
    pose_msg_.twist.twist.angular.z = veul.Z() + this->GaussianKernel(0, this->gaussian_noise_);

    // fill in covariance matrix
    /// @todo: let user set separate linear and angular covariance values.
    double gn2 = this->gaussian_noise_*this->gaussian_noise_;
    pose_msg_.pose.covariance[0] = gn2;
    pose_msg_.pose.covariance[7] = gn2;
    pose_msg_.pose.covariance[14] = gn2;
    pose_msg_.pose.covariance[21] = gn2;
    pose_msg_.pose.covariance[28] = gn2;
    pose_msg_.pose.covariance[35] = gn2;

    pose_msg_.twist.covariance[0] = gn2;
    pose_msg_.twist.covariance[7] = gn2;
    pose_msg_.twist.covariance[14] = gn2;
    pose_msg_.twist.covariance[21] = gn2;
    pose_msg_.twist.covariance[28] = gn2;
    pose_msg_.twist.covariance[35] = gn2;
    return pose_msg_;
}
}  // namespace gazebo
