//
// Copyright (c) 2017 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./twist_plugin.h"

namespace gazebo
{
//!
//! \brief The TwistMovePlugin class moves an entity in the scene based on a Twist command.
//!
//! This plugin requires physics since it is applying a velocity on the attached model. This velocity should evolve
//! using a physics engine and assumes infinite acceleration.
//!
class TwistMovePlugin : public TwistPlugin
{
protected:
  double yaw_ = 0;

  void updateOdometryPose(const ignition::math::Pose3d& pose, const geometry_msgs::Twist& velocity, const common::Time& dt)
  {
    double sin_yaw = sin(yaw_);
    double cos_yaw = cos(yaw_);

    // Calculate diffs in body fixed frame
    double dx = velocity.linear.x * dt.Double();
    double dy = velocity.linear.y * dt.Double();
    double dyaw = velocity.angular.z * dt.Double();

    // Convert it to odom frame
    odom_pose_.pos.x += cos_yaw * dx - sin_yaw * dy;
    odom_pose_.pos.y += sin_yaw * dx + cos_yaw * dy;
    yaw_ += dyaw;
    odom_pose_.rot = math::Quaternion(0, 0, yaw_);
  }

  void Update(const ignition::math::Pose3d& /*pose*/, const geometry_msgs::Twist& /*twist*/,
              const math::Vector3& world_linear_velocity, const math::Vector3& world_angular_velocity, double /*dt*/,
              physics::ModelPtr model)
  {
    math::Vector3 linear_velocity = model->GetWorldLinearVel();
    linear_velocity.x = world_linear_velocity.x;  // constrain
    linear_velocity.y = world_linear_velocity.y;  // z

    math::Vector3 angular_velocity = model->GetWorldAngularVel();
    angular_velocity.z = world_angular_velocity.z;  // constrain roll and pitch

    // Update the model in gazebo
    model->SetLinearVel(linear_velocity);
    model->SetAngularVel(angular_velocity);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TwistMovePlugin)
}  // namespace gazebo
