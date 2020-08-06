//
// Copyright (c) 2017 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./twist_plugin.hpp"

namespace gazebo
{
//!
//! \brief The TwistTeleportPlugin class teleports an entity in the scene base on a Twist command.
//!
//! This plugin assumes that the physics are disabled and assumes infinite acceleration.
//!
class TwistTeleportPlugin : public TwistPlugin
{
protected:
  void Update(const ignition::math::Pose3d& pose, const geometry_msgs::Twist& /*twist*/,
              const ignition::math::Vector3d& world_linear_velocity,
              const ignition::math::Vector3d& world_angular_velocity, double dt, physics::ModelPtr model)
  {
    // Calculate the updated pose
    ignition::math::Pose3d updated_pose = pose;
    updated_pose.Pos() += dt * world_linear_velocity;

    // Only perform if we have a velocity vector
    if (world_angular_velocity.Length() > 0)
    {
      updated_pose.Rot() = ignition::math::Quaterniond(world_angular_velocity, world_angular_velocity.Length() * dt) *
                           updated_pose.Rot();
    }

    // Update the model in gazebo
    model->SetLinearVel(world_linear_velocity);
    model->SetAngularVel(world_angular_velocity);
    model->SetWorldPose(updated_pose);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TwistTeleportPlugin)
}  // namespace gazebo
