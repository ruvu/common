//
// Copyright (c) 2017 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include "./twist_plugin.h"

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
  void Update(const math::Pose& pose, const geometry_msgs::Twist& /*twist*/, const math::Vector3& world_linear_velocity,
              const math::Vector3& world_angular_velocity, double dt, physics::ModelPtr model)
  {
    // Calculate the updated pose
    math::Pose updated_pose = pose;
    updated_pose.pos += dt * world_linear_velocity;

    // Only perform if we have a velocity vector
    if (world_angular_velocity.GetLength() > 0)
    {
      updated_pose.rot =
          math::Quaternion(world_angular_velocity, world_angular_velocity.GetLength() * dt) * updated_pose.rot;
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
