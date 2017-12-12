#include "twist_plugin.h"

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

  void Update(const math::Pose& /*pose*/, const geometry_msgs::Twist& /*twist*/,
              const math::Vector3& world_linear_velocity, const math::Vector3& world_angular_velocity,
              double /*dt*/, physics::ModelPtr model)
  {
    math::Vector3 linear_velocity = model->GetWorldLinearVel();
    linear_velocity.x = world_linear_velocity.x; // constrain
    linear_velocity.y = world_linear_velocity.y; // z

    math::Vector3 angular_velocity = model->GetWorldAngularVel();
    angular_velocity.z = world_angular_velocity.z; // constrain roll and pitch

    // Update the model in gazebo
    model->SetLinearVel(linear_velocity);
    model->SetAngularVel(angular_velocity);
  }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TwistMovePlugin)

}
