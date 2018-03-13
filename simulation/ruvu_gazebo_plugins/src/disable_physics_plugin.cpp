//
// Copyright (c) 2017 RUVU Robotics
//
// @author Rein Appeldoorn
//

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
//!
//! \brief The DisablePhysicsPlugin class disables the physics engine when it is on
//!
class DisablePhysicsPlugin : public WorldPlugin
{
public:
  //!
  //! \brief Load Register update function
  //! \param world Pointer to the world
  //!
  void Load(physics::WorldPtr world, sdf::ElementPtr /*_sdf*/)
  {
    world_ = world;
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DisablePhysicsPlugin::OnUpdate, this, _1));
  }

protected:
  //!
  //! \brief _world Pointer to physics world instance
  //!
  physics::WorldPtr world_;

  //!
  //! \brief _update_connection Connection that triggers the callback function
  //!
  event::ConnectionPtr update_connection_;

  //!
  //! \brief OnUpdate Update function triggered by event connection
  //!
  void OnUpdate(const gazebo::common::UpdateInfo& /*info*/)
  {
    if (world_->GetEnablePhysicsEngine())
      world_->EnablePhysicsEngine(false);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(DisablePhysicsPlugin)
}  // namespace gazebo
