//
// Copyright (c) 2018 RUVU Robotics
//
// @author Paul Verhoeckx
//

#include "./pose_plugin.h"
#include <tf/transform_datatypes.h>
#include <math.h>

namespace gazebo
{
//!
//! \brief The PoseTeleportPlugin class teleports an entity in the scene base on a Pose command.
//!
//! This plugin assumes that the physics are disabled and assumes infinite acceleration.
//!
class PoseTeleportPlugin : public PosePlugin
{
protected:
  void Update(const math::Pose& pose, physics::ModelPtr model)
  {
    math::Pose updated_pose = pose;

    // Convert to from camera axis (x up, y front, z left) to model axis (x front, y, left, z up)
    tf::Quaternion q(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double camera_roll, camera_pitch, camera_yaw;
    camera_roll = -roll + M_PI/2;
    camera_pitch = -yaw + M_PI/2;
    camera_yaw = -pitch;

    tf::Quaternion q_camera = tf::createQuaternionFromRPY(camera_roll, camera_pitch , camera_yaw);
    updated_pose.rot = math::Quaternion(q_camera[0], q_camera[1], q_camera[2], q_camera[3]);

    // Update the model in gazebo
    model->SetWorldPose(updated_pose);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(PoseTeleportPlugin)
}  // namespace gazebo
