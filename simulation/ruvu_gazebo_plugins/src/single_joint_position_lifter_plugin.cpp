#include "single_joint_position_lifter_plugin.h"
#include "util.h"

namespace gazebo
{

void SingleJointPositionLifterPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  //! Store the model so that we can use it later
  model_ = model;

  //! Parse SDF for parameters
  std::string robot_namespace = getParameterFromSDF(sdf, "robotNamespace", std::string(""));
  std::string tf_prefix = robotNamespaceToTFPrefix(robot_namespace);
  std::string action_name = getParameterFromSDF(sdf, "action_name", std::string("lifter"));
  std::string joint_name = getParameterFromSDF(sdf, "joint_name", std::string("lift_joint"));

  // Now try to find the joint in the parent model (required)
  joint_ = model_->GetJoint(joint_name);
  if (!joint_)
  {
    ROS_FATAL("Could not find joint with name %s in model.", joint_name.c_str());
    return;
  }
  joint_child_link_ = joint_->GetChild();
  ROS_INFO("Found joint %s with child link %s", joint_name.c_str(), joint_child_link_->GetName().c_str());
  joint_controller_.reset(new physics::JointController(model));

  // Ensure that ROS has been initialized (required for using ROS COM)
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("single_joing_position_lifter", "SingleJointPositionLifterPlugin"
      << ": A ROS node for Gazebo has not been initialized, "
      << "unable to load plugin. Load the Gazebo system plugin "
      << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Setup the ROS connection
  rosnode_.reset(new ros::NodeHandle(robot_namespace));
  action_server_.reset(
        new SingleJointPositionActionServer(*rosnode_, action_name,
                                            boost::bind(&SingleJointPositionLifterPlugin::goalCallback, this, _1),
                                            false)); // No autostart
  action_server_->start();

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&SingleJointPositionLifterPlugin::UpdateChild, this));
}

void SingleJointPositionLifterPlugin::goalCallback(SingleJointPositionActionServer::GoalHandle goal)
{
  double desired_position = goal.getGoal()->position;
  double lower_limit = joint_->GetLowerLimit(0).Radian();
  double upper_limit = joint_->GetUpperLimit(0).Radian();

  if (desired_position >= lower_limit && desired_position <= upper_limit)
  {
    // Update the goalhandle so we can used it in the UpdateChild hook
    action_goal_ = goal;
    action_goal_.setAccepted();
  }
  else
  {
    goal.setRejected(control_msgs::SingleJointPositionResult(), "Incoming goal " + std::to_string(desired_position) +
                     " out of bound [" + std::to_string(lower_limit) + ", " + std::to_string(upper_limit) + "]");
  }
}

physics::ModelPtr SingleJointPositionLifterPlugin::getLiftModel(double d_position, double& travel_distance)
{
  physics::RayShapePtr rayShape = boost::dynamic_pointer_cast<physics::RayShape>(
      model_->GetWorld()->GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));

  ignition::math::Box box = joint_child_link_->GetCollisionBoundingBox().Ign();
  ignition::math::Vector3d start = joint_child_link_->GetWorldPose().pos.Ign();
  ignition::math::Vector3d end = start;
  start.Z() = box.Max().Z() + 0.00001;
  end.Z() += d_position;
  std::string lift_entity_name;
  double nearest_distance;
  rayShape->SetPoints(start, end);
  rayShape->GetIntersection(nearest_distance, lift_entity_name);
  nearest_distance -= 0.00001;

  physics::EntityPtr e = model_->GetWorld()->GetEntity(lift_entity_name);

  if (e)
  {
    travel_distance = d_position - nearest_distance;
    ROS_INFO("Lifting model: %s - d_position: %.2f - nearest_distance: %.2f - travel distance: %.2f", e->GetParentModel()->GetName().c_str(), d_position, nearest_distance, travel_distance);
    return e->GetParentModel();
  }
  else
  {
    return nullptr;
  }
}

void SingleJointPositionLifterPlugin::UpdateChild()
{  
  if (action_goal_.isValid() && action_goal_.getGoal() != nullptr)
  {
    double current_position = joint_->GetAngle(0).Radian();
    double desired_position = action_goal_.getGoal()->position;

    if (desired_position > current_position) // We are lifting
    {
      double d_position = desired_position - current_position;
      double travel_distance;

      lift_model_ = getLiftModel(d_position, travel_distance);
      if (lift_model_)
      {
        lift_world_pose_relative_to_joint_child_link_ = lift_model_->GetWorldPose() - joint_child_link_->GetWorldPose();
      }
    }
    else // dropping (if we are going down, we always drop the model we are lifting)
    {
      if (lift_model_)
      {
        math::Pose lift_model_pose = lift_model_->GetWorldPose();
        std::cout << lift_model_pose << std::endl;

        double dz = lift_world_pose_relative_to_joint_child_link_.pos.z;
        ROS_INFO("Dropping model %s with dz: %.2f", lift_model_->GetName().c_str(), dz);

        lift_model_pose.pos.z -= dz;

        std::cout << lift_model_pose << std::endl;

        lift_model_->SetWorldPose(lift_model_pose);
        lift_model_->SetWorldPose(lift_model_pose);
        lift_model_->SetWorldPose(lift_model_pose);
        lift_model_ = 0;
      }
    }

    joint_controller_->SetJointPosition(joint_, action_goal_.getGoal()->position);
    joint_controller_->SetJointPosition(joint_, action_goal_.getGoal()->position);
    joint_controller_->SetJointPosition(joint_, action_goal_.getGoal()->position);

    ROS_DEBUG("SingleJointPositionLifterPlugin: Set joint to %.2f", action_goal_.getGoal()->position);
    action_goal_.setSucceeded();
    action_goal_ = SingleJointPositionActionServer::GoalHandle();
  }

  if (lift_model_)
  {
    math::Pose joint_child_link_pose = joint_child_link_->GetWorldPose();
    lift_model_->SetWorldPose(lift_world_pose_relative_to_joint_child_link_ + joint_child_link_pose);
  }
}

void SingleJointPositionLifterPlugin::FiniChild()
{
  rosnode_->shutdown();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SingleJointPositionLifterPlugin)

}
