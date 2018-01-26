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
  std::string action_name = getParameterFromSDF(sdf, "action_name", std::string("lift"));
  std::string joint_name = getParameterFromSDF(sdf, "joint_name", std::string("lift_joint"));
  state_publish_rate_ = getParameterFromSDF(sdf, "joint_state_publish_rate", 50.0);

  // Now try to find the joint in the parent model (required)
  joint_ = model_->GetJoint(joint_name);
  if (!joint_ || !joint_->GetChild())
  {
    ROS_FATAL("Could not find joint with name %s in model.", joint_name.c_str());
    return;
  }
  joint_position_ = joint_->GetAngle(0).Radian();

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

  // For publishing joint state
  joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState>(action_name + "/joint_state", 1);
  joint_state_msg_.name.push_back(joint_->GetName());
  joint_state_msg_.position.push_back(0);
}

void SingleJointPositionLifterPlugin::goalCallback(SingleJointPositionActionServer::GoalHandle goal)
{
  double current_position = joint_position_;
  double desired_position = goal.getGoal()->position;
  double lower_limit = joint_->GetLowerLimit(0).Radian();
  double upper_limit = joint_->GetUpperLimit(0).Radian();
  ROS_INFO("Current position: %.2f; Desired position: %.2f", current_position, desired_position);

  if (desired_position >= lower_limit && desired_position <= upper_limit)
  {
    ROS_INFO("Received valid goal");
    goal.setAccepted();

    if (desired_position > current_position + 1e-3) // We are lifting
    {
      ROS_INFO("SingleJointPositionLifterPlugin: We are lifting");
      lift_model_ = getModelAboveUs();
      if (lift_model_)
      {
        lift_world_pose_relative_to_model_ = lift_model_->GetWorldPose() - model_->GetWorldPose();
      }
    }
    else if (desired_position < current_position - 1e-3) // dropping (if we are going down, we always drop the model we are lifting)
    {
      ROS_INFO("SingleJointPositionLifterPlugin: We are dropping");
      if (lift_model_)
      {
        ROS_INFO("SingleJointPositionLifterPlugin: Dropping model %s", lift_model_->GetName().c_str());
        lift_model_ = 0;
      }
      else
      {
        ROS_WARN("SingleJointPositionLifterPlugin: No lift model attached, doing nothing.");
      }
    }

    ROS_DEBUG("SingleJointPositionLifterPlugin: Set joint to %.2f", desired_position);
    joint_position_ = desired_position;

    goal.setSucceeded();
  }
  else
  {
    std::string msg = "Incoming goal " + std::to_string(desired_position) + " out of bound [" +
        std::to_string(lower_limit) + ", " + std::to_string(upper_limit) + "]";
    ROS_ERROR_STREAM("Received invalid goal: " << msg);
    goal.setRejected(control_msgs::SingleJointPositionResult(), msg);
  }
}

physics::ModelPtr SingleJointPositionLifterPlugin::getModelAboveUs()
{
  model_->GetWorld()->GetPhysicsEngine()->InitForThread();

  physics::RayShapePtr rayShape = boost::dynamic_pointer_cast<physics::RayShape>(
      model_->GetWorld()->GetPhysicsEngine()->CreateShape("ray", physics::CollisionPtr()));

  math::Box box = model_->GetBoundingBox();
  math::Vector3 start = model_->GetWorldPose().pos;
  math::Vector3 end = start;
  start.z = box.max.z + 0.00001;
  end.z += 1e3;

  std::string lift_entity_name;
  double nearest_distance;
  rayShape->SetPoints(start, end);

  ROS_INFO_STREAM("Shooting ray from " << start << " to " << end);
  rayShape->GetIntersection(nearest_distance, lift_entity_name);

  physics::EntityPtr e = model_->GetWorld()->GetEntity(lift_entity_name);

  if (e) 
  {
    ROS_INFO("Found entity: %s", e->GetName().c_str());
  }

  if (e && e->GetParentModel() && e->GetParentModel()->GetName() != model_->GetName())
  {
    ROS_INFO("getModelAboveUs(): model %s", e->GetParentModel()->GetName().c_str());
    return e->GetParentModel();
  }
  else
  {
    ROS_WARN("getModelAboveUs(): no model found");
    return 0;
  }
}

void SingleJointPositionLifterPlugin::UpdateChild()
{  
  common::Time now = model_->GetWorld()->GetSimTime();
  if (state_publish_rate_ > 0.0) {
    double seconds_since_last_publish = (now - common::Time(joint_state_msg_.header.stamp.toSec())).Double();
    if (seconds_since_last_publish > (1.0 / state_publish_rate_)) {
      joint_state_msg_.header.stamp = ros::Time(now.Double());
      joint_state_msg_.position[0] = joint_position_; // Constructor allocates mem
      joint_state_publisher_.publish(joint_state_msg_);
    }
  }

  if (lift_model_)
  {
    math::Pose model_pose = model_->GetWorldPose();
    lift_model_->SetWorldPose(lift_world_pose_relative_to_model_ + model_pose);
  }
}

void SingleJointPositionLifterPlugin::FiniChild()
{
  rosnode_->shutdown();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SingleJointPositionLifterPlugin)

}
