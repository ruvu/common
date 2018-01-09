#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>

#include <control_msgs/SingleJointPositionAction.h>
#include <actionlib/server/action_server.h>

namespace gazebo
{

//!
//! \brief The SingleJointPositionLifterPlugin class that exposes an actionlib interface to move a lift joint
//!
//! And attach a model to the lift body
//!
class SingleJointPositionLifterPlugin : public ModelPlugin
{

public:

  //!
  //! \brief Load Register update function
  //! \param ptr Pointer to the world
  //!
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  typedef actionlib::ActionServer<control_msgs::SingleJointPositionAction> SingleJointPositionActionServer;

  //!
  //! \brief model_ Pointer to the parent model
  //!
  physics::ModelPtr model_;

  //!
  //! \brief lift_model_ Pointer to the model we are lifting
  //!
  physics::ModelPtr lift_model_;
  math::Pose lift_world_pose_relative_to_joint_child_link_;

  //!
  //! \brief joint_ Pointer to the joint that needs to be actuated
  //!
  physics::JointPtr joint_;

  //!
  //! \brief joint_controller Pointer to the joint_controller used for actuation
  //!
  physics::JointControllerPtr joint_controller_;

  //!
  //! \brief joint_child_link_ Pointer to joint child that is being moved
  //!
  physics::LinkPtr joint_child_link_;

  //!
  //! \brief update_connection_ Connection that triggers the callback function of the model
  //!
  event::ConnectionPtr update_connection_;

  //!
  //! \brief mutex_ Mutex to make sure the physics thread and the ROS callback queue thread do not collide
  //!
  std::mutex mutex_;

  //!
  //! \brief rosnode_ Handle to the ROS node
  //!
  std::shared_ptr<ros::NodeHandle> rosnode_;

  //!
  //! \brief goalCallback Action goal callback
  //! \param goal Incoming action goal
  //!
  std::shared_ptr<SingleJointPositionActionServer> action_server_;
  SingleJointPositionActionServer::GoalHandle action_goal_;
  void goalCallback(SingleJointPositionActionServer::GoalHandle goal);

  //!
  //! \brief getLiftEntity
  //! \param d_position
  //! \return
  //!
  physics::ModelPtr getLiftModel(double d_position, double& travel_distance);

  //!
  //! \brief UpdateChild Called on every tick of the simulation; updates the position and velocity of the parent
  //!
  void UpdateChild();

  //!
  //! \brief FiniChild Called on shutdown
  //!
  void FiniChild();

};

}
