#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace gazebo
{

//!
//! \brief The TwistPlugin class that alters a model based on a twist command
//!
class TwistPlugin : public ModelPlugin
{

public:

  //!
  //! \brief Load Register update function
  //! \param ptr Pointer to the world
  //!
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

protected:

  //!
  //! \brief Update Update function that needs to be implemented by the child class
  //! \param pose The current pose of the model
  //! \param twist The current twist of the model (derived from the input)
  //! \param world_linear_velocity The current world linear velocity of the model (derived from the input)
  //! \param world_angular_velocity The current world angular velocity of the model (derived from the input)
  //! \param dt Delta time since last call
  //! \param model Pointer to the model, can be used to update the model
  //!
  virtual void Update(const math::Pose& pose, const geometry_msgs::Twist& twist,
                      const math::Vector3& world_linear_velocity, const math::Vector3& world_angular_velocity,
                      double dt, physics::ModelPtr model) = 0;

private:

  //!
  //! \brief model_ Pointer to the parent model
  //!
  physics::ModelPtr model_;

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
  //! \brief QueueThread Custom callback queue thread in order to not block the physics engine
  //!
  bool alive_; //! To ensure we shutdown the thread when the plugin dies
  ros::CallbackQueue queue_; // The ROS callback queue
  boost::thread callback_queue_thread_; // The ROS message handler thread
  void QueueThread();

  //!
  //! \brief twistCmdCallback Callback of the twist cmd
  //! \param cmd_msg Twist command
  //!
  geometry_msgs::TwistConstPtr cmd_msg_; // Last received cmd_msg
  common::Time last_cmd_time_; // Time last received cmd_msg
  double cmd_timeout_; // Base controller timeout
  ros::Subscriber twist_subscriber_; // The ROS subcriber on the Twist msg
  void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  //!
  //! \brief getCurrentVelocity Get the current velocity based on the input command and the last received time
  //!
  //! If the command is older than the command timeout, we will return zero
  //!
  //! \param now The current time
  //! \return Return the current velocity
  //!
  geometry_msgs::Twist getCurrentVelocity(const common::Time& now);

  //!
  //! \brief UpdateChild Called on every tick of the simulation; updates the position and velocity of the parent
  //!
  //! Also publishes the odometry if we exceed the step time of the odom
  //!
  common::Time last_update_time_; // Keep track of the last update time in order to calculate the dt
  void UpdateChild();

  //!
  //! \brief publishOdometry Publish the odometry via ROS
  //! \param pose Gazebo pose
  //! \param velocity Current velocity
  //! \param now Current time
  //!
  double odometry_rate_; // Rate of the odometry publisher
  ros::Publisher odometry_publisher_; // Odometry publisher
  nav_msgs::Odometry odom_msg_; // The odom message to be published
  void publishOdometry(const math::Pose& pose, const geometry_msgs::Twist& velocity, const common::Time& now);

  //!
  //! \brief FiniChild Called on shutdown
  //!
  void FiniChild();

};

}
