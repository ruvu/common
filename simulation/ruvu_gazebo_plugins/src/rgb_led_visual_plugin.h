#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/subscriber.h>
#include <ros/ros.h>

#include <ruvu_msgs/ColorState.h>
#include <gazebo/msgs/visual.pb.h>

#include <gazebo/transport/Node.hh>

namespace gazebo
{

class RGBLedPlugin : public ModelPlugin
{

public:

  //!
  //! \brief Load Register update function
  //! \param ptr Pointer to the world
  //!
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  //!
  //! \brief model_ Pointer to the parent model
  //!
  physics::ModelPtr model_;
  std::string link_name_;

  //!
  //! \brief rosnode_ Handle to the ROS node
  //!
  std::shared_ptr<ros::NodeHandle> ros_node_;
  transport::NodePtr gazebo_node_;

  ros::Subscriber ros_color_state_sub_;
  ros::Publisher ros_color_state_pub_;
  ruvu_msgs::ColorState color_state_;
  transport::PublisherPtr gazebo_visual_pub_;

  void colorCB(const ruvu_msgs::ColorStateConstPtr& msg);

};

}
