#include "rgb_led_visual_plugin.h"
#include "util.h"

namespace gazebo
{

common::Color colorMsgToGazebo(const std_msgs::ColorRGBA& msg)
{
  return common::Color(msg.r, msg.g, msg.b, msg.a);
}

void setLinkVisualColor(physics::LinkPtr link, std::string name, transport::PublisherPtr pub, common::Color color)
{
  // Setup material
  gazebo::msgs::Material* material_msg = new gazebo::msgs::Material;
  gazebo::msgs::Color* colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(color));
  gazebo::msgs::Color* diffuseMsg = new gazebo::msgs::Color(*colorMsg);
  material_msg->set_allocated_ambient(colorMsg);
  material_msg->set_allocated_diffuse(diffuseMsg);

  gazebo::msgs::Visual visual_msg;
  visual_msg.set_allocated_material(material_msg);

  visual_msg.set_name(link->GetScopedName() + "::" + name);
  visual_msg.set_parent_name(link->GetScopedName());

  pub->Publish(visual_msg);
}

void RGBLedPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  //! Store the model so that we can use it later
  model_ = model;

  // Ensure that ROS has been initialized (required  for using ROS COM)
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("single_joing_position_lifter", "RGBLedPlugin"
      << ": A ROS node for Gazebo has not been initialized, "
      << "unable to load plugin. Load the Gazebo system plugin "
      << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  if (!sdf->HasElement("link")) {
    ROS_FATAL("Missing parameter <link> in RGBLedPlugin, please specify the gazebo link of the led array");
    return;
  }

  link_name_ = sdf->GetElement("link")->Get<std::string>();

  if (!sdf->HasElement("leds")) {
    ROS_FATAL("Missing parameter <leds> in RGBLedPlugin");
    return;
  }

  sdf::ElementPtr elem = sdf->GetElement("leds")->GetFirstElement();
  while (elem)
  {
    color_state_.name.push_back(elem->Get<std::string>());
    elem = elem->GetNextElement();
  }

  std_msgs::ColorRGBA default_color;
  if (!sdf->HasElement("defaultColor"))
  {
    ROS_WARN("No default color specified, defaults to RGBA(1, 1, 1, 0)");
    default_color.r = default_color.g = default_color.b = 1.0;

  }
  else
  {
    default_color.r = getParameterFromSDF(sdf->GetElement("defaultColor"), "r", 1.0);
    default_color.g = getParameterFromSDF(sdf->GetElement("defaultColor"), "g", 1.0);
    default_color.b = getParameterFromSDF(sdf->GetElement("defaultColor"), "b", 1.0);
    default_color.a = getParameterFromSDF(sdf->GetElement("defaultColor"), "a", 0.0);
  }

  for (const std::string& name : color_state_.name)
  {
    color_state_.color.push_back(default_color);
  }

  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();
  gazebo_visual_pub_ = gazebo_node_->Advertise<msgs::Visual>("~/visual");

  // Setup the ROS connection
  ros_node_.reset(new ros::NodeHandle(""));
  ros_color_state_sub_ = ros_node_->subscribe<ruvu_msgs::ColorState>("set_color", 20, &RGBLedPlugin::colorCB, this);
  ros_color_state_pub_ = ros_node_->advertise<ruvu_msgs::ColorState>("color_state", 1, true); // Latched

  color_state_.header.stamp = ros::Time::now();
  ros_color_state_pub_.publish(color_state_);
}

void RGBLedPlugin::colorCB(const ruvu_msgs::ColorStateConstPtr& msg)
{
  if (msg->color.size() != msg->name.size())
  {
    ROS_ERROR("Invalid ColorState message, color and name should have the same size. Ignoring message ..");
    return;
  }

  for (unsigned int i = 0; i < msg->color.size(); ++i)
  {
    std::string name = msg->name[i];
    std_msgs::ColorRGBA color = msg->color[i];

    size_t pos = std::find(color_state_.name.begin(), color_state_.name.end(), name) - color_state_.name.begin();
    if (pos < color_state_.name.size())
    {
      color_state_.color[pos] = color;

      for (unsigned int j= 0; j < model_->GetChildCount(); ++j)
        {
          model_->GetChild(j)->GetName();
        }

      physics::LinkPtr link = model_->GetLink(link_name_);

      if (!link || !link->GetParent())
      {
        ROS_ERROR_STREAM("Link '" << link_name_ << "' does not exist or does not have a parent.");
      }
      else
      {
        setLinkVisualColor(link, name, gazebo_visual_pub_, colorMsgToGazebo(msg->color[i]));
      }
    }
    else
    {
      ROS_WARN_STREAM("Led with name '" << msg->name[i] << "' could not be found!");
    }
  }

  color_state_.header.stamp = ros::Time::now();
  ros_color_state_pub_.publish(color_state_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RGBLedPlugin)

}
