#include <ros/console.h>
#include <gazebo/gazebo.hh>

namespace gazebo
{

template<typename T>
T getParameterFromSDF(sdf::ElementPtr sdf, const std::string &parameter_name, const T &default_value) {
  T parameter_value = default_value;
  if (!sdf->HasElement(parameter_name)) {
    ROS_WARN_STREAM("Missing parameter <" << parameter_name << ">, defaults to \"" << parameter_value << "\"");
  } else {
    parameter_value = sdf->GetElement(parameter_name)->Get<T>();
    ROS_INFO_STREAM("Missing parameter <" << parameter_name << "> = \"" << parameter_value << "\"");
  }
  return parameter_value;
}

}
