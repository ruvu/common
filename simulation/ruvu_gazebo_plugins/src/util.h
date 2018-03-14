//
// Copyright (c) 2017 RUVU Robotics
//
// @author Rein Appeldoorn
//

#pragma once

#include <boost/algorithm/string/trim.hpp>
#include <gazebo/gazebo.hh>
#include <ros/console.h>
#include <string>

namespace gazebo
{
template <typename T>
T getParameterFromSDF(sdf::ElementPtr sdf, const std::string& parameter_name, const T& default_value)
{
  T parameter_value = default_value;
  if (!sdf->HasElement(parameter_name))
  {
    ROS_WARN_STREAM("Missing parameter <" << parameter_name << ">, defaults to \"" << parameter_value << "\"");
  }
  else
  {
    parameter_value = sdf->GetElement(parameter_name)->Get<T>();
    ROS_INFO_STREAM("Parameter <" << parameter_name << "> = \"" << parameter_value << "\"");
  }
  return parameter_value;
}

std::string robotNamespaceToTFPrefix(const std::string& robot_namespace)
{
  std::string tf_prefix = robot_namespace;

  // Take the first group
  tf_prefix = tf_prefix.substr(0, tf_prefix.find("/", 1)) + "/";
  boost::trim_left_if(tf_prefix, boost::is_any_of("/"));

  if (!tf_prefix.empty())
  {
    ROS_INFO_STREAM("<robotNamespace>" << robot_namespace << "<robotNamespace> resolves to tfPrefix '" << tf_prefix
                                       << "'");
  }
  return tf_prefix;
}
}  // namespace gazebo
