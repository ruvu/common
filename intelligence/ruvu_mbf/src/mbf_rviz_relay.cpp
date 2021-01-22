// Copyright (c) 2018 RUVU Robotics

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  auto controller = private_nh.param("controller", std::string{});
  auto planner = private_nh.param("planner", std::string{});
  auto tolerance = private_nh.param("tolerance", 1.0);

  actionlib::SimpleActionClient<mbf_msgs::GetPathAction> get_path_ac("get_path", true);
  actionlib::SimpleActionClient<mbf_msgs::ExePathAction> exe_path_ac("exe_path", true);

  while (!get_path_ac.waitForServer(ros::Duration(5)) && ros::ok())
    ROS_INFO("Still waiting for for get_path");
  while (!exe_path_ac.waitForServer(ros::Duration(5)) && ros::ok())
    ROS_INFO("Still waiting for exe_path_ac");

  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> simple_goal_cb =
      [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        {
          ROS_INFO("Calculating path");
          mbf_msgs::GetPathGoal goal;
          goal.target_pose = *msg;
          goal.tolerance = tolerance;
          goal.planner = planner;
          if (get_path_ac.sendGoalAndWait(goal) != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_ERROR("Failed to calculate a path");
            return;
          }
        }
        {
          ROS_INFO("Executing path");
          auto result = get_path_ac.getResult();
          mbf_msgs::ExePathGoal goal;
          goal.path = result->path;
          goal.controller = controller;
          exe_path_ac.sendGoalAndWait(goal);
        }
      };
  auto sub = nh.subscribe("simple_goal", 1, simple_goal_cb);

  ROS_INFO("%s started", private_nh.getNamespace().c_str());
  ros::spin();
}
