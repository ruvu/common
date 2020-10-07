# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_teleop)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)
catkin_package()

ruvu_lint_add_test()

catkin_install_python(PROGRAMS
  scripts/teleop_cancel_actionlib_goal_joy
  scripts/teleop_publish_pose_joy
  scripts/teleop_single_joint_position_joy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
