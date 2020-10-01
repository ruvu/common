# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 2.8.3)
project(ruvu_msg_converters)
find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()

catkin_install_python(PROGRAMS
  scripts/odom_to_pose_with_covariance_stamped
  scripts/odom_to_tf
  scripts/pose_stamped_to_tf
  scripts/pose_with_covariance_stamped_to_pose_stamped
  scripts/rewrite_imu_covariance
  scripts/rewrite_odom_covariance
  scripts/tf_to_pose_stamped
  scripts/twist_to_twist_stamped
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
