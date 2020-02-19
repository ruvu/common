cmake_minimum_required(VERSION 2.8.3)
project(ruvu_msg_converters)
find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()

install(PROGRAMS
  scripts/odom_to_pose_with_covariance_stamped
  scripts/rewrite_odom_covariance
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
