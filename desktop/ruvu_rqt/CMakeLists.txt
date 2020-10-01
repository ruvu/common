# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 2.8.3)
project(ruvu_rqt)
find_package(catkin REQUIRED COMPONENTS ruvu_lint)
catkin_python_setup()
catkin_package()

ruvu_lint_add_test()

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/publish_pose
  scripts/rqt.bash
  scripts/single_joint_position_action
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
