# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_rostest)
find_package(catkin REQUIRED COMPONENTS ruvu_lint)
catkin_package()

ruvu_lint_add_test()

install(PROGRAMS
  scripts/test_twist_odom_has_movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
