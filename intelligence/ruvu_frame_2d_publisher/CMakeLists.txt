# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 2.8.3)
project(ruvu_frame_2d_publisher)
find_package(catkin REQUIRED COMPONENTS ruvu_lint)
catkin_package()

ruvu_lint_add_test()

install(PROGRAMS
  scripts/frame_2d_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
