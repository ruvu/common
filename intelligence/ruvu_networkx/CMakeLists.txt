# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_networkx)
find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()

catkin_install_python(PROGRAMS
  scripts/generate_grid_graph
  scripts/pose_graph_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
