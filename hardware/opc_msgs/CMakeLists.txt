# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(opc_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  ruvu_lint
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Colors.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

ruvu_lint_add_test()
