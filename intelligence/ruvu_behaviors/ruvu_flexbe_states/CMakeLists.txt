# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 2.8.3)
project(ruvu_flexbe_states)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()
