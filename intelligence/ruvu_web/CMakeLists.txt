cmake_minimum_required(VERSION 3.0.2)
project(ruvu_web)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_package()

ruvu_lint_add_test()
