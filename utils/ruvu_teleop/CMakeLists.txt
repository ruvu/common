cmake_minimum_required(VERSION 2.8.3)
project(ruvu_teleop)

find_package(catkin)
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    find_package(catkin REQUIRED COMPONENTS roslint)
    roslint_python()
    roslint_add_test()
endif()
