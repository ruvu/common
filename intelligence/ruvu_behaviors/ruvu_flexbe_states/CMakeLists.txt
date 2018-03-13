cmake_minimum_required(VERSION 2.8.3)
project(ruvu_flexbe_states)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    roslint_python()
    roslint_add_test()
endif()
