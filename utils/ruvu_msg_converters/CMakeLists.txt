cmake_minimum_required(VERSION 2.8.3)
project(ruvu_msg_converters)
find_package(catkin REQUIRED)
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    find_package(catkin REQUIRED roslint)
    file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
    roslint_python(${PYTHON_SCRIPTS})
    roslint_add_test()
endif()
