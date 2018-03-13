cmake_minimum_required(VERSION 2.8.3)
project(ruvu_bringup)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    find_package(catkin REQUIRED COMPONENTS roslint)
    file(GLOB_RECURSE PYTHON_SRCS *.py)
    roslint_python(${PYTHON_SRCS} scripts/install_systemd_launch_files)
    roslint_add_test()
endif()
