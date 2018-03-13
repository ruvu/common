cmake_minimum_required(VERSION 2.8.3)
project(ruvu_rostest)
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    roslint_python()
    roslint_add_test()
endif()

install(PROGRAMS
  scripts/test_twist_odom_has_movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
