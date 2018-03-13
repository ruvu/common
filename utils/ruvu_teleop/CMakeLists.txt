cmake_minimum_required(VERSION 2.8.3)
project(ruvu_teleop)

find_package(catkin REQUIRED COMPONENTS roslint)
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    roslint_python()
    roslint_add_test()
endif()

install(PROGRAMS
  scripts/teleop_single_joint_position_joy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
