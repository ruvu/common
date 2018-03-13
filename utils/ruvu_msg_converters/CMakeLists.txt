cmake_minimum_required(VERSION 2.8.3)
project(ruvu_msg_converters)
find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
    roslint_python(${PYTHON_SCRIPTS})
    roslint_add_test()
endif()

install(PROGRAMS
  scripts/odom_to_pose
  scripts/p3d_to_imu
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
