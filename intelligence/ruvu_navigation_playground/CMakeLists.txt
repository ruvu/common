cmake_minimum_required(VERSION 2.8.3)
project(ruvu_navigation_playground)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_package()

ruvu_lint_add_test()

if (CATKIN_ENABLE_TESTING)
  file(GLOB_RECURSE TEST_LAUNCH_FILES launch/*.test)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(${TEST_LAUNCH_FILES})
endif()
