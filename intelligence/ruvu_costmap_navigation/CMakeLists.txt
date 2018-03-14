cmake_minimum_required(VERSION 2.8.3)
project(ruvu_costmap_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mbf_costmap_core
  mbf_costmap_nav
  roslint
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ruvu_costmap_navigation
  CATKIN_DEPENDS mbf_costmap_core mbf_costmap_nav
)

# Tests
if (CATKIN_ENABLE_TESTING)
	# ROS Lint
	roslint_python(scripts/pose_stamped_move_base_goal)
	roslint_add_test()

	# Launch check
	find_package(roslaunch REQUIRED)
	file(GLOB_RECURSE TEST_LAUNCH_FILES test/launch/*.test)
	roslaunch_add_file_check(${TEST_LAUNCH_FILES})

	# Catkin lint
	find_program(CATKIN_LINT catkin_lint REQUIRED)
	execute_process(COMMAND "${CATKIN_LINT}" "-q" "-W2" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
	if(NOT ${lint_result} EQUAL 0)
		message(FATAL_ERROR "catkin_lint failed")
	endif()
endif()

install(PROGRAMS
	scripts/pose_stamped_move_base_goal
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
