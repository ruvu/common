cmake_minimum_required(VERSION 2.8.3)
project(pepper_hardware)

find_package(catkin REQUIRED)

# Make sure to install the SDK (required for this package)
catkin_add_env_hooks(99.setup_python_sdk SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
	find_package(catkin REQUIRED roslint)
	file(GLOB_RECURSE PYTHON_SRCS src/*.py)
	file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
	file(GLOB_RECURSE PYTHON_TESTS test/*)
	roslint_python(${PYTHON_SRCS} ${PYTHON_SCRIPTS} ${PYTHON_TESTS})
    roslint_add_test()

    # Launch check
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

install(PROGRAMS
  scripts/base_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
