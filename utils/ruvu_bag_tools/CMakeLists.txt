cmake_minimum_required(VERSION 2.8.3)
project(ruvu_bag_tools)

find_package(catkin REQUIRED COMPONENTS roslint)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package()

#############
## Install ##
#############

install(PROGRAMS
  scripts/bag_to_json
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  # Catkin lint
  find_program(CATKIN_LINT catkin_lint REQUIRED)
  execute_process(COMMAND "${CATKIN_LINT}" "-q" "-W2" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
  if(NOT ${lint_result} EQUAL 0)
      message(FATAL_ERROR "catkin_lint failed")
  endif()

  # ROS Lint
  file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
  roslint_python(${PYTHON_SCRIPTS})
  roslint_add_test()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
endif()
