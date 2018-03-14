cmake_minimum_required(VERSION 2.8.3)
project(ruvu_rostest)
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    roslint_python()
    roslint_add_test()

    # Catkin lint
    find_program(CATKIN_LINT catkin_lint REQUIRED)
    execute_process(COMMAND "${CATKIN_LINT}" "-q" "-W2" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
    if(NOT ${lint_result} EQUAL 0)
    message(FATAL_ERROR "catkin_lint failed")
    endif()
endif()

install(PROGRAMS
  scripts/test_twist_odom_has_movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
