cmake_minimum_required(VERSION 2.8.3)
project(ruvu_rqt)
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_python_setup()
catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    roslint_python()
    roslint_add_test()

    # Catkin lint
    find_program(CATKIN_LINT catkin_lint REQUIRED)
    execute_process(COMMAND "${CATKIN_LINT}" "-q" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
    if(NOT ${lint_result} EQUAL 0)
        message(FATAL_ERROR "catkin_lint failed")
    endif()
endif()

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/publish_pose
  scripts/rqt.bash
  scripts/single_joint_position_action
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
