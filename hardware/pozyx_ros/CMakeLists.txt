# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 2.8.3)
project(pozyx_ros)

find_package(catkin REQUIRED COMPONENTS message_generation roslint std_msgs)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    DeviceRange.msg
    DeviceRanges.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    roslint_python()

    # Launch check
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(test/test_pozyx_ros.test)
    roslaunch_add_file_check(test/test_two_tag_pozyx_ros.test)

    # Catkin lint
    find_program(CATKIN_LINT catkin_lint REQUIRED)
    execute_process(COMMAND "${CATKIN_LINT}" "-q" "-W2" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
    if(NOT ${lint_result} EQUAL 0)
        message(FATAL_ERROR "catkin_lint failed")
    endif()
endif()

install(PROGRAMS
  scripts/pozyx_positioning_node.py
  scripts/pozyx_ranging_node.py
  scripts/two_tag_positioner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
