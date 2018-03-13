cmake_minimum_required(VERSION 2.8.3)
project(pozyx_ros)

find_package(catkin REQUIRED COMPONENTS message_generation roslint std_msgs)

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
    roslint_add_test()

    # Launch check
    find_package(roslaunch REQUIRED)
    file(GLOB_RECURSE TEST_LAUNCH_FILES test/*.test)
    roslaunch_add_file_check(${TEST_LAUNCH_FILES})
endif()

install(PROGRAMS
  scripts/pozyx_node.py
  scripts/range_info_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
