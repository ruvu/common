cmake_minimum_required(VERSION 2.8.3)
project(pozyx_ros)

find_package(catkin REQUIRED message_generation std_msgs)

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

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
    # ROS Lint
    find_package(catkin REQUIRED roslint)
    roslint_python()
    roslint_add_test()

    # Launch check
    find_package(roslaunch REQUIRED)
    file(GLOB_RECURSE TEST_LAUNCH_FILES test/*.test)
    roslaunch_add_file_check(${TEST_LAUNCH_FILES})
endif()
