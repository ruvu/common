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
