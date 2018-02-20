cmake_minimum_required(VERSION 2.8.3)
project(ruvu_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs control_msgs actionlib sensor_msgs tf2_ros)
#include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs
    #    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

add_library(ruvu_disable_physics src/disable_physics_plugin.cpp)
target_link_libraries(ruvu_disable_physics ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_twist_teleport src/twist_teleport_plugin.cpp src/twist_plugin.cpp src/twist_plugin.h src/util.h)
target_link_libraries(ruvu_twist_teleport ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_twist_move src/twist_move_plugin.cpp src/twist_plugin.cpp src/twist_plugin.h src/util.h)
target_link_libraries(ruvu_twist_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_single_joint_position_lifter src/single_joint_position_lifter_plugin.cpp src/single_joint_position_lifter_plugin.h src/util.h)
target_link_libraries(ruvu_single_joint_position_lifter ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

###########
## TEST ##
###########

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    #add_rostest(test/disable_physics_world.test)
    #add_rostest(test/twist_move_world.test)
    #add_rostest(test/twist_move_urdf_world.test)
    #add_rostest(test/twist_teleport_urdf_world.test)
    #add_rostest(test/twist_teleport_world.test)
    #add_rostest(test/single_joint_position_lifter_world.test)
endif()

#############
## Install ##
#############

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

install(TARGETS
  ruvu_disable_physics
  ruvu_twist_teleport
  ruvu_twist_move
  ruvu_single_joint_position_lifter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
