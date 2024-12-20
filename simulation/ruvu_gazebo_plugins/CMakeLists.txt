# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_gazebo_plugins)

# -Wno-overloaded-virtual was required for gazebo on kinetic
set(EXTRA_FLAGS -Wno-overloaded-virtual)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  gazebo_dev
  gazebo_plugins
  geometry_msgs
  nav_msgs
  roscpp
  ruvu_lint
  sensor_msgs
  tf2_ros
)
#include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    gazebo_plugins
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    tf2_ros
)

###########
## Build ##
###########

# show headers in qtcreator
FILE(GLOB_RECURSE HEADERS "src/*.hpp")
add_custom_target(headers SOURCES ${HEADERS})

add_library(ruvu_disable_physics src/disable_physics_plugin.cpp)
target_compile_warnings(ruvu_disable_physics ${EXTRA_FLAGS})
target_link_libraries(ruvu_disable_physics ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_twist_teleport src/twist_teleport_plugin.cpp src/twist_plugin.cpp)
target_compile_warnings(ruvu_twist_teleport ${EXTRA_FLAGS})
target_link_libraries(ruvu_twist_teleport ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_twist_move src/twist_move_plugin.cpp src/twist_plugin.cpp)
target_compile_warnings(ruvu_twist_move ${EXTRA_FLAGS})
target_link_libraries(ruvu_twist_move ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_single_joint_position_lifter src/single_joint_position_lifter_plugin.cpp)
target_compile_warnings(ruvu_single_joint_position_lifter ${EXTRA_FLAGS})
target_link_libraries(ruvu_single_joint_position_lifter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_pose_teleport src/pose_teleport_plugin.cpp)
target_compile_warnings(ruvu_pose_teleport ${EXTRA_FLAGS})
target_link_libraries(ruvu_pose_teleport ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(ruvu_accerion_triton src/accerion_triton_plugin.cpp)
target_compile_warnings(ruvu_accerion_triton ${EXTRA_FLAGS})
target_link_libraries(ruvu_accerion_triton ${Boost_LIBRARIES} ${catkin_LIBRARIES})

###########
## TEST ##
###########

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-build/include_what_you_use")
ruvu_lint_add_test()

 if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # add_rostest(test/disable_physics_world.test)
  # add_rostest(test/pose_teleport_world.test)
  # add_rostest(test/single_joint_position_lifter_world.test)
  # add_rostest(test/twist_move_urdf_world.test)
  # add_rostest(test/twist_move_world.test)
  # add_rostest(test/twist_teleport_urdf_world.test)
  # add_rostest(test/twist_teleport_world.test)
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
  ruvu_pose_teleport
  ruvu_single_joint_position_lifter
  ruvu_accerion_triton
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/spawn_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
