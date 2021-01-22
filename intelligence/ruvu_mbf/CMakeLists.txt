# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mbf)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS ruvu_lint roscpp mbf_msgs actionlib)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp mbf_msgs actionlib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mbf_rviz_relay src/mbf_rviz_relay.cpp)
target_link_libraries(mbf_rviz_relay
  ${catkin_LIBRARIES}
)
target_compile_warnings(mbf_rviz_relay)

#############
## Install ##
#############

install(TARGETS mbf_rviz_relay
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
	scripts/pose_stamped_move_base_goal
	scripts/get_path_exe_path_relay
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# TODO: remove build/c++11 when kinetic is dropped
set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces,-build/c++11")
ruvu_lint_add_test()
