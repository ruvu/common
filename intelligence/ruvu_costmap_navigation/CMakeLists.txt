cmake_minimum_required(VERSION 2.8.3)
project(ruvu_costmap_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mbf_costmap_core
  mbf_costmap_nav
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ruvu_costmap_navigation
  CATKIN_DEPENDS mbf_costmap_core mbf_costmap_nav
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
