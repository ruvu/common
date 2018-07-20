cmake_minimum_required(VERSION 2.8.3)
project(ruvu_lint)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/find_python_files
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
