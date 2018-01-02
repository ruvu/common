cmake_minimum_required(VERSION 2.8.3)
project(ruvu_networkx)
find_package(catkin)

catkin_python_setup()

catkin_package()

###########
## TEST ##
###########

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    # TODO
endif()
