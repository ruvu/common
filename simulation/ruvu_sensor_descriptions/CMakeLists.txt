cmake_minimum_required(VERSION 2.8.3)
project(ruvu_sensor_descriptions)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package()

if (CATKIN_ENABLE_TESTING)
    # Catkin lint
    find_program(CATKIN_LINT catkin_lint REQUIRED)
    execute_process(COMMAND "${CATKIN_LINT}" "-q" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
    if(NOT ${lint_result} EQUAL 0)
    message(FATAL_ERROR "catkin_lint failed")
    endif()
endif()
