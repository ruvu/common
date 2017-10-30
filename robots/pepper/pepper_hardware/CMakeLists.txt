cmake_minimum_required(VERSION 2.8.3)
project(pepper_hardware)

find_package(catkin REQUIRED)

# Make sure to install the SDK (required for this package)
catkin_add_env_hooks(99.setup_python_sdk SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/base_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
