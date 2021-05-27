# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_bringup)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)
catkin_python_setup()

catkin_add_env_hooks(99.ruvu_bringup_connect_to_external_ros_master SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(99.ruvu_bringup_set_ros_ip_from_network_interface SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(99.ruvu_bringup_set_ros_master_uri SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_package()

ruvu_lint_add_test()

install(PROGRAMS
    scripts/connect_to_external_ros_master.bash
    scripts/generate_functions_for_launch_files_in_package.bash
    scripts/install_systemd_roscore
    scripts/set_ros_ip_from_network_interface.bash
    scripts/set_ros_master_uri.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/install_systemd_launch_files
    scripts/systemd_service_from_launch_file.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    systemd
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
