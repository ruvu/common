cmake_minimum_required(VERSION 2.8.3)
project(ruvu_bringup)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)
catkin_python_setup()
catkin_package()

ruvu_lint_add_test()

install(PROGRAMS
    scripts/generate_functions_for_launch_files_in_package.bash
    scripts/install_systemd_launch_files
    scripts/install_systemd_roscore
    scripts/set_ros_ip_from_network_interface.bash
    scripts/set_ros_master_uri.bash
    scripts/systemd_service_from_launch_file.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
