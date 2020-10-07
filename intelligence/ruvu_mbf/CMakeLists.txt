# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mbf)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_package()

ruvu_lint_add_test()

install(PROGRAMS
	scripts/pose_stamped_move_base_goal
	scripts/get_path_exe_path_relay
	scripts/mbf_rviz_relay
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
