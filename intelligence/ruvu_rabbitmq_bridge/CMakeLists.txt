cmake_minimum_required(VERSION 2.8.3)
project(ruvu_rabbitmq_bridge)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()

if (CATKIN_ENABLE_TESTING)
    # Rostest
    find_package(rostest REQUIRED)
    add_rostest(test/test_rabbitmq_bridge.test)
    add_rostest(test/test_rabbitmq_rpc.test)
endif()

install(PROGRAMS
	scripts/rabbitmq_bridge
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
