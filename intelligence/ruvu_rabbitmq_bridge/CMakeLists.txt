# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_rabbitmq_bridge)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()

if (CATKIN_ENABLE_TESTING)
    # Rostest
    find_package(rostest REQUIRED)
    add_rostest(test/test_rabbitmq_bridge.test)
    add_rostest(test/test_publishers.test)
    # add_rostest(test/test_rabbitmq_rpc.test)  # this test is flaky
    add_rostest(test/test_message_rate.test)
endif()

catkin_install_python(PROGRAMS
    scripts/rabbitmq_bridge
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
