cmake_minimum_required(VERSION 2.8.3)
project(ruvu_rabbitmq_bridge)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  	# ROS Lint
    file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
    file(GLOB_RECURSE PY_FILES *.py)
    file(GLOB_RECURSE TEST_SCRIPTS test/rabbitmq_*)
    roslint_python(${PYTHON_SCRIPTS} ${TEST_SCRIPTS} ${PY_FILES})
    roslint_add_test()

    # Catkin lint
    find_program(CATKIN_LINT catkin_lint REQUIRED)
    execute_process(COMMAND "${CATKIN_LINT}" "-q" "-W2" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
    if(NOT ${lint_result} EQUAL 0)
    	message(FATAL_ERROR "catkin_lint failed")
    endif()

    # Rostest
    find_package(rostest REQUIRED)
    add_rostest(test/test_rabbitmq_bridge.test)
endif()

install(PROGRAMS
	scripts/rabbitmq_bridge
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
