cmake_minimum_required(VERSION 2.8.3)
project(ruvu_networkx)
find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
	catkin_add_nosetests(test)

    # ROS Lint
    file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
    file(GLOB_RECURSE PY_FILES *.py)
    roslint_python(${PYTHON_SCRIPTS} ${PY_FILES})
    roslint_add_test()
endif()

install(PROGRAMS
  scripts/generate_grid_graph
  scripts/pose_graph_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
