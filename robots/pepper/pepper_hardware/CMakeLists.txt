cmake_minimum_required(VERSION 2.8.3)
project(pepper_hardware)

find_package(catkin REQUIRED COMPONENTS roslint)

# Make sure to install the SDK (required for this package)
catkin_add_env_hooks(99.setup_python_sdk SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

# Tests
if (CATKIN_ENABLE_TESTING)
  # ROS lint
  file(GLOB_RECURSE PYTHON_SRCS src/*.py)
  file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*)
  roslint_python(${PYTHON_SRCS} ${PYTHON_SCRIPTS})
  roslint_add_test()

  # Launch check
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)

  # Catkin lint
  find_program(CATKIN_LINT catkin_lint REQUIRED)
  execute_process(COMMAND "${CATKIN_LINT}" "-q" "${CMAKE_SOURCE_DIR}" RESULT_VARIABLE lint_result)
  if(NOT ${lint_result} EQUAL 0)
    message(FATAL_ERROR "catkin_lint failed")
  endif()
endif()

install(PROGRAMS
  scripts/autonomous_life_disabler
  scripts/base_controller
  scripts/battery_status
  scripts/body_controller
  scripts/camera
  scripts/face_detector
  scripts/laser
  scripts/sound_source_localization
  scripts/speech_recognition
  scripts/tablet
  scripts/text_to_speech
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
