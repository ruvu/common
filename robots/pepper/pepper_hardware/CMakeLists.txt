cmake_minimum_required(VERSION 2.8.3)
project(pepper_hardware)

find_package(catkin REQUIRED COMPONENTS ruvu_lint)

# Make sure to install the SDK (required for this package)
catkin_add_env_hooks(99.setup_python_sdk SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package()

ruvu_lint_add_test()

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
