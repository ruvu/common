# Copyright 2020 RUVU Robotics B.V.

@[if INSTALLSPACE]@
# bin and template dir variables in installspace
set(RUVU_LINT_SCRIPTS_DIR "${ruvu_lint_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[else]@
# bin and template dir variables in develspace
set(RUVU_LINT_SCRIPTS_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[end if]@

function(_ruvu_lint_roslint_cpp)
  if(NOT ROSLINT_CPP_OPTS)
    set(ROSLINT_CPP_OPTS "--filter=-build/header_guard")
  endif()
  file(GLOB_RECURSE ROSLINT_CPP_ARGN *.cpp *.h *.hpp)
  if(ROSLINT_CPP_ARGN STREQUAL "")
    message(STATUS "roslint_cpp: no files provided for command")
  else()
    roslint_cpp(${ROSLINT_CPP_ARGN})
  endif()
endfunction()

function(_ruvu_lint_roslint_python)
  # find all python files, including shebang
  ## Call out to find_python_files to look for python files
  message(STATUS "find_python_files in ${CMAKE_CURRENT_SOURCE_DIR}")
  execute_process(COMMAND "${RUVU_LINT_SCRIPTS_DIR}/find_python_files"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE find_python_files_result
    ERROR_VARIABLE find_python_files_err
    OUTPUT_VARIABLE find_python_files_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(find_python_files_result)
    message(WARNING "failed to find_python_files in ${CMAKE_CURRENT_SOURCE_DIR}
${find_python_files_err}")
  endif(find_python_files_result)
  separate_arguments(find_python_files_deps_result)

  if (find_python_files_deps_result STREQUAL "")
    message(STATUS "roslint_python: no files provided for command")
  else()
    roslint_python(${find_python_files_deps_result})
  endif()
endfunction()

function(_ruvu_lint_xmllint)
  file(GLOB_RECURSE ARGN *.config *.gazebo *.launch *.sdf *.test *.urdf *.world *.xacro *.xml)

  if(NOT RUVU_LINT_XMLLINT_OPTS)
    set(RUVU_LINT_XMLLINT_OPTS --noout)
  endif()

  roslint_custom("xmllint" "${RUVU_LINT_XMLLINT_OPTS}" ${ARGN})
endfunction()

function(_ruvu_lint_yamllint)
  file(GLOB_RECURSE ARGN *.yml *.yaml)

  if(NOT RUVU_LINT_YAMLLINT_OPTS)
    set(RUVU_LINT_YAMLLINT_OPTS -d "{extends: default, rules: {line-length: {max: 120}}}")
  endif()

  if(ARGN STREQUAL "")
    message(STATUS "yamllint: no files provided for command")
  else()
  roslint_custom("yamllint" "${RUVU_LINT_YAMLLINT_OPTS}" ${ARGN})
  endif()
endfunction()

function(_ruvu_lint_catkin_lint)
  find_program(CATKIN_LINT catkin_lint REQUIRED)

  if(NOT CATKIN_LINT_OPTS)
    set(CATKIN_LINT_OPTS -W1 --quiet)
  endif()
  separate_arguments(CATKIN_LINT_OPTS)

  execute_process(COMMAND "${CATKIN_LINT}" ${CATKIN_LINT_OPTS} "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE lint_result
    OUTPUT_VARIABLE lint_output
    ERROR_VARIABLE lint_output
  )
  if(lint_result EQUAL 0)
    if(NOT lint_output STREQUAL "")
      message(WARNING ${lint_output})
    endif()
  else()
    message(SEND_ERROR "catkin_lint failed:\n${lint_output}")
  endif()
endfunction()

function(ruvu_lint_add_test)
  if (CATKIN_ENABLE_TESTING)
    find_package(roslint)

    # create the main roslint target, even when there are no files
    _roslint_create_targets()

    _ruvu_lint_roslint_cpp()
    _ruvu_lint_roslint_python()
    _ruvu_lint_catkin_lint()
    _ruvu_lint_xmllint()
    _ruvu_lint_yamllint()

    # linter fails are able to break the build
    roslint_add_test()
  endif()
endfunction()
