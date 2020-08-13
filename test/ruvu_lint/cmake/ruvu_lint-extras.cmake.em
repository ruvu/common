@[if INSTALLSPACE]@
# bin and template dir variables in installspace
set(RUVU_LINT_SCRIPTS_DIR "${ruvu_lint_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[else]@
# bin and template dir variables in develspace
set(RUVU_LINT_SCRIPTS_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[end if]@

function(ruvu_lint_add_test)
  if (CATKIN_ENABLE_TESTING)
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

    find_package(roslint)

    # create the main roslint target, even when there are no files
    _roslint_create_targets()

    # python
    roslint_python(${find_python_files_deps_result})

    # c++
    if(NOT ROSLINT_CPP_OPTS)
      set(ROSLINT_CPP_OPTS "--filter=-build/header_guard")
    endif()
    roslint_cpp()

    # linter fails are able to break the build
    roslint_add_test()

    # catkin_lint
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
  endif()
endfunction()
