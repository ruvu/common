# Copyright 2020 RUVU Robotics B.V.

# Inspired by https://github.com/lefticus/cpp_starter_project/blob/master/cmake/CompilerWarnings.cmake

function(target_compile_warnings target)
  option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" TRUE)

  set(GENERIC_WARNINGS
      -Werror
      -Wall
      -Wextra
      -Wno-unused-parameter # several ROS libraries have unused parameters
      -Wcast-align
      -Woverloaded-virtual
      -Wformat=2
      # some extra warnings I found on stackoverflow
      -Wdisabled-optimization
      -Wredundant-decls
      -Wsign-promo
      -Wstrict-overflow=5
      -Wswitch-default
  )

  set(GCC_WARNINGS
      ${GENERIC_WARNINGS}
      -Wnoexcept
      -Wstrict-null-sentinel
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(PROJECT_WARNINGS ${GENERIC_WARNINGS} ${ARGN})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${GCC_WARNINGS} ${ARGN})
  else()
    message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
  endif()

  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()
