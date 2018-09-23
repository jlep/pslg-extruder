
cmake_minimum_required (VERSION 2.8.11)

if(${CMAKE_MAJOR_VERSION} STREQUAL "3")
  # CMake 3.0 introduces VERSION variables for each Project.
  cmake_policy(SET CMP0048 NEW)
  # CMake 3.0 introduces some OS X RPath policies
  cmake_policy(SET CMP0042 OLD)
endif()


project (predicates VERSION 1.0.0)

#if(NOT MSVC)
#  list (APPEND CMAKE_C_FLAGS "-std=c99 -Wall")
#  list (APPEND CMAKE_CXX_FLAGS "-std=c++11 -Wall")
#endif()

set(LIB_TYPE STATIC)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory (src)

install (DIRECTORY ${PROJECT_SOURCE_DIR}/include/
         DESTINATION include/predicates)
