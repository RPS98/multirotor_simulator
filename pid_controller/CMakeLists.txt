cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME pid_controller)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_HPP_FILES
  include/${PROJECT_NAME}/pid.hpp
  include/${PROJECT_NAME}/pid_1d.hpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCE_HPP_FILES})

# Specify the language for the target as C++
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Export library headers
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

