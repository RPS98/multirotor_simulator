cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME quadrotor_model)
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
  ament_cmake
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}

  # uncomment if using Eigen3
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
  src/dynamics.cpp
  src/flight_controller.cpp
)

# create library and export it
add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# # check if tests are enabled
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(benchmark REQUIRED)

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  file(GLOB TEST_SOURCE "tests/*_benchmark.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark ${PROJECT_NAME})
  endforeach()
endif()

ament_package()