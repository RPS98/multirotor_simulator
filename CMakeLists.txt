cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME multirotor_simulator)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()

# Add project submodules
set(PROJECT_SUBMODULES
  pid_controller
)

foreach(SUBMODULE ${PROJECT_SUBMODULES})
  add_subdirectory(${SUBMODULE})
endforeach()

# find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  ${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
  foreach(SUBMODULE ${PROJECT_SUBMODULES})
  ${SUBMODULE}/include
  endforeach()
)

set(SOURCE_HPP_FILES

  # IMU simulator
  ${PROJECT_NAME}/imu_simulator/imu.hpp
  ${PROJECT_NAME}/imu_simulator/inertial_odometry.hpp

  # Multirotor controllers
  ${PROJECT_NAME}/multirotor_controllers/multirotor_controllers.hpp
  ${PROJECT_NAME}/multirotor_controllers/controllers/acro_controller.hpp
  ${PROJECT_NAME}/multirotor_controllers/controllers/indi_controller.hpp
  ${PROJECT_NAME}/multirotor_controllers/controllers/position_controller.hpp
  ${PROJECT_NAME}/multirotor_controllers/controllers/trajectory_controller.hpp
  ${PROJECT_NAME}/multirotor_controllers/controllers/velocity_controller.hpp

  # Multirotor dynamic model
  ${PROJECT_NAME}/multirotor_dynamic_model/common/state.hpp
  ${PROJECT_NAME}/multirotor_dynamic_model/common/utils.hpp
  ${PROJECT_NAME}/multirotor_dynamic_model/model.hpp
  ${PROJECT_NAME}/multirotor_dynamic_model/dynamics.hpp

  # Multirotor simulator
  ${PROJECT_NAME}/multirotor_simulator.hpp
)

# Create library
add_library(${PROJECT_NAME} INTERFACE ${SOURCE_HPP_FILES})

# Export library headers
target_include_directories(${PROJECT_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIRS}
  foreach(SUBMODULE ${PROJECT_SUBMODULES})
  ${SUBMODULE}/include
  endforeach()
)

# Export library dependencies
target_link_libraries(${PROJECT_NAME} INTERFACE ${PROJECT_SUBMODULES})

# Tests
include(CTest)

# Check if tests are enabled
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
