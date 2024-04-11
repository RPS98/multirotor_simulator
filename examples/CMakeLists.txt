file(GLOB EXAMPLES_SOURCE "*_example.cpp")

# Print all examples
foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  MESSAGE(STATUS "Adding example: ${EXAMPLE_NAME}")
endforeach()


find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)

  add_executable(${PROJECT_NAME}_${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${PROJECT_NAME}_${EXAMPLE_NAME} ${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
endforeach()

# Simulations with trajectory generation
add_subdirectory(trajectory_fly_example)
