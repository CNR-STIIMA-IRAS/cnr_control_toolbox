add_library(state_space_systems INTERFACE)

target_include_directories(state_space_systems
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

target_link_libraries(state_space_systems
  INTERFACE
  eigen_matrix_utils::eigen_matrix_utils
)

add_library(state_space_systems::state_space_systems ALIAS state_space_systems)






#############
## Testing ##
#############

add_executable(test_state_space_matrix
  test/test_state_space_matrix.cpp )
target_link_libraries(test_state_space_matrix
  PRIVATE
  state_space_systems::state_space_systems
   GTest::gtest_main
)


target_compile_options(test_state_space_matrix PUBLIC $<$<CONFIG:Release>:-Ofast >)
gtest_discover_tests(test_state_space_matrix)
add_test(NAME test_state_space_matrix
  COMMAND test_state_space_matrix)


if(ENABLE_COVERAGE_TESTING)
  set(COVERAGE_EXCLUDES "*/state_space_systems/test*")
  add_code_coverage(
    NAME coverage_report
    DEPENDENCIES test_state_space_matrix
  )
endif()



#############
## Install ##
#############
install(DIRECTORY include/state_space_systems
  DESTINATION include)

install(
  TARGETS state_space_systems test_state_space_matrix #node_connection_test kdtree_test
  EXPORT state_space_systemsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT state_space_systemsTargets
  DESTINATION "share/cnr_control_toolbox/cmake"
  NAMESPACE state_space_systems::
  FILE state_space_systemsTargets.cmake
)
