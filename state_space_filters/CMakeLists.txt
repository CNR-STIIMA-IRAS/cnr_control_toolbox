add_library(state_space_filters INTERFACE)

target_include_directories(state_space_filters
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
  )

target_link_libraries(state_space_filters
  INTERFACE
  state_space_systems::state_space_systems
)

add_library(state_space_filters::state_space_filters ALIAS state_space_filters)




#############
## Testing ##
#############

add_executable(filtered_values_test test/test_filtered_values.launch test/test_filtered_values.cpp)
target_link_libraries(filtered_values_test
  PUBLIC
  state_space_filters::state_space_filters
   GTest::gtest_main
)
target_compile_options(filtered_values_test PUBLIC $<$<CONFIG:Release>:-Ofast >)
gtest_discover_tests(filtered_values_test)
add_test(NAME filtered_values_test
  COMMAND filtered_values_test)


add_executable(test_filters test/test_filters.launch test/test_filters.cpp)
target_link_libraries(test_filters
  PUBLIC
  state_space_filters::state_space_filters
   GTest::gtest_main
)
target_compile_options(test_filters PUBLIC $<$<CONFIG:Release>:-Ofast >)
gtest_discover_tests(test_filters)
add_test(NAME test_filters
  COMMAND test_filters)


if(ENABLE_COVERAGE_TESTING)
  set(COVERAGE_EXCLUDES "*/state_space_filters/test*")
  add_code_coverage(
    NAME coverage_report
    DEPENDENCIES filtered_values_test test_filters
  )
endif()



#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/state_space_filters
  DESTINATION include)

install(
  TARGETS state_space_filters test_filters filtered_values_test
  EXPORT state_space_filtersTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(EXPORT state_space_filtersTargets
  DESTINATION "share/state_space_controllers/cmake"
  NAMESPACE state_space_filters::
  FILE state_space_filtersTargets.cmake
)


