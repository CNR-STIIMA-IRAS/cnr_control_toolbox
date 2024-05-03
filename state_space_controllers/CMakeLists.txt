
add_library(state_space_controllers INTERFACE)
target_include_directories(state_space_controllers
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
  )

target_link_libraries(state_space_controllers
  INTERFACE state_space_systems::state_space_systems
)

add_library(state_space_controllers::state_space_controllers ALIAS state_space_controllers)


#############
## Testing ##
#############

add_executable(test_controllers
   test/test_controllers.launch test/test_pid.cpp)
target_link_libraries(test_controllers
  PUBLIC
  state_space_controllers::state_space_controllers
   GTest::gtest_main
)


target_compile_options(test_controllers PUBLIC $<$<CONFIG:Release>:-Ofast >)
gtest_discover_tests(test_controllers)
add_test(NAME test_controllers
  COMMAND test_controllers)


if(ENABLE_COVERAGE_TESTING)
  set(COVERAGE_EXCLUDES "*/state_space_controllers/test*")
  add_code_coverage(
    NAME coverage_report
    DEPENDENCIES test_controllers
  )
endif()


#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/state_space_controllers
  DESTINATION include)


install(EXPORT state_space_controllersTargets
  DESTINATION "share/cnr_control_toolbox/cmake"
  NAMESPACE state_space_controllers::
  FILE state_space_controllersTargets.cmake
)
