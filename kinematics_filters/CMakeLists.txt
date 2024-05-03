add_library(kinematics_filters INTERFACE)
target_include_directories(kinematics_filters
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

target_link_libraries(kinematics_filters
  INTERFACE state_space_systems::state_space_systems
)

add_library(kinematics_filters::kinematics_filters ALIAS kinematics_filters)


if(BUILD_EXAMPLES)
  add_executable(kinematics_filters_example example/example.cpp)

  target_link_libraries(kinematics_filters
    INTERFACE kinematics_filters::kinematics_filters
  )
endif()


#############
## Install ##
#############

# Mark cpp header files for installation
install(DIRECTORY include/kinematics_filters
  DESTINATION include)

install(
  TARGETS kinematics_filters
  EXPORT kinematics_filtersTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(EXPORT kinematics_filtersTargets
  DESTINATION "share/cnr_control_toolbox/cmake"
  NAMESPACE kinematics_filters::
  FILE kinematics_filtersTargets.cmake
)

