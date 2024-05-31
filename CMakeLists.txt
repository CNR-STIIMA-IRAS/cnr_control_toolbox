#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)
project(cnr_control_toolbox)

option(BUILD_EXAMPLES ON)
option(ENABLE_TESTING           "ENABLE TESTING" ON)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

find_package(Eigen3 REQUIRED COMPONENTS Core Dense  Geometry)


# Add gtest based cpp test target and link libraries
if(ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

if(ENABLE_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  include(CTest)
endif()

add_subdirectory(eigen_matrix_utils)
add_subdirectory(state_space_systems)
add_subdirectory(kinematics_filters)
add_subdirectory(state_space_controllers)
add_subdirectory(state_space_filters)


message(${CMAKE_CURRENT_BINARY_DIR})
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/cnr_control_toolboxConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

install(FILES
  ${CMAKE_SOURCE_DIR}/cmake_config/cnr_control_toolboxConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cnr_control_toolboxConfigVersion.cmake
  DESTINATION "share/cnr_control_toolbox/cmake")


# make uninstall
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)

