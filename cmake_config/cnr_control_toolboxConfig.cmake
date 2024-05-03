include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED COMPONENTS core)
find_dependency(Boost REQUIRED COMPONENTS date_time filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp IMPORTED_TARGET)

include("${CMAKE_CURRENT_LIST_DIR}/cnr_control_toolboxTargets.cmake")
