include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED COMPONENTS core)
include("${CMAKE_CURRENT_LIST_DIR}/eigen_matrix_utilsTargets.cmake")
