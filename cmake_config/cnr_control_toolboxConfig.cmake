include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED COMPONENTS core)

include("${CMAKE_CURRENT_LIST_DIR}/eigen_matrix_utilsTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/state_space_systemsTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/state_space_filtersTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/state_space_controllersTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/kinematics_filtersTargets.cmake")
