
#############################
## TARGETS                 ##
#############################
add_library(eigen_matrix_utils SHARED src/eigen_matrix_utils/eigen_matrix_utils.cpp)
target_include_directories(eigen_matrix_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> 
)
target_link_libraries(eigen_matrix_utils Eigen3::Eigen)

add_library(eigen_matrix_utils::eigen_matrix_utils ALIAS eigen_matrix_utils)


#############################
## END TARGETS             ##
#############################


#############################
## INSTALL                 ##
#############################


install(
  DIRECTORY include/eigen_matrix_utils
  DESTINATION include
)

install(
  TARGETS eigen_matrix_utils
  EXPORT eigen_matrix_utilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT eigen_matrix_utilsTargets
  DESTINATION "share/eigen_matrix_utils/cmake"
  NAMESPACE eigen_matrix_utils::
  FILE eigen_matrix_utilsTargets.cmake
)

#############################
## END INSTALL             ##
#############################
