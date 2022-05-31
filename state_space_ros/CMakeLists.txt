cmake_minimum_required(VERSION 3.1)
project(state_space_ros)


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS
  state_space_systems
  state_space_controllers
  state_space_filters
  rosparam_utilities
)

find_package(Eigen3 REQUIRED)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS state_space_systems state_space_controllers state_space_filters
#  DEPENDS EIGEN3
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(eigen_matrix_utils REQUIRED)
  find_package(Eigen3 REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}_filtered_values_test test/test_filtered_values.launch test/test_filtered_values.cpp)
  add_dependencies(${PROJECT_NAME}_filtered_values_test ${catkin_EXPORTED_TARGETS})
  target_include_directories(${PROJECT_NAME}_filtered_values_test PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
  target_link_libraries(${PROJECT_NAME}_filtered_values_test ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
  cnr_target_compile_options(${PROJECT_NAME}_filtered_values_test)

  add_rostest_gtest(${PROJECT_NAME}_test_filters test/test_filters.launch test/test_filters.cpp)
  add_dependencies(${PROJECT_NAME}_test_filters ${catkin_EXPORTED_TARGETS})
  target_include_directories(${PROJECT_NAME}_test_filters PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_test_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
  cnr_target_compile_options(${PROJECT_NAME}_test_filters)

  add_rostest_gtest(${PROJECT_NAME}_test_state_space_matrix test/test_state_space_matrix.launch test/test_state_space_matrix.cpp)
  add_dependencies(${PROJECT_NAME}_test_state_space_matrix ${catkin_EXPORTED_TARGETS})
  target_include_directories(${PROJECT_NAME}_test_state_space_matrix PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
  target_link_libraries(${PROJECT_NAME}_test_state_space_matrix ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
  cnr_target_compile_options(${PROJECT_NAME}_test_state_space_matrix)
  
  add_rostest_gtest(${PROJECT_NAME}_test_controllers test/test_controllers.launch test/test_pid.cpp)
  add_dependencies(${PROJECT_NAME}_test_controllers ${catkin_EXPORTED_TARGETS})
  target_include_directories(${PROJECT_NAME}_test_controllers PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
  target_link_libraries(${PROJECT_NAME}_test_controllers ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
  cnr_target_compile_options(${PROJECT_NAME}_test_controllers)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES ${PROJECT_NAME}_filtered_values_test 
      #${PROJECT_NAME}_test_controllers
      #                ${PROJECT_NAME}_test_filters ${PROJECT_NAME}_test_state_space_matrix
    )
  endif()
endif()
