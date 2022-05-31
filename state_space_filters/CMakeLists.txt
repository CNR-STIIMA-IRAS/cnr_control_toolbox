cmake_minimum_required(VERSION 3.1)
project(state_space_filters)


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS eigen_matrix_utils state_space_systems)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()


catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS eigen_matrix_utils state_space_systems
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#############
## Install ##
#############

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roscpp REQUIRED)

  include_directories(${rostest_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
  add_rostest_gtest(filtered_values_test test/test_filtered_values.launch test/test_filtered_values.cpp)
  target_link_libraries(filtered_values_test ${catkin_LIBRARIES} ${rostest_LIBRARIES} ${roscpp_LIBRARIES})
  cnr_target_compile_options(filtered_values_test)
  
  add_rostest_gtest(${PROJECT_NAME}_test_filters test/test_filters.launch test/test_filters.cpp)
  add_dependencies(${PROJECT_NAME}_test_filters ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_test_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES} ${roscpp_LIBRARIES})
  cnr_target_compile_options(${PROJECT_NAME}_test_filters)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES filtered_values_test ${PROJECT_NAME}_test_filters
    )
  endif()
endif()

