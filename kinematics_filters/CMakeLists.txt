cmake_minimum_required(VERSION 3.1)
project(kinematics_filters)

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_EXAMPLES ON)

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

if(BUILD_EXAMPLES)
  add_executable(${PROJECT_NAME}_example example/example.cpp)
endif()

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

  add_rostest_gtest(test_filters test/test_filters.launch test/test_filters.cpp)
  add_dependencies(test_filters ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES} ${roscpp_LIBRARIES})
  target_compile_options(test_filters PUBLIC $<$<CONFIG:Release>:-Ofast >)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES test_filters
    )
  endif()
endif()

