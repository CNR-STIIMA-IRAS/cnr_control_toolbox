#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

cnr_set_flags()
#############################
## END - PRELIMINARIES     ##
#############################

#############################
## OPTIONS                 ##
#############################
option(USE_ROS1                 "ROS SUPPORT" ON)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################

if(USE_ROS1)

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
  endif(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)



  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS state_space_systems state_space_controllers state_space_filters
  #  DEPENDS EIGEN3
  )

  ###########
  ## Build ##
  ###########
  include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})


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
    target_include_directories(${PROJECT_NAME}_filtered_values_test PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
    target_link_libraries(${PROJECT_NAME}_filtered_values_test ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_filtered_values_test PUBLIC $<$<CONFIG:Release>:-Ofast >)

    add_rostest_gtest(${PROJECT_NAME}_test_filters test/test_filters.launch test/test_filters.cpp)
    add_dependencies(${PROJECT_NAME}_test_filters ${catkin_EXPORTED_TARGETS})
    target_include_directories(${PROJECT_NAME}_test_filters PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_test_filters ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_test_filters PUBLIC $<$<CONFIG:Release>:-Ofast >)

    add_rostest_gtest(${PROJECT_NAME}_test_state_space_matrix test/test_state_space_matrix.launch test/test_state_space_matrix.cpp)
    add_dependencies(${PROJECT_NAME}_test_state_space_matrix ${catkin_EXPORTED_TARGETS})
    target_include_directories(${PROJECT_NAME}_test_state_space_matrix PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
    target_link_libraries(${PROJECT_NAME}_test_state_space_matrix ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_test_state_space_matrix PUBLIC $<$<CONFIG:Release>:-Ofast >)
    
    add_rostest_gtest(${PROJECT_NAME}_test_controllers test/test_controllers.launch test/test_pid.cpp)
    add_dependencies(${PROJECT_NAME}_test_controllers ${catkin_EXPORTED_TARGETS})
    target_include_directories(${PROJECT_NAME}_test_controllers PUBLIC ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
    target_link_libraries(${PROJECT_NAME}_test_controllers ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_options(${PROJECT_NAME}_test_controllers PUBLIC $<$<CONFIG:Release>:-Ofast >)

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

endif(USE_ROS1) 