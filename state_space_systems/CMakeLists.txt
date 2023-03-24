#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}"LESS 3.0)
  project(${extracted_name})
else()
  project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
endif()

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

find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS eigen_matrix_utils)   
  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS eigen_matrix_utils
    DEPENDS
  )
else()
  find_package(eigen_matrix_utils REQUIRED)
endif()

## Add gtest based cpp test target and link libraries
if(ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

###########
## Build ##
###########
add_library(${PROJECT_NAME} src/state_space_systems.cpp)
if(NOT USE_ROS1)
  list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${eigen_matrix_utils_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
else()
  list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS} ${Catkin_INCLUDE_DIRS})
  list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${Catkin_INCLUDE_DIRS})
endif()  

target_include_directories(${PROJECT_NAME} PUBLIC 
    "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
    "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

#############
## Install ##
#############
if(NOT USE_ROS1)
  
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_state_space_systems
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # export targets
  set(export_targets ${export_targets};state_space_systems_utilities)
  set(export_targets ${export_targets};state_space_systems_server)
  export(EXPORT export_state_space_systems
    FILE "${PROJECT_BINARY_DIR}/export_state_space_systems.cmake")

  # Create the state_space_systemsConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(state_space_systemsConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/state_space_systemsConfig.cmake" @ONLY)

  # Create the state_space_systemsConfigVersion.cmake
  configure_file(state_space_systemsConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/state_space_systemsConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/state_space_systemsConfig.cmake"
    "${PROJECT_BINARY_DIR}/state_space_systemsConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_state_space_systems
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_state_space_systems.cmake
    COMPONENT dev
  )
endif()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roscpp REQUIRED)

  include_directories(${rostest_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
  add_rostest_gtest(test_state_space_matrix test/test_state_space_matrix.launch test/test_state_space_matrix.cpp)
  add_dependencies(test_state_space_matrix ${catkin_EXPORTED_TARGETS})

  target_link_libraries(test_state_space_matrix ${catkin_LIBRARIES} ${rostest_LIBRARIES} ${roscpp_LIBRARIES})
  target_compile_options(test_state_space_matrix PUBLIC $<$<CONFIG:Release>:-Ofast >)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES test_state_space_matrix
    )
  endif()
endif()
