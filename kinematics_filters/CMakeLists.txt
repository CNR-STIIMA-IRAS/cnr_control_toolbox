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
option(BUILD_EXAMPLE            "COMPILE THE EXAMPLE" OFF)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################

#############################
## DEPENDANCIES            ##
#############################
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS eigen_matrix_utils)
  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS eigen_matrix_utils
  ) 
else()
  find_package(eigen_matrix_utils REQUIRED)
  find_package(state_space_systems REQUIRED )
endif()

cnr_install_directories(USE_ROS1 
  CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
#############################
## END DEPENDANCIES        ##
#############################

#################
## Build       ##
#################
list(APPEND DEPENDENCIES_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
if(USE_ROS1)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
else()
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${eigen_matrix_utils_INCLUDE_DIRS} ${state_space_systems_INCLUDE_DIRS})
endif()

list(APPEND BUILD_INTERFACE_INCLUDE_DIRS ${DEPENDENCIES_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
list(APPEND INSTALL_INTERFACE_INCLUDE_DIRS include ${DEPENDENCIES_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} src/kinematics_fitlers.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC 
  "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>")

if(BUILD_EXAMPLES)
  add_executable(${PROJECT_NAME}_example example/example.cpp)
  target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})
endif()
#################
## END - Build ##
#################



#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CNR_INSTALL_INCLUDE_DIR}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CNR_INSTALL_LIB_DIR}
        LIBRARY DESTINATION ${CNR_INSTALL_LIB_DIR}
        RUNTIME DESTINATION ${CNR_INSTALL_BIN_DIR}
)

if(USE_ROS1)
  install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
endif()
###################
## END - Install ##
###################

##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################
if(NOT USE_ROS1)
  
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_kinematics_filters
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # export targets
  set(export_targets ${export_targets};kinematics_filters_utilities)
  set(export_targets ${export_targets};kinematics_filters_server)
  export(EXPORT export_kinematics_filters
    FILE "${PROJECT_BINARY_DIR}/export_kinematics_filters.cmake")

  # Create the kinematics_filtersConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(kinematics_filtersConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/kinematics_filtersConfig.cmake" @ONLY)

  # Create the kinematics_filtersConfigVersion.cmake
  configure_file(kinematics_filtersConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/kinematics_filtersConfigVersion.cmake" @ONLY)
  
  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/kinematics_filtersConfig.cmake"
    "${PROJECT_BINARY_DIR}/kinematics_filtersConfigVersion.cmake"
    DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev
  )

  install(EXPORT export_kinematics_filters
    DESTINATION "share/${PROJECT_NAME}/cmake"
    FILE export_kinematics_filters.cmake
    COMPONENT dev
  )
endif()
##################################
## CMAKE CONFIG (IF NOT CATKIN) ##
##################################

#############
## Testing ##
#############
if(ENABLE_TESTING)
  if(USE_ROS1)
    add_rostest_gtest(test_filters test/test_filters.launch test/test_filters.cpp)
    target_link_libraries(test_filters ${PROJECT_NAME} ${rostest_LIBRARIES} ${roscpp_LIBRARIES})
    target_compile_options(test_filters PUBLIC $<$<CONFIG:Release>:-Ofast >)

    if(ENABLE_COVERAGE_TESTING)
      set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
      add_code_coverage(
        NAME coverage_report
        DEPENDENCIES test_filters
      )
    endif()
  endif()
endif()
###################
## END - Testing ##
###################


