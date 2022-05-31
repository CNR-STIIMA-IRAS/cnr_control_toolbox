cmake_minimum_required(VERSION 3.0.2)
project(frequency_identification)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)
###################
## PRELIMINARIES ##
###################
cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS
  cnr_logger
  state_space_systems
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES frequency_identification
  CATKIN_DEPENDS cnr_logger state_space_ros roscpp sensor_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/frequency_identification.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
cnr_target_compile_options(${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test src/frequency_identification_test.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
cnr_target_compile_options(${PROJECT_NAME}_test)

install(TARGETS ${PROJECT_NAME}_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
