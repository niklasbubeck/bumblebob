cmake_minimum_required(VERSION 2.8.3)
project(bumblebob_pathfinder)

## Use C++14
#add_definitions(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  bumblebob_msgs
  tf2_bumblebob_msgs
  dynamic_reconfigure
)

## add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  config/Pathfinder.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    _${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    dynamic_reconfigure
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_core
  src/PathfinderNode.cpp
  src/PathfinderCore.cpp
  src/PathfinderHelper.cpp
  src/Visualizer.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_aggregate_default_initializers)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_core
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## Unit tests
## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-unit-test
#  test/unit/unit_test_bumblebob_pathfinder.cpp
#  test/unit/PathfinderAlgorithmTest.cpp)
#endif()

#if(TARGET ${PROJECT_NAME}-unit-test)
#  target_link_libraries(${PROJECT_NAME}-unit-test ${PROJECT_NAME}_core)
#endif()

## Integration Tests
#find_package(rostest REQUIRED)
#add_rostest_gtest(${PROJECT_NAME}-integration-test
#test/integration/bumblebob_pathfinder.test
#test/integration/integration_test_bumblebob_pathfinder.cpp
#test/integration/PathfinderIntegrationTest.cpp)

#if(TARGET ${PROJECT_NAME}-integration-test)
#  target_link_libraries(${PROJECT_NAME}-integration-test ${catkin_LIBRARIES})
#endif()
