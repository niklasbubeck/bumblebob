cmake_minimum_required(VERSION 2.8.3)
project(bumblebob_lateral_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  bumblebob_msgs
  dynamic_reconfigure
)

## add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  config/Dynamic.cfg
)


find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()





## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )




## Declare c++ library
add_library(Splines ../bumblebob_navigation/src/splines.cpp)
target_compile_features(Splines PUBLIC cxx_generalized_initializers)
add_library(Raceline ../bumblebob_navigation/src/raceline.cpp)
target_compile_features(Raceline PUBLIC cxx_generalized_initializers)
add_library(Feedback src/feedback.cpp)
target_compile_features(Feedback PUBLIC cxx_auto_type)
add_library(FeedForward src/feedforward.cpp)
target_compile_features(FeedForward PUBLIC cxx_generalized_initializers)
add_library(LateralROSNode src/lateral_kinematic_ROS_node.cpp)

add_dependencies(LateralROSNode ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Declare C++ executable
add_executable(lat_kin_controller src/main.cpp)

## Declare dependencies
add_dependencies(lat_kin_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## link target
target_link_libraries(lat_kin_controller ${catkin_LIBRARIES} FeedForward Feedback LateralROSNode)
target_link_libraries(Raceline ${catkin_LIBRARIES} Splines)
target_link_libraries(Feedback ${catkin_LIBRARIES})
target_link_libraries(FeedForward ${catkin_LIBRARIES})
target_link_libraries(LateralROSNode ${catkin_LIBRARIES} ${OpenCV_LIBS} FeedForward Feedback)



if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_feedback test/test_feedback.test test/test_feedback.cpp)
  add_dependencies(test_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_feedback ${catkin_LIBRARIES} ${OpenCV_LIBS} Feedback FeedForward Raceline)
  add_rostest_gtest(test_feedforward test/test_feedforward.test test/test_feedforward.cpp)
  add_dependencies(test_feedforward ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_feedforward ${catkin_LIBRARIES} ${OpenCV_LIBS} FeedForward Raceline)
endif()
