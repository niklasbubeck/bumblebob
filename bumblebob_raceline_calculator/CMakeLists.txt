cmake_minimum_required(VERSION 3.0.2)
project(bumblebob_raceline_calculator)

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
# generate_dynamic_reconfigure_options(
#   config/Dynamic.cfg
# )


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

add_library(VelocityProfile src/velocity_profile.cpp)
add_library(StatesCalculations src/states_calculations.cpp)
target_compile_features(StatesCalculations PUBLIC cxx_nonstatic_member_init)
add_library(RacelineCalculatorNode src/raceline_calculator_node.cpp)
add_library(RacelineOptimizer src/raceline_optimizer.cpp)
add_library(Splines_Opt ../bumblebob_navigation/src/splines.cpp)
target_compile_features(Splines_Opt PUBLIC cxx_generalized_initializers)
add_library(Raceline_Opt ../bumblebob_navigation/src/raceline.cpp)
target_compile_features(Raceline_Opt PUBLIC cxx_generalized_initializers)
## add_library(Test_msg scripts/send_test_msg.cpp)

add_dependencies(RacelineCalculatorNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare C++ executable 
add_executable(raceline_optimization src/main.cpp)
add_executable(send_test_msg scripts/send_test_msg.cpp)
target_compile_features(send_test_msg PUBLIC cxx_generalized_initializers)

## Declare dependencies
add_dependencies(raceline_optimization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(send_test_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## link target 
target_link_libraries(send_test_msg ${catkin_LIBRARIES})
target_link_libraries(raceline_optimization ${catkin_LIBRARIES} VelocityProfile StatesCalculations RacelineCalculatorNode)
target_link_libraries(VelocityProfile ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(StatesCalculations ${catkin_LIBRARIES} VelocityProfile)
target_link_libraries(RacelineCalculatorNode ${catkin_LIBRARIES} ${OpenCV_LIBS} VelocityProfile StatesCalculations RacelineOptimizer Raceline_Opt)
target_link_libraries(RacelineOptimizer ${catkin_LIBRARIES} VelocityProfile Splines_Opt)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(test_feedback test/test_feedback.test test/test_feedback.cpp)
#   target_link_libraries(test_feedback ${catkin_LIBRARIES} ${OpenCV_LIBS} Feedback FeedForward Raceline)
#   add_rostest_gtest(test_feedforward test/test_feedforward.test test/test_feedforward.cpp)
#   target_link_libraries(test_feedforward ${catkin_LIBRARIES} ${OpenCV_LIBS} FeedForward Raceline)
# endif()
