#include <iostream>
#include <string>
#include <math.h>

#include "../include/car_constants.hpp"
#include "../include/feedforward.hpp"
#include "../include/feedback.hpp"
#include "../include/lateral_kinematic_ROS_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematic_lateral_controller");

  LateralROSNode lrn;
  lrn.initialize();
  lrn.updateLoop();
  return 0;
}