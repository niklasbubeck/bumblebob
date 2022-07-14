#include <iostream>
#include <string>
#include <math.h>

#include "../include/raceline_calculator_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "raceline_calculator");

  RacelineCalculatorNode rcn;
  rcn.initialize();
  rcn.updateLoop();
  return 0;
}