#include <iostream>
#include <string>
#include <math.h>

#include "../include/pathfinder_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Pathfinder");

  PathFinderNode pfn;
  pfn.initialize();
  pfn.updateLoop();
  return 0;
}