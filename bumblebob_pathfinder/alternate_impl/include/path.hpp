#ifndef PATH_H
#define PATH_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../submods/delaunator-cpp/include/delaunator.hpp"

#include <bumblebob_msgs/ConeArray.h>
#include <bumblebob_msgs/Cone.h>
#include <bumblebob_msgs/PointArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <sstream>
#include <cstdio>
#include <vector>

class Path
{
public:
  double weight;
  std::vector<std::array<double, 2>> nodes;

  Path()
  {
    this->weight = 0;
  }

  Path(double weight, std::vector<std::array<double, 2>> nodes)
  {
    this->weight = weight;
    this->nodes = nodes;
  }
};
#endif