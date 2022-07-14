#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../submods/delaunator-cpp/include/delaunator.hpp"
#include "../../bumblebob_navigation/src/splines.hpp"

#include <bumblebob_msgs/ConeArray.h>
#include <bumblebob_msgs/Cone.h>
#include <bumblebob_msgs/PointArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <sstream>
#include <cstdio>
#include <vector>
#include <opencv2/core/types.hpp>

class PathFinder
{
private:
  double ppm_;
  std::vector<cv::Point2d> cone_positions_;
  std::vector<cv::Point2d> blue_cones_;
  std::vector<cv::Point2d> yellow_cones_;

public:
  PathFinder(std::vector<cv::Point2d> cone_positions, std::vector<cv::Point2d> blue_cones,
             std::vector<cv::Point2d> yellow_cones, double ppm);
  std::vector<cv::Point2d> middleLane();
};

#endif