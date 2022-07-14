#ifndef MESH_H
#define MESH_H

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

class Mesh
{
private:
  double ppm_;
  double threshold_;

  int density_;
  std::vector<cv::Point2d> mesh_points_;
  void deletePointsBetween();
  std::vector<double> pointToDouble(std::vector<cv::Point2d> point_array);
  std::vector<cv::Point2d> doubleToPoint(std::vector<double> double_array);
  int calculateNearestPointIndex(std::vector<cv::Point2d> point_array, cv::Point2d point);
  double calculateNearestPointDistance(std::vector<cv::Point2d> point_array, cv::Point2d point);
  void deletePointsOutside();

public:
  Mesh(std::vector<cv::Point2d> cone_positions, std::vector<cv::Point2d> blue_cones,
       std::vector<cv::Point2d> yellow_cones, double density, double ppm, double threshold);
  void setDensity(int newValue);
  std::vector<cv::Point2d> getMeshPoints();
  void calculateMesh();

  std::vector<cv::Point2d> middlepoints_ = {};
  std::vector<cv::Point2d> yellow_cones_;
  std::vector<cv::Point2d> blue_cones_;
};

#endif