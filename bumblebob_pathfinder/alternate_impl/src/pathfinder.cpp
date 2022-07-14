#include "pathfinder.hpp"

PathFinder::PathFinder(std::vector<cv::Point2d> cone_positions, std::vector<cv::Point2d> blue_cones,
                       std::vector<cv::Point2d> yellow_cones, double ppm)
{
  CatmullRomSplines crs_blue(blue_cones, false);
  CatmullRomSplines crs_yellow(yellow_cones, false);

  cone_positions_ = cone_positions;
  blue_cones_ = crs_blue.getSplineLine(ppm);
  yellow_cones_ = crs_yellow.getSplineLine(ppm);
}

std::vector<cv::Point2d> PathFinder::middleLane()
{
  int size = std::min(blue_cones_.size(), yellow_cones_.size());
  std::vector<cv::Point2d> middlepoints = {};
  for (int i = 0; i < size; i++)
  {
    middlepoints.push_back(cv::Point2d((blue_cones_[i] + yellow_cones_[i]) / 2));
  }
  return middlepoints;
}