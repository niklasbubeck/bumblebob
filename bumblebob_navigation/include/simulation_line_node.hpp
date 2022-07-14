#ifndef SIMULATIONLINENODE_H
#define SIMULATIONLINENODE_H

#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "bumblebob_msgs/ConeArray.h"
#include <opencv2/core/types.hpp>

class SimulationLineNode
{
public:
  SimulationLineNode();
  void cameraConeCallback(const bumblebob_msgs::ConeArray::ConstPtr& cone_array);

  std::vector<cv::Point2d> bluecones_;
  std::vector<cv::Point2d> yellowcones_;
  std::vector<cv::Point2d> orangecones_;
  std::vector<cv::Point2d> bigorangecones_;

  // std::vector<cv::Point2d> racingline_;
  // std::vector<cv::Point2d> blueline_;
  // std::vector<cv::Point2d> yellowline_;
};

#endif
