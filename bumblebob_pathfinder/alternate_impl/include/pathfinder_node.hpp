#ifndef PATHFINDERNODE_H
#define PATHFINDERNODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "../submods/delaunator-cpp/include/delaunator.hpp"
#include "../include/mesh.hpp"
#include "../include/pathfinder.hpp"
#include "pathfinder.hpp"
#include "laneAdaption.hpp"

#include <bumblebob_msgs/ConeArray.h>
#include <bumblebob_msgs/Cone.h>
#include <bumblebob_msgs/PointArray.h>
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/ModelStates.h>
#include <sstream>
#include <cstdio>
#include <vector>
#include <opencv2/core/types.hpp>

class PathFinderNode
{
private:
  void debug();
  void publishRaceline();
  void waitForConeMessages();
  void steeringCallback(const std_msgs::Float32& cmd);
  void conesCallback(const bumblebob_msgs::ConeArray::ConstPtr& msg);
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& states);
  void readROSParams();
  void visualizeMeshPoints(std::vector<cv::Point2d> point_array);
  void visualizeRaceLine(std::vector<cv::Point2d> raceline);

  std::vector<cv::Point2d> mesh_points_;

  std::vector<bumblebob_msgs::Cone> coneArray_;
  std::vector<cv::Point2d> conePositions_;
  std::vector<cv::Point2d> yellow_cones_;
  std::vector<cv::Point2d> blue_cones_;
  std::vector<cv::Point2d> previous_waypoints_;
  std::vector<cv::Point2d> raceline_;

  double steering_cmd_;
  double vx_;
  cv::Point2d vehicle_position_;

  // rosparams
  std::string pathfinder_path_;
  std::string visualization_markers_;
  std::string raceline_topic_;

  std::string cones_sub_;
  std::string states_sub_;
  std::string steer_sub_;

  double threshold_;
  double ppm_;
  double density_;
  bool debug_mode_;

  // ros
  ros::NodeHandle node_;
  ros::Rate loop_rate_;
  ros::Subscriber cones_subscriber_;
  ros::Subscriber model_states_subscriber_;
  ros::Subscriber steering_cmd_sub_;
  ros::Publisher mesh_pub_;
  ros::Publisher path_pub_;
  ros::Publisher raceline_pub_;
  ros::Publisher pub_steering_;

public:
  PathFinderNode();
  void initialize();
  void updateLoop();
};
#endif