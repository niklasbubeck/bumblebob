#pragma once

#include "PathfinderCore.hpp"
#include "Visualizer.hpp"

#include <bumblebob_msgs/Bias.h>
#include <bumblebob_msgs/Cone.h>
#include <bumblebob_msgs/ConeArray.h>
#include <bumblebob_msgs/PointArray.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <bumblebob_pathfinder/PathfinderConfig.h>

#include <sstream>
#include <cstdio>
#include <vector>
#include <array>
#include <chrono>
#include <iostream>

namespace bumblebob_pathfinder
{
class PathfinderNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_handle the ROS node handle.
   */
  PathfinderNode(ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  virtual ~PathfinderNode() = default;

  /**
   * @brief Runs the pathfinder node.
   */
  void run();

  /**
   * @brief Setting parameters from dynamic reconfigure.
   */
  void reconfigure(bumblebob_pathfinder::PathfinderConfig& config, uint32_t& level);

private:
  /**
   * @brief Set directional bias.
   */
  void biasCallback(const bumblebob_msgs::Bias::ConstPtr& msg);

  /**
   * @brief Convert incoming Cone coordinates from ConeArray message to input vector for delaunator and fill coneArray.
   */
  void conesCallback(const bumblebob_msgs::ConeArray::ConstPtr& msg);

  /**
   * @brief Publish PointArray generated from path p.
   */
  void publishPath(Path& p);

  /**
   * @brief Calculate and publish target point for controller.
   */
  geometry_msgs::Point getTargetPoint(Path& p);

  std::string base_link_name;
  std::string finish_line_detected_topic;
  ros::NodeHandle nh;
  ros::Publisher path_pub;
  ros::Publisher target_pub;
  ros::Subscriber bias_sub;
  ros::Subscriber cones_sub;

  std::string path_topic;
  std::string target_topic;
  std::string bias_topic;
  std::string cones_topic;

  PathfinderCore core;
  Visualizer viz;

  int received;
  double prev_target_distance;
  std::vector<bumblebob_msgs::Cone> coneArray;
  std::vector<double> coneCoords;

  int direction_bias;
  double target_distance_min;
  double target_distance_max;
  double target_distance_falloff;
  double target_distance_scaler;
  double prev_target_dist_influence;
};

}  // namespace bumblebob_pathfinder