/**
 *  @file PurePursuitNode.hpp
 *
 *  @author Dominik Prossel
 *  @version 1.0
 */

#pragma once
// ROS
#include <ros/ros.h>
#include "bumblebob_msgs/PointArray.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

namespace bumblebob_pure_pursuit
{
class PurePursuitNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_handle the ROS node handle.
   */
  PurePursuitNode(ros::NodeHandle& node_handle);

  /**
   * @brief Runs the cone fusion node.
   */
  void run();

private:
  void pathCallback(const bumblebob_msgs::PointArray& msg);

  void targetCallback(const geometry_msgs::PointStamped& point);

  /**
   * @brief Reads and verifies the ROS parameters.
   *
   * @return true if successful.
   */
  bool readParameters();

  static double norm(double x, double y);

  ros::NodeHandle node_handle_;
  std::string path_topic_;
  std::string steering_topic_;
  std::string target_point_topic_;
  ros::Publisher steering_pub_;
  ros::Publisher target_point_pub_;
  ros::Subscriber target_point_sub_;
  ros::Subscriber path_sub_;
  double target_dist_;
  double dist_thresh_;
  geometry_msgs::Point target_;
  double steering_angle_;
};
}  // namespace bumblebob_pure_pursuit