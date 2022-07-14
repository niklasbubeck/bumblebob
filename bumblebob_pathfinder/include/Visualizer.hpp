#pragma once

#include "delaunator.hpp"
#include "PathfinderCore.hpp"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace bumblebob_pathfinder
{
class Visualizer
{
public:
  /**
   * @brief Constructor.
   *
   * @param node_handle the ROS node handle.
   */
  Visualizer(ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  virtual ~Visualizer() = default;

  /**
   * @brief
   */
  void visualize_coneMesh(delaunator::Delaunator& d, std::vector<bumblebob_msgs::Cone>& coneArray);

  void visualize_pathMesh(delaunator::Delaunator& pathMesh);

  void visualize_bestPath_and_target(Path& bestPath, geometry_msgs::Point target);

private:
  ros::Publisher cone_mesh_pub;
  ros::Publisher path_mesh_pub;
  ros::Publisher best_path_pub;

  std::string cone_mesh_topic;
  std::string path_mesh_topic;
  std::string best_path_topic;
  std::string base_link_name;

  visualization_msgs::Marker points;
  visualization_msgs::Marker target;
  visualization_msgs::Marker cone_undefined;
  visualization_msgs::Marker cone_blue;
  visualization_msgs::Marker cone_yellow;
  visualization_msgs::Marker cone_orange;
  visualization_msgs::Marker line_list_conemesh;
  visualization_msgs::Marker line_list_pathmesh;
  visualization_msgs::Marker line_list_path;
};

}  // namespace bumblebob_pathfinder