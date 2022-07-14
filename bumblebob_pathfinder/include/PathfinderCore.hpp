#pragma once

#include "delaunator.hpp"
#include "PathfinderHelper.hpp"

#include <bumblebob_msgs/Cone.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <bumblebob_pathfinder/PathfinderConfig.h>

#include <vector>
#include <array>
#include <cmath>
#include <limits>

#define CUTOFFWEIGHT 10000000

namespace bumblebob_pathfinder
{
enum Bias
{
  none,
  straight,
  right,
  left
};

class Path
{
public:
  Path();
  Path(double weight, std::vector<std::array<double, 2>> nodes);
  double weight;
  std::vector<std::array<double, 2>> nodes;
  double finish_line_crossed = 0;
  double blues = 0;
  double yellows = 0;
  double oranges = 0;
  double bb_crossed = 0;
  double yy_crossed = 0;
  double oo_crossed = 0;
};

class PathfinderCore
{
public:
  /**
   * @brief Constructor.
   *
   */
  PathfinderCore(ros::NodeHandle& nh);

  /**
   * @brief Destructor.
   */
  virtual ~PathfinderCore() = default;

  /**
   * @brief Returns calculated path mesh.
   */
  delaunator::Delaunator getMesh(const delaunator::Delaunator& d, const std::vector<bumblebob_msgs::Cone>& coneArray);
  /**
   * @brief Calculate and return the best path on pathMesh, starting at carPos.
   */
  Path findPath(const delaunator::Delaunator& pathMesh, Bias bias);

  /**
   * @brief Print path mesh points for debugging.
   */
  void printPathMeshPoints();

  /**
   * @brief Setting parameters from dynamic reconfigure.
   */
  void reconfigure(bumblebob_pathfinder::PathfinderConfig& config, uint32_t& level);

private:
  typedef std::array<double, 5> (PathfinderCore::*weightFuncPtr)(const std::array<double, 2>&,
                                                                 const std::array<double, 2>&,
                                                                 const std::array<double, 2>&);

  void calcPathPointCoords(const delaunator::Delaunator& d, const std::vector<bumblebob_msgs::Cone>& coneArray);

  std::vector<std::array<double, 2>> getNextPoints(const std::array<double, 2>& curPt,
                                                   const std::vector<std::array<double, 6>>& pathMeshTriangles,
                                                   std::vector<std::array<double, 2>>& ignoreList);

  Path getNextSection(unsigned remainingLength, const std::array<double, 2>& prevPt, const std::array<double, 2>& curPt,
                      const std::vector<std::array<double, 2>>& nextPts,
                      const std::vector<std::array<double, 6>>& pathMeshTriangles,
                      std::vector<std::array<double, 2>> ignoreList, weightFuncPtr wf);

  std::array<double, 5> coneTypeWeight(const std::array<double, 2>& point, const std::array<double, 2>& prevPt);

  std::array<double, 5> calcPathWeight(const std::array<double, 2>& prevPt, const std::array<double, 2>& curPt,
                                       const std::array<double, 2>& nextPt);

  std::array<double, 5> calcPathWeight_prefStraight(const std::array<double, 2>& prevPt,
                                                    const std::array<double, 2>& curPt,
                                                    const std::array<double, 2>& nextPt);

  std::array<double, 5> calcPathWeight_prefRight(const std::array<double, 2>& prevPt,
                                                 const std::array<double, 2>& curPt,
                                                 const std::array<double, 2>& nextPt);

  std::array<double, 5> calcPathWeight_prefLeft(const std::array<double, 2>& prevPt, const std::array<double, 2>& curPt,
                                                const std::array<double, 2>& nextPt);

  ros::NodeHandle nh;
  ros::Publisher finish_line_pub;
  std::string finish_line_topic;
  std_msgs::String finish_line_msg;

  int seg_seg_angle_normalizer;
  std::vector<double> pathPointCoords;
  std::vector<Cone> pathPointCones;

  std::array<double, 2> carPos;  //{ {0, 0} };
  int same_type_limit;
  int section_length;
  int section_stride;
  int seg_seg_min_angle;
  double segment_max_len;
  double finish_line_trigger_radius;
  double finish_line_trigger_corridor;
};

}  // namespace bumblebob_pathfinder