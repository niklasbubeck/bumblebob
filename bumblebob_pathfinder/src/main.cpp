/*
1. Generate Delaunay triangulation mesh from cone array
2. Generate graph from Delaunay mesh middle points
3. Find correct path on graph along the supposed racetrack utilizing a weight function for each possible path
*/

#include "PathfinderNode.hpp"
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <bumblebob_pathfinder/PathfinderConfig.h>

void dynamicReconfigureCallback(bumblebob_pathfinder::PathfinderConfig& config, uint32_t level,
                                bumblebob_pathfinder::PathfinderNode* pf)
{
  pf->reconfigure(config, level);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathfinder");

  ros::NodeHandle nh("~");

  bumblebob_pathfinder::PathfinderNode pathfinder(nh);

  dynamic_reconfigure::Server<bumblebob_pathfinder::PathfinderConfig> reconfigureServer;
  reconfigureServer.setCallback(boost::bind(&dynamicReconfigureCallback, _1, _2, &pathfinder));

  ros::spin();

  return 0;
}