#include "PathfinderNode.hpp"

#define DEBUG_COORDS false  // print coordinates
#define DEBUG_PATH false    // print path info
#define RVIZ true           // generate RVIZ data
#define TIMING false        // print conesCallback timing

namespace bumblebob_pathfinder
{
PathfinderNode::PathfinderNode(ros::NodeHandle& nh) : nh(nh), core(PathfinderCore(nh)), viz(Visualizer(nh))
{
  ros::param::get("/pathfinder/params/default_bias", direction_bias);
  ros::param::get("/pathfinder/params/target_distance_min", target_distance_min);
  ros::param::get("/pathfinder/params/target_distance_max", target_distance_max);
  ros::param::get("/pathfinder/params/prev_target_dist_influence", prev_target_dist_influence);
  ros::param::get("/pathfinder/params/target_distance_falloff", target_distance_falloff);
  ros::param::get("/pathfinder/params/target_distance_scaler", target_distance_scaler);
  ros::param::get("/pathfinder/topics/pub/path", path_topic);
  ros::param::get("/pathfinder/topics/pub/target", target_topic);
  ros::param::get("/pathfinder/topics/sub/bias", bias_topic);
  ros::param::get("/pathfinder/topics/sub/cones", cones_topic);

  if (target_distance_min < 0)
    target_distance_min = 0;
  if (target_distance_max < 0)
    target_distance_max = 0;
  if (target_distance_max < target_distance_min)
    target_distance_max = target_distance_min;
  if (target_distance_falloff < 0)
    target_distance_falloff = 0;
  if (target_distance_scaler < 0)
    target_distance_scaler = 0;
  if (prev_target_dist_influence < 0)
    prev_target_dist_influence = 0;
  if (prev_target_dist_influence > 1)
    prev_target_dist_influence = 1;

  prev_target_distance = target_distance_min;

  path_pub = nh.advertise<bumblebob_msgs::PointArray>(path_topic, 1);
  target_pub = nh.advertise<geometry_msgs::Point>(target_topic, 1);
  bias_sub = nh.subscribe(bias_topic, 1, &PathfinderNode::biasCallback, this);
  cones_sub = nh.subscribe(cones_topic, 1, &PathfinderNode::conesCallback, this);
}

/* Setting parameters from dynamic reconfigure. */
void PathfinderNode::reconfigure(bumblebob_pathfinder::PathfinderConfig& config, uint32_t& level)
{
  if (config.target_distance_max < config.target_distance_min)
    config.target_distance_max = config.target_distance_min;

  direction_bias = config.direction_bias;
  target_distance_min = config.target_distance_min;
  target_distance_max = config.target_distance_max;
  target_distance_falloff = config.target_distance_falloff;
  target_distance_scaler = config.target_distance_scaler;

  core.reconfigure(config, level);
}

/* Calculate target point for controller. */
geometry_msgs::Point PathfinderNode::getTargetPoint(Path& p)
{
  geometry_msgs::Point target;
  double target_distance = target_distance_max;
  double y_deviation = 0;
  std::vector<double> path_point_distances = { { 0 } };

  // store distance on path for each point and accumulate deviation from middle line (y == 0)
  double distance = 0;
  for (auto pt = p.nodes.begin() + 1; pt != p.nodes.end(); ++pt)
  {
    distance += getDistance(*(pt - 1), *pt);
    path_point_distances.push_back(distance);
    y_deviation += (abs(pt->at(1)) * target_distance_max) / (distance * (1 + target_distance_falloff));
  }

  // set distance on path for target point based on total y_deviation, the more the closer, within min/max
  target_distance *= target_distance_scaler;
  target_distance /= y_deviation;
  if (target_distance < target_distance_min)
    target_distance = target_distance_min;
  else if (target_distance > target_distance_max)
    target_distance = target_distance_max;

  // blend in previous target distance to dampen jitter
  target_distance =
      (1 - prev_target_dist_influence) * target_distance + prev_target_dist_influence * prev_target_distance;
  prev_target_distance = target_distance;

#if DEBUG_PATH
  printf("\ntarget_distance: %f\n", target_distance);
#endif

  // initialize target with furthest path point
  target.x = p.nodes.rbegin()->at(0);
  target.y = p.nodes.rbegin()->at(1);
  target.z = 0;

  // calculate target point x and y based on target_distance and closest interpolated path points
  for (int i = 1; i < path_point_distances.size(); ++i)
  {
    double diff = path_point_distances[i] - target_distance;
    if (diff >= 0)
    {
      double scale = diff / (path_point_distances[i] - path_point_distances[i - 1]);
      target.x = scale * p.nodes[i - 1][0] + (1 - scale) * p.nodes[i][0];
      target.y = scale * p.nodes[i - 1][1] + (1 - scale) * p.nodes[i][1];
      break;
    }
  }

  return target;
}

/* Publish PointArray generated from path p. */
void PathfinderNode::publishPath(Path& p)
{
  bumblebob_msgs::PointArray path_points;
  path_points.position.resize(p.nodes.size());

  for (std::size_t i = 0; i < p.nodes.size(); ++i)
  {
    path_points.position[i].x = p.nodes[i][0];
    path_points.position[i].y = p.nodes[i][1];
    path_points.position[i].z = 0;
  }

  path_pub.publish(path_points);
}

/* Set directional bias. */
void PathfinderNode::biasCallback(const bumblebob_msgs::Bias::ConstPtr& msg)
{
  direction_bias = msg->bias;
}

/* Convert incoming Cone coordinates from ConeArray message to input vector for delaunator and fill coneArray. */
void PathfinderNode::conesCallback(const bumblebob_msgs::ConeArray::ConstPtr& msg)
{
#if TIMING
  auto start_time = std::chrono::steady_clock::now();
#endif

  received = 0;
  coneArray.clear();
  coneCoords.clear();
  coneArray.resize(msg->cones.size());
  coneCoords.resize(msg->cones.size() * 2);

  for (bumblebob_msgs::Cone c : msg->cones)
  {
    if (c.type == coneType::ORANGE)
      c.type = (int8_t)coneType::orange;

    coneArray[received] = c;
    coneCoords[2 * received] = static_cast<double>(c.position.x);
    coneCoords[2 * received + 1] = static_cast<double>(c.position.y);
    ++received;
  }

#if DEBUG_COORDS
  for (std::size_t i = 0; i < received * 2; i += 2)
  {
    printf("Cone %zu: [%f, %f]\n", i / 2 + 1, coneCoords[i], coneCoords[i + 1]);
  }
#endif

  // Create cone mesh via delaunay triangulation.
  delaunator::Delaunator* coneMesh;
  try
  {
    coneMesh = new delaunator::Delaunator(coneCoords);
  }
  catch (std::runtime_error)
  {
    return;
  }

// print all "delaunated" triangles for debugging
#if DEBUG_COORDS
  for (std::size_t i = 0; i < (*coneMesh).triangles.size(); i += 3)
  {
    printf("Cone Triangle %zu: [[%f, %f], [%f, %f], [%f, %f]]\n", i / 3 + 1,
           (*coneMesh).coords[2 * (*coneMesh).triangles[i]],          // tx0
           (*coneMesh).coords[2 * (*coneMesh).triangles[i] + 1],      // ty0
           (*coneMesh).coords[2 * (*coneMesh).triangles[i + 1]],      // tx1
           (*coneMesh).coords[2 * (*coneMesh).triangles[i + 1] + 1],  // ty1
           (*coneMesh).coords[2 * (*coneMesh).triangles[i + 2]],      // tx2
           (*coneMesh).coords[2 * (*coneMesh).triangles[i + 2] + 1]   // ty2
    );
  }
#endif

#if RVIZ
  viz.visualize_coneMesh((*coneMesh), coneArray);
#endif

  // Generate path mesh.
  delaunator::Delaunator pathMesh = core.getMesh((*coneMesh), coneArray);

#if DEBUG_COORDS
  core.printPathMeshPoints();

  // print all "delaunated" path triangles
  for (std::size_t i = 0; i < pathMesh.triangles.size(); i += 3)
  {
    printf("Path Triangle %zu: [[%f, %f], [%f, %f], [%f, %f]]\n", i / 3 + 1,
           pathMesh.coords[2 * pathMesh.triangles[i]],          // tx0
           pathMesh.coords[2 * pathMesh.triangles[i] + 1],      // ty0
           pathMesh.coords[2 * pathMesh.triangles[i + 1]],      // tx1
           pathMesh.coords[2 * pathMesh.triangles[i + 1] + 1],  // ty1
           pathMesh.coords[2 * pathMesh.triangles[i + 2]],      // tx2
           pathMesh.coords[2 * pathMesh.triangles[i + 2] + 1]   // ty2
    );
  }
#endif

#if RVIZ
  viz.visualize_pathMesh(pathMesh);
#endif

  // Determine the best path on the path mesh.
  Path bestPath = core.findPath(pathMesh, static_cast<Bias>(direction_bias));

  // Publish bestPath
  // publishPath(bestPath);

#if DEBUG_PATH
  for (auto it = bestPath.nodes.begin(); it != bestPath.nodes.end(); it++)
  {
    printf("Path point: [%f, %f]\n", (*it)[0], (*it)[1]);
  }
  printf("\nPath weight: %f\n", bestPath.weight);
  printf("\nCones B: %d\nCones Y: %d\nCones O: %d\n", (int)bestPath.blues, (int)bestPath.yellows,
         (int)bestPath.oranges);
  printf("\nB-B: %d\nY-Y: %d\nO-O: %d\n", (int)bestPath.bb_crossed, (int)bestPath.yy_crossed, (int)bestPath.oo_crossed);
  printf("\nFinish line detected %d times.\n", (int)bestPath.finish_line_crossed);
#endif

  // Determine target point on bestPath.
  geometry_msgs::Point target = getTargetPoint(bestPath);

  // publish target point
  target_pub.publish(target);

#if RVIZ
  viz.visualize_bestPath_and_target(bestPath, target);
#endif

#if DEBUG_COORDS | DEBUG_PATH
  printf("\n--------------------------------------------------------------------------------\n\n");
#endif

#if TIMING
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time)
                   .count()
            << std::endl;
#endif
}
}  // namespace bumblebob_pathfinder