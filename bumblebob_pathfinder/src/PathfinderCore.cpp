#include "PathfinderCore.hpp"

namespace bumblebob_pathfinder
{
Path::Path() : weight(0)
{
}

Path::Path(double weight, std::vector<std::array<double, 2>> nodes) : weight(weight), nodes(nodes)
{
}

PathfinderCore::PathfinderCore(ros::NodeHandle& nh) : nh(nh)
{
  ros::param::get("/pathfinder/params/car_position_x", carPos[0]);
  ros::param::get("/pathfinder/params/car_position_y", carPos[1]);
  ros::param::get("/pathfinder/params/same_type_limit", same_type_limit);
  ros::param::get("/pathfinder/params/section_length", section_length);
  ros::param::get("/pathfinder/params/section_stride", section_stride);
  ros::param::get("/pathfinder/params/seg_seg_min_angle", seg_seg_min_angle);
  ros::param::get("/pathfinder/params/segment_max_len", segment_max_len);
  ros::param::get("/pathfinder/params/finish_line_trigger_radius", finish_line_trigger_radius);
  ros::param::get("/pathfinder/params/finish_line_trigger_corridor", finish_line_trigger_corridor);
  ros::param::get("/pathfinder/topics/pub/finish_line_signal", finish_line_topic);

  // correct values to safe default if out of reasonable bounds
  if (section_length < 1)
    section_length = 1;
  if (section_stride > section_length || section_stride < 1)
    section_stride = section_length;
  if (segment_max_len < 1)
    segment_max_len = 1;
  if (seg_seg_min_angle < 0 || seg_seg_min_angle > 180)
    seg_seg_min_angle = 90;
  if (finish_line_trigger_radius < 0)
    finish_line_trigger_radius = 0;
  if (finish_line_trigger_corridor < 0)
    finish_line_trigger_corridor = 0;

  seg_seg_angle_normalizer = 180 - seg_seg_min_angle;

  finish_line_pub = nh.advertise<std_msgs::String>(finish_line_topic, 1);
  finish_line_msg.data = "STOP";
}

/* Setting parameters from dynamic reconfigure. */
void PathfinderCore::reconfigure(bumblebob_pathfinder::PathfinderConfig& config, uint32_t& level)
{
  if (config.section_stride > config.section_length)
    config.section_stride = config.section_length;

  carPos[0] = config.car_position_x;
  carPos[1] = config.car_position_y;
  same_type_limit = config.same_type_limit;
  section_length = config.section_length;
  section_stride = config.section_stride;
  seg_seg_min_angle = config.seg_seg_min_angle;
  segment_max_len = config.segment_max_len;
  finish_line_trigger_radius = config.finish_line_trigger_radius;
  finish_line_trigger_corridor = config.finish_line_trigger_corridor;
}

/* Calculate middle points of triangle edges (possible path points) AND according edgepoint types (cone colors). */
void PathfinderCore::calcPathPointCoords(const delaunator::Delaunator& d,
                                         const std::vector<bumblebob_msgs::Cone>& coneArray)
{
  double tx0, ty0, tx1, ty1, tx2, ty2;
  Cone cone0, cone1, cone2;

  unsigned pathPointCount = 0;
  std::size_t triangle_count = d.triangles.size();

  pathPointCoords.clear();
  pathPointCones.clear();
  pathPointCoords.resize(triangle_count * 6);
  pathPointCones.resize(triangle_count * 6);

  for (std::size_t i = 0; i < triangle_count; i += 3)
  {
    tx0 = d.coords[2 * d.triangles[i]];
    ty0 = d.coords[2 * d.triangles[i] + 1];
    tx1 = d.coords[2 * d.triangles[i + 1]];
    ty1 = d.coords[2 * d.triangles[i + 1] + 1];
    tx2 = d.coords[2 * d.triangles[i + 2]];
    ty2 = d.coords[2 * d.triangles[i + 2] + 1];

    cone0 = getCone(tx0, ty0, coneArray);
    cone1 = getCone(tx1, ty1, coneArray);
    cone2 = getCone(tx2, ty2, coneArray);

    if (addPointIfUnknownAtIndex(pathPointCoords, getMiddlePoint(tx0, ty0, tx1, ty1), pathPointCount * 2))
    {
      pathPointCones[pathPointCount * 2] = cone0;
      pathPointCones[pathPointCount * 2 + 1] = cone1;
      ++pathPointCount;
    }
    if (addPointIfUnknownAtIndex(pathPointCoords, getMiddlePoint(tx0, ty0, tx2, ty2), pathPointCount * 2))
    {
      pathPointCones[pathPointCount * 2] = cone0;
      pathPointCones[pathPointCount * 2 + 1] = cone2;
      ++pathPointCount;
    }
    if (addPointIfUnknownAtIndex(pathPointCoords, getMiddlePoint(tx1, ty1, tx2, ty2), pathPointCount * 2))
    {
      pathPointCones[pathPointCount * 2] = cone1;
      pathPointCones[pathPointCount * 2 + 1] = cone2;
      ++pathPointCount;
    }
  }
  // resize to valid elements
  pathPointCoords.resize(pathPointCount * 2);
  pathPointCones.resize(pathPointCount * 2);
}

/* Return delaunated path mesh. */
delaunator::Delaunator PathfinderCore::getMesh(const delaunator::Delaunator& d,
                                               const std::vector<bumblebob_msgs::Cone>& coneArray)
{
  // Calculate all possible path points and store types of adjacent cones.
  calcPathPointCoords(d, coneArray);

  // Add car position to path points.
  pathPointCoords.push_back(carPos[0]);
  pathPointCoords.push_back(carPos[1]);

  // Generate mesh of possible path points that contains all possible paths.
  delaunator::Delaunator pathMesh(pathPointCoords);

  return pathMesh;
}

/* Print path mesh points for debugging. */
void PathfinderCore::printPathMeshPoints()
{
  for (int i = 0; i < pathPointCoords.size(); i += 2)
  {
    printf("Path point %i: [%f, %f]\n", i / 2 + 1, pathPointCoords[i], pathPointCoords[i + 1]);
  }
}

/* Returns all possible next points that have not yet been visited. */
std::vector<std::array<double, 2>> PathfinderCore::getNextPoints(
    const std::array<double, 2>& curPt, const std::vector<std::array<double, 6>>& pathMeshTriangles,
    std::vector<std::array<double, 2>>& ignoreList)
{
  std::vector<std::array<double, 2>> nextPoints;

  for (std::array<double, 6> tri : pathMeshTriangles)
  {
    if (tri[0] == curPt[0] && tri[1] == curPt[1])
    {
      if (addPointIfUnknown(ignoreList, { tri[2], tri[3] }))
      {
        nextPoints.push_back({ tri[2], tri[3] });
      }
      if (addPointIfUnknown(ignoreList, { tri[4], tri[5] }))
      {
        nextPoints.push_back({ tri[4], tri[5] });
      }
    }
    else if (tri[2] == curPt[0] && tri[3] == curPt[1])
    {
      if (addPointIfUnknown(ignoreList, { tri[0], tri[1] }))
      {
        nextPoints.push_back({ tri[0], tri[1] });
      }
      if (addPointIfUnknown(ignoreList, { tri[4], tri[5] }))
      {
        nextPoints.push_back({ tri[4], tri[5] });
      }
    }
    else if (tri[4] == curPt[0] && tri[5] == curPt[1])
    {
      if (addPointIfUnknown(ignoreList, { tri[0], tri[1] }))
      {
        nextPoints.push_back({ tri[0], tri[1] });
      }
      if (addPointIfUnknown(ignoreList, { tri[2], tri[3] }))
      {
        nextPoints.push_back({ tri[2], tri[3] });
      }
    }
  }
  return nextPoints;
}

/* Recursively determine next path section. */
Path PathfinderCore::getNextSection(unsigned remainingLength, const std::array<double, 2>& prevPt,
                                    const std::array<double, 2>& curPt,
                                    const std::vector<std::array<double, 2>>& nextPts,
                                    const std::vector<std::array<double, 6>>& pathMeshTriangles,
                                    std::vector<std::array<double, 2>> ignoreList, weightFuncPtr wf)
{
  Path bestPathSeg;
  double minWeight = std::numeric_limits<double>::max();

  if (remainingLength <= 0 || nextPts.empty())  // base case
  {
    bestPathSeg = Path(0, { curPt });
  }
  else  // recursion
  {
    for (std::array<double, 2> nextPt : nextPts)
    {
      std::vector<std::array<double, 2>> ignoreListLocal = ignoreList;
      std::vector<std::array<double, 2>> nextNextPts = getNextPoints(nextPt, pathMeshTriangles, ignoreListLocal);
      Path p = getNextSection(remainingLength - 1, curPt, nextPt, nextNextPts, pathMeshTriangles, ignoreListLocal, wf);
      std::array<double, 5> cpw = ((*this).*wf)(prevPt, curPt, nextPt);

      double newWeight = p.weight + cpw[0];

      if (newWeight < minWeight)
      {
        p.finish_line_crossed += cpw[1];
        p.blues += cpw[2];
        p.yellows += cpw[3];
        p.oranges += cpw[4];
        if (cpw[2] == 2)
          ++p.bb_crossed;
        else if (cpw[3] == 2)
          ++p.yy_crossed;
        else if (cpw[4] == 2)
          ++p.oo_crossed;
        if (p.bb_crossed + p.yy_crossed <= same_type_limit)
        {
          minWeight = newWeight;
          bestPathSeg = p;
        }
      }
    }
    bestPathSeg.nodes.push_back(curPt);
    bestPathSeg.weight += minWeight;
  }
  return bestPathSeg;
}

/* BFS of the best path. */
Path PathfinderCore::findPath(const delaunator::Delaunator& pathMesh, Bias bias)
{
  weightFuncPtr wf;

  switch (bias)
  {
    case Bias::none:
      wf = &PathfinderCore::calcPathWeight;
      break;
    case Bias::straight:
      wf = &PathfinderCore::calcPathWeight_prefStraight;
      break;
    case Bias::right:
      wf = &PathfinderCore::calcPathWeight_prefRight;
      break;
    case Bias::left:
      wf = &PathfinderCore::calcPathWeight_prefLeft;
      break;
  }

  // initialize path with car position
  Path bestPath;
  bestPath.nodes.push_back(carPos);

  // generate triangle vector from pathMesh
  std::vector<std::array<double, 6>> pathMeshTriangles(pathMesh.triangles.size() / 3);

  for (std::size_t i = 0; i < pathMesh.triangles.size(); i += 3)
  {
    pathMeshTriangles[i / 3][0] = pathMesh.coords[2 * pathMesh.triangles[i]];          // tx0
    pathMeshTriangles[i / 3][1] = pathMesh.coords[2 * pathMesh.triangles[i] + 1];      // ty0
    pathMeshTriangles[i / 3][2] = pathMesh.coords[2 * pathMesh.triangles[i + 1]];      // tx1
    pathMeshTriangles[i / 3][3] = pathMesh.coords[2 * pathMesh.triangles[i + 1] + 1];  // ty1
    pathMeshTriangles[i / 3][4] = pathMesh.coords[2 * pathMesh.triangles[i + 2]];      // tx2
    pathMeshTriangles[i / 3][5] = pathMesh.coords[2 * pathMesh.triangles[i + 2] + 1];  // ty2
  }

  // initialize parameters for first loop
  std::array<double, 2> curPt = carPos;
  std::array<double, 2> prevPt = { carPos[0] - 1.0, carPos[1] };
  std::vector<std::array<double, 2>> ignoreList, trash = { carPos };
  std::vector<std::array<double, 2>> nextPts = getNextPoints(curPt, pathMeshTriangles, trash);

  // loop until no more points or cut-off
  while (!nextPts.empty())
  {
    // determine next path section from curPt on
    Path pathSection = getNextSection(section_length, prevPt, curPt, nextPts, pathMeshTriangles, ignoreList, wf);

    // cut-off, only if bestPath has at least two nodes
    if (pathSection.weight >= CUTOFFWEIGHT && bestPath.nodes.size() > 1)
      break;

    // add path section points to bestPath and ignore list, up to section_stride (or end) deep
    for (auto pt = pathSection.nodes.rbegin() + 1;
         pt != pathSection.nodes.rend() && pt != pathSection.nodes.rbegin() + 1 + section_stride; ++pt)
    {
      addPointIfUnknown(ignoreList, *pt);
      bestPath.nodes.push_back(*pt);
    }

    // add additional parameters to bestPath TODO respect stride in parameter calculation, right now its only correct
    // for stride==seglen
    bestPath.weight += pathSection.weight;
    bestPath.finish_line_crossed += pathSection.finish_line_crossed;
    bestPath.blues += pathSection.blues;
    bestPath.yellows += pathSection.yellows;
    bestPath.oranges += pathSection.oranges;
    bestPath.bb_crossed += pathSection.bb_crossed;
    bestPath.yy_crossed += pathSection.yy_crossed;
    bestPath.oo_crossed += pathSection.oo_crossed;

    // update parameters for next loop -> new curPt is furthest point in bestPath
    curPt = *bestPath.nodes.rbegin();
    prevPt = *(bestPath.nodes.rbegin() + 1);
    nextPts = getNextPoints(curPt, pathMeshTriangles, ignoreList);
  }
  return bestPath;
}

/* Return weight for potential path point based of types of both adjacent cones.
   Also handles finish line detection. */
std::array<double, 5> PathfinderCore::coneTypeWeight(const std::array<double, 2>& point,
                                                     const std::array<double, 2>& prevPt)
{
  Cone cone0, cone1;
  double weight = 1;
  double finish_line_detected = 0;
  double blues = 0;
  double yellows = 0;
  double oranges = 0;

  // find cone types
  for (unsigned i = 0; i < pathPointCoords.size(); i += 2)
  {
    if (pathPointCoords[i] == point[0] && pathPointCoords[i + 1] == point[1])
    {
      cone0 = pathPointCones[i];
      cone1 = pathPointCones[i + 1];
      break;
    }
  }

  // count cone types
  switch (cone0.type)
  {
    case coneType::blue:
      ++blues;
      break;
    case coneType::yellow:
      ++yellows;
      break;
    case coneType::ORANGE:
      ++oranges;
      break;
  }
  switch (cone1.type)
  {
    case coneType::blue:
      ++blues;
      break;
    case coneType::yellow:
      ++yellows;
      break;
    case coneType::ORANGE:
      ++oranges;
      break;
  }

  // Check for finish line
  if (cone0.type == coneType::ORANGE && cone1.type == coneType::ORANGE)
  {
    if (point[1] <= finish_line_trigger_corridor && point[1] >= -finish_line_trigger_corridor &&
        getDistance(carPos, point) <= finish_line_trigger_radius)
    {
      finish_line_pub.publish(finish_line_msg);
      finish_line_detected = 1;
    }
  }

  // Point probably between track borders, the place to be!
  if ((cone0.type == coneType::blue && cone1.type == coneType::yellow) ||
      (cone0.type == coneType::yellow && cone1.type == coneType::blue))
    weight = 0;

  // cone0-prevPt-cone1 angle, avoid small angles (driving on track border)
  double beta = getTriangleAngleBeta({ cone0.x, cone0.y }, prevPt, { cone1.x, cone1.y });
  if (beta < 40)
    weight = CUTOFFWEIGHT;

  return { weight, finish_line_detected, blues, yellows, oranges };
}

// TODO if cone type check not robust enough: Check distance between path and cones.
/* Calculates weight of path from curPt to nextPt. */
std::array<double, 5> PathfinderCore::calcPathWeight(const std::array<double, 2>& prevPt,
                                                     const std::array<double, 2>& curPt,
                                                     const std::array<double, 2>& nextPt)
{
  double weight = 0;

  // weight based on adjacent cone types
  std::array<double, 5> ctw = coneTypeWeight(nextPt, curPt);
  weight += ctw[0];

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on next section length, scale: 0-1 corresponding to 0-segment_max_len meters
    double dist = getDistance(curPt, nextPt);
    if (dist <= segment_max_len)
      weight += dist / segment_max_len;
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on angle between last and next section, scale: 0-1 corresponding to 180-seg_seg_min_angle degrees
    double beta = getTriangleAngleBeta(prevPt, curPt, nextPt);
    if (beta >= seg_seg_min_angle)
      weight += (180 - beta) / seg_seg_angle_normalizer;
    else
      weight = CUTOFFWEIGHT;
  }

  return { weight, ctw[1], ctw[2], ctw[3], ctw[4] };
}

// Calculates weight of path from curPt to nextPt. Prefer straight paths.
std::array<double, 5> PathfinderCore::calcPathWeight_prefStraight(const std::array<double, 2>& prevPt,
                                                                  const std::array<double, 2>& curPt,
                                                                  const std::array<double, 2>& nextPt)
{
  double weight = 0;

  // weight based on adjacent cone types
  std::array<double, 5> ctw = coneTypeWeight(nextPt, curPt);
  weight += ctw[0];

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on next section length, scale: 0-1 corresponding to 0-segment_max_len meters
    double dist = getDistance(curPt, nextPt);
    if (dist <= segment_max_len)
      weight += dist / segment_max_len;
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on angle between last and next section, scale: 0-75 corresponding to 180-seg_seg_min_angle degrees
    double beta = getTriangleAngleBeta(prevPt, curPt, nextPt);
    if (beta >= seg_seg_min_angle)
      weight += (180 - beta);
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // prefer points straight ahead of the car
    weight += pow(nextPt[1], 2);
  }

  return { weight, ctw[1], ctw[2], ctw[3], ctw[4] };
}

// Calculates weight of path from curPt to nextPt. Prefer paths heading right.
std::array<double, 5> PathfinderCore::calcPathWeight_prefRight(const std::array<double, 2>& prevPt,
                                                               const std::array<double, 2>& curPt,
                                                               const std::array<double, 2>& nextPt)
{
  double weight = 0;

  // weight based on adjacent cone types
  std::array<double, 5> ctw = coneTypeWeight(nextPt, curPt);
  weight += ctw[0];

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on next section length, scale: 0-1 corresponding to 0-segment_max_len meters
    double dist = getDistance(curPt, nextPt);
    if (dist <= segment_max_len)
      weight += dist / segment_max_len;
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on angle between last and next section, scale: 0-1 corresponding to 180-seg_seg_min_angle degrees
    double beta = getTriangleAngleBeta(prevPt, curPt, nextPt);
    if (beta >= seg_seg_min_angle)
      weight += (180 - beta) / seg_seg_angle_normalizer;
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // prefer points on the right side of the car
    if (nextPt[1] > 1)
      weight += pow(nextPt[1], 2);
    else if (nextPt[1] >= -1)
      weight += (nextPt[1] + 1) / 2;
  }

  return { weight, ctw[1], ctw[2], ctw[3], ctw[4] };
}

// Calculates weight of path from curPt to nextPt. Prefer paths heading left.
std::array<double, 5> PathfinderCore::calcPathWeight_prefLeft(const std::array<double, 2>& prevPt,
                                                              const std::array<double, 2>& curPt,
                                                              const std::array<double, 2>& nextPt)
{
  double weight = 0;

  // weight based on adjacent cone types
  std::array<double, 5> ctw = coneTypeWeight(nextPt, curPt);
  weight += ctw[0];

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on next section length, scale: 0-1 corresponding to 0-segment_max_len meters
    double dist = getDistance(curPt, nextPt);
    if (dist <= segment_max_len)
      weight += dist / segment_max_len;
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // weight based on angle between last and next section, scale: 0-1 corresponding to 180-seg_seg_min_angle degrees
    double beta = getTriangleAngleBeta(prevPt, curPt, nextPt);
    if (beta >= seg_seg_min_angle)
      weight += (180 - beta) / seg_seg_angle_normalizer;
    else
      weight = CUTOFFWEIGHT;
  }

  if (weight < CUTOFFWEIGHT)
  {
    // prefer points on the right side of the car
    if (nextPt[1] < -1)
      weight += pow(nextPt[1], 2);
    else if (nextPt[1] <= 1)
      weight += -(nextPt[1] - 1) / 2;
  }

  return { weight, ctw[1], ctw[2], ctw[3], ctw[4] };
}

}  // namespace bumblebob_pathfinder