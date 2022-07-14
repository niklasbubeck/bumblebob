#include <bumblebob_msgs/Cone.h>

#include <ros/ros.h>

#include <vector>
#include <array>
#include <cmath>

namespace bumblebob_pathfinder
{
enum coneType
{
  undefined,
  yellow,
  blue,
  orange,
  ORANGE
};

struct Cone
{
  double x, y = 0;
  coneType type = coneType::undefined;
};

/**
 * @brief Returns distance between two 2D points.
 */
double getDistance(std::array<double, 2> pt1, std::array<double, 2> pt2);

/**
 * @brief Returns the middle between two 2D points.
 */
std::array<double, 2> getMiddlePoint(double x0, double y0, double x1, double y1);

/**
 * @brief Returns area of triangle spanned by point and two cones.
 */
double getTriangleArea(std::array<double, 2> pt, Cone cone0, Cone cone1);

/**
 * @brief Returns angle at point B.
 */
double getTriangleAngleBeta(std::array<double, 2> A, std::array<double, 2> B, std::array<double, 2> C);

/**
 * @brief Returns Cone at the given coordinates.
 */
Cone getCone(double x, double y, const std::vector<bumblebob_msgs::Cone>& coneArray);

/**
 * @brief Add 2D Point pt to array vector vec at index, if it is not already contained.
 */
bool addPointIfUnknownAtIndex(std::vector<double>& vec, std::array<double, 2> pt, int index);

/**
 * @brief Add 2D Point pt to double vector vec, if it is not already contained.
 */
bool addPointIfUnknown(std::vector<std::array<double, 2>>& vec, std::array<double, 2> pt);

/**
 * @brief Add 2D Point pt to array vector vec, if it is not already contained.
 */
bool addPointIfUnknown(std::vector<double>& vec, std::array<double, 2> pt);

}  // namespace bumblebob_pathfinder