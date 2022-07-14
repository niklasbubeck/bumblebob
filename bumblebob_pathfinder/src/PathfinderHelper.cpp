#include "PathfinderHelper.hpp"

namespace bumblebob_pathfinder
{
/* Returns distance between two 2D points. */
double getDistance(std::array<double, 2> pt1, std::array<double, 2> pt2)
{
  return std::sqrt(std::pow(pt2[0] - pt1[0], 2) + std::pow(pt2[1] - pt1[1], 2));
}

/* Returns the middle between two 2D points. */
std::array<double, 2> getMiddlePoint(double x0, double y0, double x1, double y1)
{
  double xm = (x0 + x1) / 2;
  double ym = (y0 + y1) / 2;
  return { xm, ym };
}

/* Returns area of triangle spanned by point and two cones. */
double getTriangleArea(std::array<double, 2> pt, Cone cone0, Cone cone1)
{
  return std::abs(pt[0] * (cone0.y - cone1.y) + cone0.x * (cone1.y - pt[1]) + cone1.x * (pt[1] - cone0.y)) / 2;
}

/* Returns angle at point B. */
double getTriangleAngleBeta(std::array<double, 2> A, std::array<double, 2> B, std::array<double, 2> C)
{
  double a = getDistance(B, C);
  double b = getDistance(A, C);
  double c = getDistance(A, B);
  return std::acos((std::pow(c, 2) + std::pow(a, 2) - std::pow(b, 2)) / (2 * c * a)) * 180 / M_PI;
}

/* Returns Cone at the given coordinates. */
Cone getCone(double x, double y, const std::vector<bumblebob_msgs::Cone>& coneArray)
{
  Cone cone;
  for (bumblebob_msgs::Cone c : coneArray)
  {
    if (c.position.x == x && c.position.y == y)
    {
      cone.x = c.position.x;
      cone.y = c.position.y;
      cone.type = coneType(c.type);
      break;
    }
  }
  return cone;
}

/* Add 2D Point pt to array vector vec at index, if it is not already contained. */
bool addPointIfUnknownAtIndex(std::vector<double>& vec, std::array<double, 2> pt, int index)
{
  for (int i = vec.size() - 1; i >= 0; i -= 2)
  {
    if (vec[i - 1] == pt[0] && vec[i] == pt[1])
    {
      return false;
    }
  }
  vec[index] = pt[0];
  vec[index + 1] = pt[1];
  return true;
}

/* Add 2D Point pt to double vector vec, if it is not already contained. */
bool addPointIfUnknown(std::vector<std::array<double, 2>>& vec, std::array<double, 2> pt)
{
  // iterate over vec from the end, because the same point is more likely a recently added one
  if (!vec.empty())
  {
    for (int i = vec.size() - 1; i >= 0; --i)
    {
      if (vec[i][0] == pt[0] && vec[i][1] == pt[1])
      {
        return false;
      }
    }
  }
  vec.push_back({ pt[0], pt[1] });
  return true;
}

/* Add 2D Point pt to array vector vec, if it is not already contained. */
bool addPointIfUnknown(std::vector<double>& vec, std::array<double, 2> pt)
{
  // iterate over vec from the end, because the same point is more likely a recently added one
  for (int i = vec.size() - 1; i >= 0; i -= 2)
  {
    if (vec[i - 1] == pt[0] && vec[i] == pt[1])
    {
      return false;
    }
  }
  vec.push_back(pt[0]);
  vec.push_back(pt[1]);
  return true;
}

}  // namespace bumblebob_pathfinder