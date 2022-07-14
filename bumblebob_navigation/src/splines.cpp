#include "splines.hpp"

CatmullRomSplines::CatmullRomSplines(std::vector<cv::Point2d>& points, bool isLooped)
{
  points_ = points;
  isLooped_ = isLooped;
}

CatmullRomSplines::CatmullRomSplines(const CatmullRomSplines& crs)
{
  points_ = crs.points_;
  isLooped_ = crs.isLooped_;
}

std::vector<cv::Point2d> CatmullRomSplines::getInitPoints()
{
  return points_;
}

bool CatmullRomSplines::isLooped()
{
  return isLooped_;
}

cv::Point2d CatmullRomSplines::getSplinePoint(double t, bool isLooped = false)
{
  int p0, p1, p2, p3;
  if (!isLooped)
  {
    p1 = static_cast<int>(t) + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;
  }
  else
  {
    p1 = static_cast<int>(t);
    p2 = (p1 + 1) % points_.size();
    p3 = (p2 + 1) % points_.size();
    if (p1 >= 1)
    {
      p0 = p1 - 1;
    }
    else
    {
      p0 = points_.size() - 1;
    }
  }

  t = t - static_cast<int>(t);

  // define shortcuts for cubics
  double tt = t * t;
  double ttt = tt * t;
  // define weight functions
  double q1 = -ttt + 2.0 * tt - t;
  double q2 = 3.0 * ttt - 5.0 * tt + 2.0;
  double q3 = -3.0 * ttt + 4.0 * tt + t;
  double q4 = ttt - tt;
  // calculate the point dependent on all weight functions
  double tx = 0.5 * (points_[p0].x * q1 + points_[p1].x * q2 + points_[p2].x * q3 + points_[p3].x * q4);
  double ty = 0.5 * (points_[p0].y * q1 + points_[p1].y * q2 + points_[p2].y * q3 + points_[p3].y * q4);
  return { tx, ty };
}

cv::Point2d CatmullRomSplines::getSplineGradient(double t, bool isLooped = false)
{
  int p0, p1, p2, p3;

  if (!isLooped)
  {
    p1 = static_cast<int>(t) + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;
  }
  else
  {
    p1 = static_cast<int>(t);
    p2 = (p1 + 1) % points_.size();
    p3 = (p2 + 1) % points_.size();
    if (p1 >= 1)
    {
      p0 = p1 - 1;
    }
    else
    {
      p0 = points_.size() - 1;
    }
  }
  double indicator = t;
  t = t - static_cast<int>(t);
  // define shortcuts
  double tt = t * t;
  double ttt = tt * t;
  // define the derivatives of the weight functions
  double q1 = -3.0 * tt + 4.0 * t - 1;
  double q2 = 9.0 * tt - 10.0 * t;
  double q3 = -9.0 * tt + 8.0 * t + 1.0;
  double q4 = 3.0 * tt - 2.0 * t;
  // define the point dependent on the gradient weight functions
  double tx = 0.5 * (points_[p0].x * q1 + points_[p1].x * q2 + points_[p2].x * q3 + points_[p3].x * q4);
  double ty = 0.5 * (points_[p0].y * q1 + points_[p1].y * q2 + points_[p2].y * q3 + points_[p3].y * q4);

  if (indicator == static_cast<double>(points_.size() - 1))
  {
    return { -tx, ty };
  }

  return { tx, ty };
}

std::vector<cv::Point2d> CatmullRomSplines::getSplineLine(double ppm)
{
  std::vector<cv::Point2d> spline;
  int counter = 0;
  double length = 0;
  // calculate length of spline
  for (int i = 1; i < points_.size(); i++)
  {
    length += sqrt(pow(points_[i].x - points_[i - 1].x, 2) + pow(points_[i].y - points_[i - 1].y, 2));
  }

  double increaser = points_.size() / (ppm * length);
  if (!isLooped_)
  {
    cv::Point2d startvector = { points_[1].x - points_[0].x, points_[1].y - points_[0].y };
    cv::Point2d new_first = points_[0] - startvector;
    points_.insert(points_.begin(), new_first);

    cv::Point2d endvector = { points_[points_.size() - 2].x - points_[points_.size() - 1].x,
                              points_[points_.size() - 2].y - points_[points_.size() - 1].y };

    cv::Point2d new_last = points_[points_.size() - 1] - endvector;
    points_.push_back(new_last);
  }

  for (double t = 0; t < static_cast<double>(points_.size()); t += increaser)
  {
    if (t > static_cast<double>(points_.size() - 3) && !isLooped_)
    {
      break;
    }
    cv::Point2d pos = getSplinePoint(t, isLooped_);

    spline.push_back(pos);
    counter++;
  }
  return spline;
}
