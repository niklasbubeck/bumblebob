#include "splines.hpp"
#include "raceline.hpp"

double RaceLine::calculateTrackWidth(cv::Point2d middlepoint)
{
  double shortest_distance_blue = 10000;
  double shortest_distance_yellow = 10000;

  CatmullRomSplines crs_blue(bluecones_, false);
  CatmullRomSplines crs_yellow(yellowcones_, false);

  std::vector<cv::Point2d> blueline = crs_blue.getSplineLine(ppm_);
  std::vector<cv::Point2d> yellowline = crs_yellow.getSplineLine(ppm_);

  for (int i = 0; i < blueline.size(); i++)
  {
    double distance = sqrt(pow(middlepoint.x - blueline[i].x, 2) + pow(middlepoint.y - blueline[i].y, 2));

    if (distance < shortest_distance_blue)
    {
      shortest_distance_blue = distance;
    }
  }

  for (int j = 0; j < yellowline.size(); j++)
  {
    double distance = sqrt(pow(middlepoint.x - yellowline[j].x, 2) + pow(middlepoint.y - yellowline[j].y, 2));

    if (distance < shortest_distance_yellow)
    {
      shortest_distance_yellow = distance;
    }
  }

  return shortest_distance_yellow + shortest_distance_blue;
}

std::vector<cv::Point2d> RaceLine::smoothRaceline(std::vector<cv::Point2d> raceline)
{
  std::vector<cv::Point2d> smooth_raceline{ raceline.size() };
  for (int i = 0; i < raceline.size(); i++)
  {
    double sum_x = 0;
    double sum_y = 0;
    int counter = 0;
    // define mean
    for (int j = i - strength_; j < i + strength_; j++)
    {
      if (j < 0)
      {
        sum_x += raceline[raceline.size() - abs(j)].x;
        sum_y += raceline[raceline.size() - abs(j)].y;
      }
      else
      {
        sum_x += raceline[j % raceline.size()].x;
        sum_y += raceline[j % raceline.size()].y;
      }

      counter++;
    }

    smooth_raceline[i] = cv::Point2d(sum_x / counter, sum_y / counter);
  }

  return smooth_raceline;
}

int RaceLine::getIndexOfNearestPoint(cv::Point2d middlepoint, Color clr)
{
  double shortest_distance_blue = 10000;
  double shortest_distance_yellow = 10000;

  int index = 0;

  CatmullRomSplines crs_blue(bluecones_, false);
  CatmullRomSplines crs_yellow(yellowcones_, false);

  if (clr == blue)
  {
    std::vector<cv::Point2d> blueline = crs_blue.getSplineLine(ppm_);
    for (int i = 0; i < blueline.size(); i++)
    {
      double distance = sqrt(pow(middlepoint.x - blueline[i].x, 2) + pow(middlepoint.y - blueline[i].y, 2));

      if (distance < shortest_distance_blue)
      {
        shortest_distance_blue = distance;
        index = i;
      }
    }
  }

  if (clr == yellow)
  {
    std::vector<cv::Point2d> yellowline = crs_yellow.getSplineLine(ppm_);
    for (int j = 0; j < yellowline.size(); j++)
    {
      double distance = sqrt(pow(middlepoint.x - yellowline[j].x, 2) + pow(middlepoint.y - yellowline[j].y, 2));

      if (distance < shortest_distance_yellow)
      {
        shortest_distance_yellow = distance;
        index = j;
      }
    }
  }
  return index;
}

double RaceLine::getDistFromPtToPt(cv::Point2d pt1, cv::Point2d pt2)
{
  return sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y));
}

double RaceLine::calculateCurvature(cv::Point2d p0, cv::Point2d p1, cv::Point2d p2)
{
  double area = fabs((p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y)) / 2);

  double dist01 = getDistFromPtToPt(p0, p1);
  double dist12 = getDistFromPtToPt(p1, p2);
  double dist02 = getDistFromPtToPt(p0, p2);
  double curvature = 4 * area / (dist01 * dist12 * dist02);

  return curvature;
}

std::vector<cv::Point2d> RaceLine::calculateRestriction(Color clr, bool isLooped)
{
  if (clr == blue)
  {
    std::vector<cv::Point2d> middle_points = getMiddlePoints();
    for (int i = 0; i < middle_points.size(); i++)
    {
      Color clr = blue;
      int index = getIndexOfNearestPoint(middle_points[i], clr);
      CatmullRomSplines crs_blue(bluecones_, false);
      std::vector<cv::Point2d> blueline = crs_blue.getSplineLine(ppm_);
      cv::Point2d nearest_point = blueline[index];
      cv::Point2d vector = { blueline[index].x - middle_points[i].x, blueline[index].y - middle_points[i].y };
      double veclen = sqrt(vector.x * vector.x + vector.y * vector.y);
      vector.x /= veclen;
      vector.y /= veclen;
      vector.x *= (veclen - distance_);
      vector.y *= (veclen - distance_);
      middle_points[i].x += vector.x;
      middle_points[i].y += vector.y;
    }

    CatmullRomSplines crs(middle_points, isLooped);
    std::vector<cv::Point2d> restrictionline = crs.getSplineLine(ppm_);
    return restrictionline;
  }
  else if (clr == yellow)
  {
    std::vector<cv::Point2d> middle_points = getMiddlePoints();
    for (int i = 0; i < middle_points.size(); i++)
    {
      Color clr = yellow;
      int index = getIndexOfNearestPoint(middle_points[i], yellow);
      CatmullRomSplines crs_yellow(yellowcones_, false);
      std::vector<cv::Point2d> yellowline = crs_yellow.getSplineLine(ppm_);
      cv::Point2d nearest_point = yellowline[index];
      cv::Point2d vector = { yellowline[index].x - middle_points[i].x, yellowline[index].y - middle_points[i].y };
      double veclen = sqrt(vector.x * vector.x + vector.y * vector.y);
      vector.x /= veclen;
      vector.y /= veclen;
      vector.x *= (veclen - distance_);
      vector.y *= (veclen - distance_);
      middle_points[i].x += vector.x;
      middle_points[i].y += vector.y;
    }

    CatmullRomSplines crs(middle_points, isLooped);
    std::vector<cv::Point2d> restrictionline = crs.getSplineLine(ppm_);
    return restrictionline;
  }
}

std::vector<cv::Point2d> RaceLine::getMiddlePoints()
{
  std::vector<cv::Point2d> middle_points;
  for (int i = 0; i < bluecones_.size(); i++)
  {
    cv::Point2d middlepoint = { (bluecones_[i].x + yellowcones_[i].x) / 2, (bluecones_[i].y + yellowcones_[i].y) / 2 };
    middle_points.push_back(middlepoint);
  }
  return middle_points;
}

RaceLine::RaceLine(std::vector<cv::Point2d> bluecones, std::vector<cv::Point2d> yellowcones, double distance,
                   int strength, int ppm)
{
  bluecones_ = bluecones;
  yellowcones_ = yellowcones;
  distance_ = distance;
  strength_ = strength;
  ppm_ = ppm;
}

std::vector<cv::Point2d> RaceLine::calculateMappedRacinglineShortest(int nIter)
{
  std::vector<cv::Point2d> middle_points = getMiddlePoints();

  std::vector<cv::Point2d> racing_line_points(middle_points.size());
  // double trackwidth = 0.68;
  std::vector<double> f_displace(middle_points.size());
  for (int n = 0; n < nIter; n++)
  {
    for (int i = 0; i < middle_points.size(); i++)
    {
      // Get locations of neighbour nodes
      cv::Point2d point_right = middle_points[(i + 1) % middle_points.size()];
      cv::Point2d point_left = middle_points[(i + middle_points.size() - 1) % middle_points.size()];
      cv::Point2d point_middle = middle_points[i];

      // Create vectors to neighbours
      cv::Point2d vector_left = { point_left.x - point_middle.x, point_left.y - point_middle.y };
      cv::Point2d vector_right = { point_right.x - point_middle.x, point_right.y - point_middle.y };

      // normalise neighbours
      double length_left = sqrtf(vector_left.x * vector_left.x + vector_left.y * vector_left.y);
      double length_right = sqrtf(vector_right.x * vector_right.x + vector_right.y * vector_right.y);
      cv::Point2d n_left = { vector_left.x / length_left, vector_left.y / length_left };
      cv::Point2d n_right = { vector_right.x / length_right, vector_right.y / length_right };

      // create bisector vector
      cv::Point2d vector_sum = { n_right.x + n_left.x, n_right.y + n_left.y };
      double len = sqrtf(vector_sum.x * vector_sum.x + vector_sum.y * vector_sum.y);
      vector_sum.x /= len;
      vector_sum.y /= len;

      // Get point gradient and normalise
      CatmullRomSplines crs(middle_points, true);
      cv::Point2d gradient = crs.getSplineGradient(i, true);
      double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
      gradient.x /= g_length;
      gradient.y /= g_length;

      // Project required correction onto point tangent to give displacment
      double dp = -gradient.y * vector_sum.x + gradient.x * vector_sum.y;

      // shortest
      f_displace[i] += (dp * 0.1);
    }

    // Clamp displaced points to track width
    for (int i = 0; i < middle_points.size(); i++)
    {
      double trackwidth = calculateTrackWidth(middle_points[i]);
      if (f_displace[i] >= trackwidth / 2 - distance_)
        f_displace[i] = trackwidth / 2 - distance_;
      if (f_displace[i] <= -trackwidth / 2 + distance_)
        f_displace[i] = -trackwidth / 2 + distance_;

      CatmullRomSplines crs(middle_points, true);
      cv::Point2d gradient = crs.getSplineGradient(i, true);
      double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
      gradient.x /= g_length;
      gradient.y /= g_length;

      racing_line_points[i].x = middle_points[i].x + -gradient.y * f_displace[i];

      racing_line_points[i].y = middle_points[i].y + gradient.x * f_displace[i];
    }
  }

  CatmullRomSplines crs(racing_line_points, true);
  std::vector<cv::Point2d> racing_line = crs.getSplineLine(ppm_);

  return racing_line;
}

std::vector<cv::Point2d> RaceLine::calculateMappedRacinglineLeastCurvature(int nIter)
{
  std::vector<cv::Point2d> middle_points = getMiddlePoints();

  std::vector<cv::Point2d> racing_line_points(middle_points.size());
  std::vector<double> f_displace(middle_points.size());
  for (int n = 0; n < nIter; n++)
  {
    for (int i = 0; i < middle_points.size(); i++)
    {
      // Get locations of neighbour nodes
      cv::Point2d point_right = middle_points[(i + 1) % middle_points.size()];
      cv::Point2d point_left = middle_points[(i + middle_points.size() - 1) % middle_points.size()];
      cv::Point2d point_middle = middle_points[i];

      // Create vectors to neighbours
      cv::Point2d vector_left = { point_left.x - point_middle.x, point_left.y - point_middle.y };
      cv::Point2d vector_right = { point_right.x - point_middle.x, point_right.y - point_middle.y };

      // normalise neighbours
      double length_left = sqrtf(vector_left.x * vector_left.x + vector_left.y * vector_left.y);
      double length_right = sqrtf(vector_right.x * vector_right.x + vector_right.y * vector_right.y);
      cv::Point2d n_left = { vector_left.x / length_left, vector_left.y / length_left };
      cv::Point2d n_right = { vector_right.x / length_right, vector_right.y / length_right };

      // create bisector vector
      cv::Point2d vector_sum = { n_right.x + n_left.x, n_right.y + n_left.y };
      double len = sqrtf(vector_sum.x * vector_sum.x + vector_sum.y * vector_sum.y);
      vector_sum.x /= len;
      vector_sum.y /= len;

      // Get point gradient and normalise
      CatmullRomSplines crs(middle_points, true);
      cv::Point2d gradient = crs.getSplineGradient(i, true);

      double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
      gradient.x /= g_length;
      gradient.y /= g_length;

      // Project required correction onto point tangent to give displacment
      double dp = -gradient.y * vector_sum.x + gradient.x * vector_sum.y;

      // Curvature
      f_displace[(i + 1) % middle_points.size()] += dp * -0.2;
      f_displace[(i - 1 + middle_points.size()) % middle_points.size()] += dp * -0.2;
    }

    // Clamp displaced points to track width
    for (int i = 0; i < middle_points.size(); i++)
    {
      double trackwidth = calculateTrackWidth(middle_points[i]);
      if (f_displace[i] >= trackwidth / 2 - distance_)
        f_displace[i] = trackwidth / 2 - distance_;
      if (f_displace[i] <= -trackwidth / 2 + distance_)
        f_displace[i] = -trackwidth / 2 + distance_;

      CatmullRomSplines crs(middle_points, true);
      cv::Point2d gradient = crs.getSplineGradient(i, true);
      double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
      gradient.x /= g_length;
      gradient.y /= g_length;

      racing_line_points[i].x = middle_points[i].x + -gradient.y * f_displace[i];

      racing_line_points[i].y = middle_points[i].y + gradient.x * f_displace[i];
    }
  }

  CatmullRomSplines crs(racing_line_points, true);
  std::vector<cv::Point2d> racing_line = crs.getSplineLine(ppm_);

  return racing_line;
}

std::vector<cv::Point2d> RaceLine::calculateVisualRacinglineCurvature()
{
  std::vector<cv::Point2d> middle_points = getMiddlePoints();
  std::vector<cv::Point2d> racing_line_points(middle_points.size());
  std::vector<double> f_displace(middle_points.size());
  for (int i = 0; i < middle_points.size(); i++)
  {
    // calculate the nearest point to the conelines
    Color blue = blue;
    Color yellow = yellow;
    int index_blue = getIndexOfNearestPoint(middle_points[i], blue);
    int index_yellow = getIndexOfNearestPoint(middle_points[i], yellow);

    double curvature_blue;
    double curvature_yellow;

    // calculate the curvatures of the conelines at the specific point
    if (i == 0)
    {
      curvature_blue =
          calculateCurvature(bluecones_[index_blue], bluecones_[index_blue + 1], bluecones_[index_blue + 2]);
      curvature_yellow = calculateCurvature(yellowcones_[index_yellow], yellowcones_[index_yellow + 1],
                                            yellowcones_[index_yellow + 2]);
    }
    else if (i == middle_points.size() - 1)
    {
      curvature_blue =
          calculateCurvature(bluecones_[index_blue - 2], bluecones_[index_blue - 1], bluecones_[index_blue]);
      curvature_yellow = calculateCurvature(yellowcones_[index_yellow - 2], yellowcones_[index_yellow - 1],
                                            yellowcones_[index_yellow]);
    }
    else
    {
      curvature_blue =
          calculateCurvature(bluecones_[index_blue - 1], bluecones_[index_blue], bluecones_[index_blue + 1]);
      curvature_yellow = calculateCurvature(yellowcones_[index_yellow - 1], yellowcones_[index_yellow],
                                            yellowcones_[index_yellow + 1]);
    }

    // take average
    double curvature = (curvature_blue + curvature_yellow) / 2;
    // 0.435 is the max curvature possible for byssa according Ackermann and is mapped linearly
    double length;
    // ros::param::get("/raceline_publisher/car/kinematics/l", length);
    double max_steering_wheel;
    // ros::param::get("/raceline_publisher/car/tire/max_steering", max_steering_wheel);
    double max_curvature = tan(max_steering_wheel) / length;
    double scale = 1 / max_curvature;
    f_displace[i] = curvature * scale;
  }
  // Clamp displaced points to track width
  for (int i = 0; i < middle_points.size(); i++)
  {
    double trackwidth = calculateTrackWidth(middle_points[i]);

    CatmullRomSplines crs(middle_points, false);
    cv::Point2d gradient = crs.getSplineGradient(i, false);
    double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
    gradient.x /= g_length;
    gradient.y /= g_length;

    racing_line_points[i].x = middle_points[i].x + -gradient.y * f_displace[i] * trackwidth / 2;

    racing_line_points[i].y = middle_points[i].y + gradient.x * f_displace[i] * trackwidth / 2;
  }
  CatmullRomSplines crs(racing_line_points, false);
  std::vector<cv::Point2d> racing_line = crs.getSplineLine(ppm_);

  return racing_line;
}

std::vector<cv::Point2d> RaceLine::calculateVisualRacingLineMiddle()
{
  std::vector<cv::Point2d> middle_points = getMiddlePoints();
  CatmullRomSplines crs2(middle_points, false);
  std::vector<cv::Point2d> racing_line = crs2.getSplineLine(ppm_);
  return racing_line;
}

std::vector<cv::Point2d> RaceLine::calculateMappedRacingLineMiddle()
{
  std::vector<cv::Point2d> middle_points = getMiddlePoints();
  CatmullRomSplines crs2(middle_points, true);
  std::vector<cv::Point2d> racing_line = crs2.getSplineLine(ppm_);
  std::vector<cv::Point2d> smoothed = smoothRaceline(racing_line);
  return smoothed;
}
