#include "../include/feedforward.hpp"

FeedForward::FeedForward(const cv::Point2d vehicle_position, const double speed, std::vector<cv::Point2d>& waypoints,
                         double heading)
  : radius_(0)
  , straight_line_(false)
  , circle_direction_(1)
  , look_back_(10)
  , look_front_(10)
  , vehicle_position_(vehicle_position)
  , speed_(speed)
  , waypoints_(waypoints)
  , heading_(heading)
{
}

void FeedForward::runIteration()
{
  defineRelevantPoints();
  leastSquareCircle();
  calculateFeedforwardAngle();
}

int FeedForward::findNearestPoint()
{
  // find nearest point to the path
  int index = 0;

  double smallest_dist = 100000;
  for (int i = 0; i < waypoints_.size(); ++i)
  {
    double distance =
        sqrt(pow(waypoints_[i].x - vehicle_position_.x, 2) + pow(waypoints_[i].y - vehicle_position_.y, 2));
    if (distance < smallest_dist)
    {
      smallest_dist = distance;
      index = i;
    }
  }
  return index;
}

void FeedForward::defineRelevantPoints()
{
  int index = findNearestPoint();

  if (waypoints_.size() < look_back_ + look_front_)
  {
    ROS_ERROR("The number of waypoints is not big enough to match the desired field of calculation: Please lower "
              "the values for look_front and look_back or include more waypoints");
  }

  nearest_point_ = { waypoints_[index].x, waypoints_[index].y };

  for (int j = index - look_back_; j < index + look_front_; j++)
  {
    // Systematically erase the possibility to divide by zero later on
    if (waypoints_[j % waypoints_.size()].x == 0 and waypoints_[j % waypoints_.size()].y == 0)
    {
      ROS_INFO("A Waypoint (0, 0) was found and ignored");
      continue;
    }

    if (j < 0)
    {
      relevant_points_.push_back(waypoints_[waypoints_.size() - abs(j)]);
    }
    else
    {
      relevant_points_.push_back(waypoints_[j % waypoints_.size()]);
    }
  }
}

void FeedForward::leastSquareCircle()
{
  int cols = 2;
  cv::Mat X(static_cast<int>(relevant_points_.size()), cols, CV_64F);
  cv::Mat Y(static_cast<int>(relevant_points_.size()), 1, CV_64F);
  cv::Mat C;

  if (int(relevant_points_.size()) >= 3)
  {
    for (size_t i = 0; i < relevant_points_.size(); ++i)
    {
      X.at<double>(static_cast<int>(i), 0) = 2 * relevant_points_[i].x;
      X.at<double>(static_cast<int>(i), 1) = 2 * relevant_points_[i].y;
      Y.at<double>(static_cast<int>(i), 0) =
          (relevant_points_[i].x * relevant_points_[i].x + relevant_points_[i].y * relevant_points_[i].y);
    }
    cv::solve(X, Y, C, cv::DECOMP_SVD);
  }
  std::vector<double> coefs;
  C.col(0).copyTo(coefs);
  center_.x = coefs[0];
  center_.y = coefs[1];
  radius_ = sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1]);

  double relative_x = center_.x - vehicle_position_.x;
  double relative_y = center_.y - vehicle_position_.y;
  double rotated_x = std::cos(heading_) * relative_x - std::sin(heading_) * relative_y;
  double rotated_y = std::cos(heading_) * relative_y + std::sin(heading_) * relative_x;

  if (rotated_x < 0)
  {
    circle_direction_ = -1;
  }
  else if (rotated_x > 0)
  {
    circle_direction_ = 1;
  }
  else
  {
    straight_line_ = true;
  }
}

void FeedForward::calculateFeedforwardAngle()
{
  if (straight_line_)
  {
    feedforward_angle_ = 0;
  }
  else
  {
    double length_between_axles = 1.53;
    double angle = atan(length_between_axles / radius_);
    feedforward_angle_ = circle_direction_ * angle;
  }
}
