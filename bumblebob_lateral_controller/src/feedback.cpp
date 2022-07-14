#include "../include/feedback.hpp"

Feedback::Feedback(FeedForward& ff, cv::Point2d vehicle_position, double heading, double angular_velocity,
                   std::vector<cv::Point2d>& waypoints, double k_lat, double k_head, double k_o)
  : ff_(ff)
  , vehicle_position_(vehicle_position)
  , heading_(heading)
  , angular_velocity_(angular_velocity)
  , k_lat_(k_lat)
  , k_head_(k_head)
  , k_o_(k_o)
{
  double lat_corr = calculateLateralErrorDirect();
  double head_corr = calculateHeadingError();
  double omega_corr = calculateDeltaDotError();

  if (head_corr + lat_corr < 0 && omega_corr < 0)
  {
    omega_corr *= -1;
  }
  else if (head_corr + lat_corr > 0 && omega_corr > 0)
  {
    omega_corr *= -1;
  }

  feedback_angle_ = lat_corr + head_corr + omega_corr;
}

double Feedback::calculateLateralErrorDirect()
{
  // If its a straight line we dont want to have steering action
  if (ff_.straight_line_)
    return 0;

  // Find the shortest distance to the path
  // Attention!! This is not exactly the lateral error!!
  lateral_error_ = 100000;
  cv::Point2d nearest_point;
  for (auto relevant_point : ff_.relevant_points_)
  {
    double lateral_dist =
        sqrt(pow(vehicle_position_.x - relevant_point.x, 2) + pow(vehicle_position_.y - relevant_point.y, 2));
    if (lateral_dist < lateral_error_)
    {
      lateral_error_ = lateral_dist;
      nearest_point = relevant_point;
    }
  }

  // Calculate the shortest Indexpoint of the path relative to the vehicle coordinate system
  double relative_x = nearest_point.x - vehicle_position_.x;
  double relative_y = nearest_point.y - vehicle_position_.y;
  double rotated_x = std::cos(heading_) * relative_x - std::sin(heading_) * relative_y;
  double rotated_y = std::cos(heading_) * relative_y + std::sin(heading_) * relative_x;

  if (rotated_x < 0)
  {
    // we are on the right hand side of the path
    lateral_error_ = lateral_error_ * -1;
  }
  else if (rotated_x = 0)
  {
    return 0;
  }

  // Apply gain
  double correction = k_lat_ * lateral_error_;
  return correction;
}

double Feedback::calculateHeadingError()
{
  // Calculate vector from circle center to car position
  cv::Point2d circle_vector(vehicle_position_.x - ff_.center_.x, vehicle_position_.y - ff_.center_.y);

  // Get tangent vector to the circle vector (depends on the circle's direction)
  cv::Point2d road_vector;
  if (ff_.circle_direction_ == 1)
  {
    // right
    road_vector = cv::Point2d(-circle_vector.x, circle_vector.y);
  }
  else
  {
    // left
    road_vector = cv::Point2d(circle_vector.x, -circle_vector.y);
  }
  // Get the road's heading in world space
  road_heading_ = std::atan2(road_vector.y, road_vector.x);
  double vehicle_heading = fmod(heading_, M_PI * 2);

  // Calculate the angle difference between the car's and the road's headings
  heading_error_ = road_heading_ - vehicle_heading;

  if (heading_error_ > M_PI)
  {
    heading_error_ -= 2 * M_PI;
  }
  else if (heading_error_ < -M_PI)
  {
    heading_error_ += 2 * M_PI;
  }

  // Handling for more robustness, since when the heading diff is over 180 we already lost the game
  if (abs(heading_error_) > M_PI_2)
  {
    ROS_INFO("Mistakes were made... heading error too big \n");
    return 0;
  }

  // Apply gain to the heading error
  double correction = k_head_ * heading_error_;
  return correction;
}

double Feedback::calculateDeltaDotError()
{
  // Apply gain to the angular velocity
  double correction = k_o_ * angular_velocity_;
  return correction;
}
