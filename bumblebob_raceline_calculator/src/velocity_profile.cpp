#include "../include/velocity_profile.hpp"

VelocityProfile::VelocityProfile(std::vector<cv::Point2d> middle_line)
  : look_back_(3), look_ahead_(3), middle_line_(middle_line)
{
  readParams();
  estimateInitialVelocityProfile();
  calculateLengthProfile();
  estimateMaxCurvAcceleration();
  forwardStep();
  backwardStep();
}

void VelocityProfile::readParams()
{
  ros::param::get("/raceline_optimization/profiles/look_ahead", look_ahead_);
  ros::param::get("/raceline_optimization/profiles/look_back", look_back_);
  ros::param::get("/raceline_optimization/profiles/friction", friction_);
  ros::param::get("/raceline_optimization/profiles/gravity", gravity_);
  ros::param::get("/raceline_optimization/profiles/max_acceleration", max_accel_);
  ros::param::get("/raceline_optimization/profiles/max_deceleration", max_decel_);
  ros::param::get("/raceline_optimization/profiles/mass", mass_);
}

int VelocityProfile::pythonModulo(int a, int b)
{
  return (b + (a % b)) % b;
}

void VelocityProfile::calculateLengthProfile()
{
  length_profile_.push_back(0);
  for (int i = 1; i < middle_line_.size(); ++i)
  {
    double delta_s =
        sqrt(pow(middle_line_[i].x - middle_line_[i - 1].x, 2) + pow(middle_line_[i].y - middle_line_[i - 1].y, 2));
    length_profile_.push_back(delta_s + length_profile_[length_profile_.size() - 1]);
  }
}

double VelocityProfile::geometricCircleFit(std::vector<cv::Point2d> relevant_points)
{
  cv::Point2d point1 = relevant_points[0];
  cv::Point2d point2 = relevant_points[1];
  cv::Point2d point3 = relevant_points[2];

  // check if points need to be reordered to not get a vertical slope
  // (which would cause us to divide by 0)
  if (point2.x - point1.x == 0 || point3.x - point2.x == 0)
  {
    point1 = relevant_points[2];
    point2 = relevant_points[0];
    point3 = relevant_points[1];
  }

  // Get slopes of lines between points 1 and 2 and points 2 and 3
  float m1 = (point2.y - point1.y) / (point2.x - point1.x);
  float m2 = (point3.y - point2.y) / (point3.x - point2.x);

  if (m1 == m2)
  {
    return 0.01;
  }

  // Calculate the center of the circle made by points 1, 2, and 3
  float x = ((m1 * m2 * (point1.y - point3.y)) + (m2 * (point1.x + point2.x)) - (m1 * (point2.x + point3.x))) /
            (2 * (m2 - m1));
  float y = ((-1 / m1) * (x - ((point2.x + point1.x) / 2))) + ((point2.y + point1.y) / 2);
  cv::Point2d center = cv::Point2d(x, y);

  // Calculate the radius of the circle
  Eigen::Vector2f radius_vector(point1.x - center.x, point1.y - center.y);
  double radius = radius_vector.norm();
  return 1 / radius;
}

void VelocityProfile::estimateInitialVelocityProfile()
{
  for (int index = 0; index < middle_line_.size(); ++index)
  {
    std::vector<cv::Point2d> relevant_points;
    for (int i = index - look_back_; i <= index + look_ahead_; ++i)
    {
      int mod = pythonModulo(i, middle_line_.size());
      relevant_points.push_back(middle_line_[mod]);
    }
    double curvature = solveLeastSquaresCircleKasa(relevant_points);

    relevant_points.clear();
    double velocity = sqrt((friction_ * gravity_) / abs(curvature));
    velocity_profile_.push_back(velocity);
    velocity_initial_.push_back(velocity);

    curvature_profile_.push_back(curvature);
  }
}

double VelocityProfile::calculateCurvature(std::vector<cv::Point2d> relevant_points)
{
  int cols = 2;
  cv::Mat X(static_cast<int>(relevant_points.size()), cols, CV_64F);
  cv::Mat Y(static_cast<int>(relevant_points.size()), 1, CV_64F);
  cv::Mat C;

  if (int(relevant_points.size()) >= 3)
  {
    for (int i = 0; i < relevant_points.size(); ++i)
    {
      X.at<double>(i, 0) = relevant_points[i].x;
      X.at<double>(i, 1) = relevant_points[i].y;
      X.at<double>(i, 2) = 1;
      Y.at<double>(i, 0) = relevant_points[i].x * relevant_points[i].x + relevant_points[i].y * relevant_points[i].y;
    }
    cv::solve(X, Y, C, cv::DECOMP_SVD);
  }
  std::vector<double> coefs;
  C.col(0).copyTo(coefs);
  cv::Point2d center;
  center.x = coefs[0];
  center.y = coefs[1];
  center_points_.push_back(center);
  double radius = sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1]) / 4 + coefs[2];
  double curvature = 1 / radius;

  return curvature;
}

/**
 * Fit a circle in a set of points. You need a minimum of 1 point to fit a circle.
 *
 * @param points Is the set of points.
 * @param Returns true, if no error occur. An error occurs, if the points vector is empty.
 */
double VelocityProfile::solveLeastSquaresCircleKasa(std::vector<cv::Point2d> relevant_points)
{
  int length = relevant_points.size();
  double x1;
  double x2;
  double x3;
  double radius;
  Eigen::MatrixXd AFill(3, length);
  Eigen::MatrixXd A(length, 3);
  Eigen::VectorXd AFirst(length);
  Eigen::VectorXd ASec(length);
  Eigen::VectorXd AFirstSquared(length);
  Eigen::VectorXd ASecSquared(length);
  Eigen::VectorXd ASquaredRes(length);
  Eigen::VectorXd b(length);
  Eigen::VectorXd c(3);
  bool ok = true;

  if (length > 1)
  {
    for (int i = 0; i < length; i++)
    {
      AFill(0, i) = relevant_points[i].x;
      AFill(1, i) = relevant_points[i].y;
      AFill(2, i) = 1;
    }

    A = AFill.transpose();

    for (int i = 0; i < length; i++)
    {
      AFirst(i) = A(i, 0);
      ASec(i) = A(i, 1);
    }

    for (int i = 0; i < length; i++)
    {
      AFirstSquared(i) = AFirst(i) * AFirst(i);
      ASecSquared(i) = ASec(i) * ASec(i);
    }

    b = AFirstSquared + ASecSquared;

    c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    x1 = c(0);
    // midpoint(0) = x1 * 0.5;
    x2 = c(1);
    // midpoint(1) = x2 * 0.5;
    x3 = c(2);
    radius = sqrt((x1 * x1 + x2 * x2) / 4 + x3);
  }

  return 1 / radius;
}

void VelocityProfile::estimateMaxCurvAcceleration()
{
  for (int i = 0; i < velocity_profile_.size(); i++)
  {
    double gravity_2 = pow(gravity_, 2);
    double velocity_4 = pow(velocity_profile_[i], 2);
    double curv_2 = pow(curvature_profile_[i], 2);
    if (gravity_2 < velocity_4 * curv_2)
    {
      accel_restriction_.push_back(0);
    }
    else
    {
      accel_restriction_.push_back(sqrt(gravity_2 - velocity_4 * curv_2) * mass_);
    }
  }
}

void VelocityProfile::forwardStep()
{
  for (int i = 0; i < velocity_profile_.size(); ++i)
  {
    double delta_s = sqrt(pow(middle_line_[pythonModulo(i - 1, middle_line_.size())].x - middle_line_[i].x, 2) +
                          pow(middle_line_[pythonModulo(i - 1, middle_line_.size())].y - middle_line_[i].y, 2));
    double velocity_forward = sqrt(pow(velocity_profile_[(i - 1) % velocity_profile_.size()], 2) +
                                   2 * (std::min(max_accel_, accel_restriction_[i]) / mass_) * delta_s);

    velocity_profile_[i] = std::min(velocity_forward, velocity_profile_[i]);
  }
}

void VelocityProfile::backwardStep()
{
  for (int i = velocity_profile_.size(); i >= 0; --i)
  {
    double delta_s = sqrt(pow(middle_line_[pythonModulo(i + 1, middle_line_.size())].x - middle_line_[i].x, 2) +
                          pow(middle_line_[pythonModulo(i + 1, middle_line_.size())].y - middle_line_[i].y, 2));
    double velocity_backward = sqrt(pow(velocity_profile_[(i + 1) % velocity_profile_.size()], 2) +
                                    2 * (std::min(max_decel_, accel_restriction_[i]) / mass_) * delta_s);
    velocity_profile_[i] = std::min(velocity_backward, velocity_profile_[i]);
  }
}

std::vector<double> VelocityProfile::getVelocityProfile()
{
  return velocity_profile_;
}

std::vector<double> VelocityProfile::getCurvatureProfile()
{
  return curvature_profile_;
}

std::vector<double> VelocityProfile::getAccelRestriction()
{
  return accel_restriction_;
}

std::vector<double> VelocityProfile::getInitialVelocity()
{
  return velocity_initial_;
}