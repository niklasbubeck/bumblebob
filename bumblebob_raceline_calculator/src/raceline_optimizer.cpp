#include "../include/raceline_optimizer.hpp"

RacelineOptimizer::RacelineOptimizer(VelocityProfile vel_prof, std::vector<cv::Point2d> bluecones,
                                     std::vector<cv::Point2d> yellcones, std::vector<cv::Point2d> waypoints)
  : waypoints_(waypoints), bluecones_(bluecones), yellcones_(yellcones), vel_current_(vel_prof)
{
  CatmullRomSplines init_(waypoints, true);
  raceline_current_ = init_.getSplineLine(20);
  readParams();
  optimizeRaceline();
  useSpline();
  smoothRaceline();
}

void RacelineOptimizer::readParams()
{
  ros::param::get("/raceline_optimization/optimizer/vehicle_width", vehicle_width_);
  ros::param::get("/raceline_optimization/optimizer/ppm", ppm_);
  ros::param::get("/raceline_optimization/optimizer/dp_factor", dp_factor_);
  ros::param::get("/raceline_optimization/optimizer/smoothing_strength", strength_);
  ros::param::get("/raceline_optimization/optimizer/initial_time_current", time_current_);
  ros::param::get("/raceline_optimization/optimizer/initial_time_before", time_before_);
  ros::param::get("/raceline_optimization/optimizer/initial_accel_current", accel_current_);
  ros::param::get("/raceline_optimization/optimizer/initial_accel_before", accel_before_);
  ros::param::get("/raceline_optimization/optimizer/loop", loop_);
  ros::param::get("/raceline_optimization/optimizer/counter", counter_);
  ros::param::get("/raceline_optimization/optimizer/increaser", increaser_);
  ros::param::get("/raceline_optimization/optimizer/decreaser", decreaser_);
  ros::param::get("/raceline_optimization/optimizer/max_displacement", max_displacement_);
}

void RacelineOptimizer::smoothRaceline()
{
  for (int i = 0; i < raceline_current_.size(); i++)
  {
    double sum_x = 0;
    double sum_y = 0;
    int counter = 0;
    // define mean
    for (int j = i - strength_; j < i + strength_; j++)
    {
      if (j < 0)
      {
        sum_x += raceline_current_[raceline_current_.size() - abs(j)].x;
        sum_y += raceline_current_[raceline_current_.size() - abs(j)].y;
      }
      else
      {
        sum_x += raceline_current_[(j + raceline_current_.size() - 1) % raceline_current_.size()].x;
        sum_y += raceline_current_[(j + raceline_current_.size() - 1) % raceline_current_.size()].y;
      }

      counter++;
    }

    raceline_current_[i] = cv::Point2d(sum_x / counter, sum_y / counter);
  }
}

void RacelineOptimizer::useSpline()
{
  CatmullRomSplines crs(raceline_current_, true);
  raceline_current_ = crs.getSplineLine(ppm_);
}

double RacelineOptimizer::calculateLapTime(VelocityProfile vel_prof, std::vector<cv::Point2d> raceline, int count)
{
  std::vector<double> velocities = vel_prof.getVelocityProfile();
  std::vector<double> curvatures = vel_prof.getCurvatureProfile();

  double sum = 0;
  for (int i = 0; i < count; i++)
  {
    sum += velocities[i];
  }
  double average_vel = sum / velocities.size();
  double length = 0;
  for (int i = 0; i < count; i++)
  {
    double delta = sqrt(pow(raceline[i].x - raceline[(i - 1 + count) % count].x, 2) +
                        pow(raceline[i].y - raceline[(i - 1 + count) % count].y, 2));
    length += delta;
  }

  double time = length / average_vel;
  return time;
}

double RacelineOptimizer::calculateSquaredAcceleration(VelocityProfile vel_prof, std::vector<cv::Point2d> raceline)
{
  std::vector<double> accels = vel_prof.getAccelRestriction();
  double accel_sum = 0;
  for (int i = 0; i < raceline.size(); i++)
  {
    double delta_s = sqrt(pow(raceline[i].x - raceline[(i + 1) % raceline.size()].x, 2) +
                          pow(raceline[i].y - raceline[(i + 1) % raceline.size()].y, 2));
    accel_sum += pow(abs(accels[i]), 2) * delta_s;
  }
  return accel_sum;
}

void RacelineOptimizer::optimizeRaceline()
{
  for (int i = 0; i < waypoints_.size(); i++)
  {
    displacements_.push_back(0);
  }
  for (int i = 0; i < loop_; i++)
  {
    double dp_step = dp_factor_;
    int counter = 0;
    int step_drop_counter = 0;
    while (counter < counter_)
    {
      std::cout << "counter: " << counter << std::endl;
      if (abs(time_before_ - time_current_) < 0.000001)
      {
        if (step_drop_counter % 2 != 0)
        {
          dp_step = dp_step * increaser_;
        }
        else if (step_drop_counter % 2 == 0)
        {
          dp_step = dp_step * decreaser_;
        }

        step_drop_counter++;
      }
      time_before_ = time_current_;

      for (int i = 0; i < waypoints_.size(); i++)
      {
        // Get locations of neighbour nodes
        cv::Point2d point_right = waypoints_[(i + 1) % waypoints_.size()];
        cv::Point2d point_left = waypoints_[(i + waypoints_.size() - 1) % waypoints_.size()];
        cv::Point2d point_middle = waypoints_[i];

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
        CatmullRomSplines crs(waypoints_, true);
        cv::Point2d gradient = crs.getSplineGradient(i, true);

        double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
        gradient.x /= g_length;
        gradient.y /= g_length;

        // Project required correction onto point tangent to give displacment
        double dp_left = (-gradient.y * vector_sum.x + gradient.x * vector_sum.y) * dp_step;
        double dp_right = (-gradient.y * vector_sum.x + gradient.x * vector_sum.y) * -dp_step;

        // create new waypoints track
        std::vector<cv::Point2d> waypoints_left = waypoints_;
        std::vector<cv::Point2d> waypoints_right = waypoints_;

        waypoints_left[i].x += -gradient.y * dp_left;
        waypoints_left[i].y += gradient.x * dp_left;

        waypoints_right[i].x += -gradient.y * dp_right;
        waypoints_right[i].y += gradient.x * dp_right;

        // calculate the new velocity profiles
        VelocityProfile vel_left(waypoints_left);
        VelocityProfile vel_right(waypoints_right);

        // calculate lap times / cost function

        double time_left = calculateLapTime(vel_left, waypoints_left, i);
        double time_right = calculateLapTime(vel_right, waypoints_right, i);

        if ((time_left < time_right && time_left < time_current_) ||
            (displacements_[i] - dp_step > -max_displacement_ && time_left < time_current_))
        {
          if (abs(displacements_[i]) + dp_step < max_displacement_)
          {
            time_current_ = time_left;
            vel_current_ = vel_left;
            raceline_current_ = waypoints_left;
            waypoints_ = waypoints_left;
            displacements_[i] += dp_step;
          }
        }
        else if ((time_right < time_left && time_right < time_current_) ||
                 (displacements_[i] + dp_step < max_displacement_ && time_right < time_current_))
        {
          if (abs(displacements_[i]) + dp_step < max_displacement_)
          {
            time_current_ = time_right;
            vel_current_ = vel_right;
            raceline_current_ = waypoints_right;
            waypoints_ = waypoints_right;
            displacements_[i] -= dp_step;
          }
        }
      }

      counter++;
    }
  }
}

void RacelineOptimizer::optimizeRacelineAccel()
{
  for (int i = 0; i < waypoints_.size(); i++)
  {
    displacements_.push_back(0);
  }
  for (int i = 0; i < loop_; i++)
  {
    double dp_step = 0.5;
    int counter = 0;
    int step_drop_counter = 0;
    while (counter < counter_)
    {
      if (abs(accel_before_ - accel_current_) < 0.000001)
      {
        if (step_drop_counter % 2 != 0)
        {
          dp_step = dp_step * increaser_;
        }
        else if (step_drop_counter % 2 == 0)
        {
          dp_step = dp_step * decreaser_;
        }

        step_drop_counter++;
      }

      accel_before_ = accel_current_;

      for (int i = 0; i < waypoints_.size(); i++)
      {
        // Get locations of neighbour nodes
        cv::Point2d point_right = waypoints_[(i + 1) % waypoints_.size()];
        cv::Point2d point_left = waypoints_[(i + waypoints_.size() - 1) % waypoints_.size()];
        cv::Point2d point_middle = waypoints_[i];

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
        CatmullRomSplines crs(waypoints_, true);
        cv::Point2d gradient = crs.getSplineGradient(i, true);

        double g_length = sqrtf(gradient.x * gradient.x + gradient.y * gradient.y);
        gradient.x /= g_length;
        gradient.y /= g_length;

        // Project required correction onto point tangent to give displacment
        double dp_left = (-gradient.y * vector_sum.x + gradient.x * vector_sum.y) * dp_step;
        double dp_right = (-gradient.y * vector_sum.x + gradient.x * vector_sum.y) * -dp_step;

        // create new waypoints track
        std::vector<cv::Point2d> waypoints_left = waypoints_;
        std::vector<cv::Point2d> waypoints_right = waypoints_;

        waypoints_left[i].x += -gradient.y * dp_left;
        waypoints_left[i].y += gradient.x * dp_left;

        waypoints_right[i].x += -gradient.y * dp_right;
        waypoints_right[i].y += gradient.x * dp_right;

        // calculate the new velocity profiles
        VelocityProfile vel_left(waypoints_left);
        VelocityProfile vel_right(waypoints_right);

        // calculate lap times / cost function

        double accel_left = calculateSquaredAcceleration(vel_left, waypoints_left);
        double accel_right = calculateSquaredAcceleration(vel_right, waypoints_right);

        if ((accel_left < accel_right && accel_left < accel_current_) ||
            (displacements_[i] - dp_step > -max_displacement_ && accel_left < accel_current_))
        {
          if (abs(displacements_[i]) + dp_step < max_displacement_)
          {
            accel_current_ = accel_left;
            vel_current_ = vel_left;
            raceline_current_ = waypoints_left;
            waypoints_ = waypoints_left;
            displacements_[i] += dp_step;
          }
        }
        else if ((accel_right < accel_left && accel_right < accel_current_) ||
                 (displacements_[i] + dp_step < max_displacement_ && accel_right < accel_current_))
        {
          if (abs(displacements_[i]) + dp_step < max_displacement_)
          {
            accel_current_ = accel_right;
            vel_current_ = vel_right;
            raceline_current_ = waypoints_right;
            waypoints_ = waypoints_right;
            displacements_[i] -= dp_step;
          }
        }
      }

      counter++;
    }
  }
}

std::vector<cv::Point2d> RacelineOptimizer::getCurrentRaceline()
{
  return raceline_current_;
}

VelocityProfile RacelineOptimizer::getCurrentVelocityProfile()
{
  return vel_current_;
}