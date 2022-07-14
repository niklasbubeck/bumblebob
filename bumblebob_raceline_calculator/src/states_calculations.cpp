

/* ------------------------------

      Under Construction

--------------------------------*/

#include "states_calculations.hpp"
#include <eigen3/Eigen/Dense>

StatesCalculations::StatesCalculations(VelocityProfile vel_prof) : vel_prof_(vel_prof)
{
  readParams();
  setInitialState();
  calculateStateList();
}

void StatesCalculations::readParams()
{
  // Read ROS params
  ros::param::get("/raceline_optimization/states/length", length_);
  ros::param::get("/raceline_optimization/states/weight_front", weight_f_);
  ros::param::get("/raceline_optimization/states/weight_rear", weight_r_);
  ros::param::get("/raceline_optimization/states/inertia", inertia_);
  ros::param::get("/raceline_optimization/states/tire_coefficient", tire_coefficient_);
  ros::param::get("/raceline_optimization/states/tire_correction", tire_correction_);
  ros::param::get("/raceline_optimization/states/tire_degression", tire_deg_);
  ros::param::get("/raceline_optimization/states/slip_angle", sa_);
  ros::param::get("/raceline_optimization/profiles/mass", mass_);
  ros::param::get("/raceline_optimization/profiles/gravity", gravity_);

  // Calculate additional Parameters

  l_f_ = length_ * 0.47;
  l_r_ = length_ * 0.53;

  calculateCf();
  calculateCr();
}

void StatesCalculations::calculateCf()
{
  double m_f = mass_ * gravity_ * weight_f_;
  // no aerodynamic devices --> Fz_f = m_f
  c_f_ = 2 * ((m_f * tire_coefficient_ * tire_correction_) - (tire_deg_ * pow(m_f, 2))) / (sa_ * M_1_PI / 180);

  // std::cout << "m_f: " << m_f << std::endl;
  // std::cout << "tire_coeff: " << tire_coefficient_ << std::endl;
  // std::cout << "tire_corr: " << tire_correction_ << std::endl;
  // std::cout << "tire_deg: " << tire_deg_ << std::endl;
  // std::cout << "sa: " << sa_ << std::endl;
  // std::cout << "weight_f: " << weight_f_ << std::endl;
  // std::cout << "mass_: " << mass_ << std::endl;
  // std::cout << "gravity: " << gravity_ << std::endl;
}

void StatesCalculations::calculateCr()
{
  double m_r = mass_ * gravity_ * weight_r_;
  // no aerodynamic devices --> Fz_r = m_r
  c_r_ = 2 * ((m_r * tire_coefficient_ * tire_correction_) - (tire_deg_ * pow(m_r, 2))) / (sa_ * M_1_PI / 180);
}

void StatesCalculations::setInitialState()
{
  // Get the inital states from the starting position
  // A state is defined as {lateral_error, delta_yaw, yaw_rate, beta, yaw}

  // Hardcoded
  ROS_INFO_STREAM("SET INIT STATE");
  Eigen::MatrixXd state(5, 1);
  state << 0, 0, 0, 0, 0;

  states_list_.push_back(state);
  ROS_INFO_STREAM("INIT STATE SET");
}

Eigen::MatrixXd StatesCalculations::calculateNewState(double velocity, double curvature, Eigen::MatrixXd old_state)
{
  // Calculate A matrix
  Eigen::MatrixXd a(5, 5);
  double a_2_2 = -(pow(l_f_, 2) * c_f_ + pow(l_r_, 2) * c_r_) / (inertia_ * velocity);
  double a_2_3 = (l_r_ * c_r_ - l_f_ * c_f_) / inertia_;
  double a_3_2 = (l_r_ * c_r_ - l_f_ * c_f_) / (mass_ * pow(velocity, 2)) - 1;
  double a_3_3 = -(c_f_ + c_r_) / (mass_ * velocity);
  if (counter_ < 1)
  {
    std::cout << "L_f: " << l_f_ << std::endl;
    std::cout << "c_f: " << c_f_ << std::endl;
    std::cout << "l_r: " << l_r_ << std::endl;
    std::cout << "c_r: " << c_r_ << std::endl;
    std::cout << "inertia: " << inertia_ << std::endl;
    std::cout << "mass: " << mass_ << std::endl;
    std::cout << "velocity: " << velocity << std::endl;
  }

  a << 0, velocity, 0, velocity, 0, 0, 0, 1, 0, 0, 0, 0, a_2_2, a_2_3, 0, 0, 0, a_3_2, a_3_3, 0, 0, 0, 1, 0, 0;

  // Calculate b vector

  Eigen::MatrixXd b(5, 1);
  double b_2 = (l_f_ * c_f_) / inertia_;
  double b_3 = c_f_ / (mass_ * velocity);

  b << 0, 0, b_2, b_3, 0;

  // Calculate d vector || Is skipped

  Eigen::MatrixXd d(5, 1);
  double d_1 = -curvature * velocity;
  double d_2 = l_f_ * c_f_;
  double d_3 = 1;

  d << 0, d_1, d_2, d_3, 0;

  // Calculate new state

  double delta = atan(length_ * curvature);
  if (counter_ < 50)
  {
    delta = 0;
  }

  if (counter_ > 50 && counter_ < 60)
  {
    std::cout << "delta: " << delta << std::endl;
    std::cout << "A matrix:  " << a << std::endl;
    std::cout << "B matrix:  " << b << std::endl;
    std::cout << "Old state:  " << old_state << std::endl;
  }

  counter_++;
  Eigen::MatrixXd new_state = a * old_state + b * delta;
  return new_state;
}

void StatesCalculations::calculateStateList()
{
  std::vector<double> velocities = vel_prof_.getVelocityProfile();
  std::vector<double> curvatures = vel_prof_.getCurvatureProfile();
  ROS_INFO_STREAM("CalculateStateList");

  for (int i = 1; i < velocities.size(); ++i)
  {
    Eigen::MatrixXd state =
        calculateNewState(velocities[i], curvatures[i], states_list_[(i - 1) % states_list_.size()]);
    states_list_.push_back(state);
  }
}

std::vector<Eigen::MatrixXd> StatesCalculations::getStatesList()
{
  return states_list_;
}