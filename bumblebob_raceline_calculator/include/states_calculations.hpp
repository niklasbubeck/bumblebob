

/* ------------------------------

      Under Construction

--------------------------------*/

#ifndef STATES_CALCULATIONS_H
#define STATES_CALCULATIONS_H

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/core/types.hpp>
#include <eigen3/Eigen/Dense>

#include "velocity_profile.hpp"

class StatesCalculations
{
private:
  VelocityProfile vel_prof_;
  std::vector<Eigen::MatrixXd> states_list_;

  // Rosparameters
  double c_f_;
  double c_r_;
  double length_;
  double weight_f_;
  double weight_r_;
  double l_f_;
  double l_r_;
  double inertia_;
  double mass_;
  double gravity_;
  double tire_coefficient_;
  double tire_correction_;
  double tire_deg_;
  double sa_;
  int counter_ = 0;

  /**
   * @brief reads the rosparameters
   */
  void readParams();

  /**
   * @brief calculates the tire stiffnes for the front wheels
   */
  void calculateCf();

  /**
   * @brief calculates the tire stiffnes for the rear wheels
   */
  void calculateCr();

  /**
   * @brief sets the initial state with zeros
   */
  void setInitialState();

  /**
   * @brief calculates the next state depending on the one before
   *
   * @param velocity the velocity of the vehicle
   * @param curvature the curvature of the path
   * @param old_state the old state of the control
   */
  Eigen::MatrixXd calculateNewState(double velocity, double curvature, Eigen::MatrixXd old_state);

  /**
   * @brief calculates the list of the states
   */
  void calculateStateList();

public:
  /**
   * @brief Normal Constructor
   *
   * @param vel_prof the velocity profile of the track
   */
  StatesCalculations(VelocityProfile vel_prof);

  /**
   * @brief getter method for the states list
   */
  std::vector<Eigen::MatrixXd> getStatesList();
};

#endif