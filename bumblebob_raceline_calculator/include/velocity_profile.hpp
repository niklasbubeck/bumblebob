#ifndef VELOCITY_PROFILE_H
#define VELOCITY_PROFILE_H

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

/**
 *  Implementation of the Velocity and Curvature Profiles
 * 	This class includes several methods for estimating said profiles depending on a set of given Waypoints. This also
 *  includes restrictions in terms of acceleration and general vehicle kinematics.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.0
 */
class VelocityProfile
{
private:
  /**
   * @brief reads the rosparameters given in the /config/config.yaml file, from the rosparam server.
   */
  void readParams();

  /**
   * @brief calculates the curvature based on matrix calculation. A SVD is used to find a circle that fits the best for
   * all points. The result gets better if more points are included to the calculations.
   *
   * @param relevant_points Points that define the circle.
   *
   * @return The curvature
   */
  double calculateCurvature(std::vector<cv::Point2d> relevant_points);

  double solveLeastSquaresCircleKasa(std::vector<cv::Point2d> relevant_points);

  /**
   * @brief calculates the initial velocity and curvature profile
   */
  void estimateInitialVelocityProfile();

  /**
   * @brief iterates over the velocity profile and restricts it based on the maximum acceleration
   */
  void forwardStep();

  /**
   * @brief iterates over the velocity profile and restricts it based on the maximum deceleration
   */
  void backwardStep();

  /**
   * @brief calculates the length profile, often refered to as "s"
   */
  void calculateLengthProfile();

  /**
   * @brief imitates the python modulo function as a % b
   *
   * @param a first value
   * @param b second value
   *
   * @return the modulo value as in python
   */
  int pythonModulo(int a, int b);

  /**
   * @brief calculates the maximum acceleration for a given curvature
   */
  void estimateMaxCurvAcceleration();

  /**
   * @brief a geometric solution to calculate the curvature of a given set of points.
   *
   * @param relevant_points The set of points the curvature is calculated for
   *
   * @return The curvature
   */
  double geometricCircleFit(std::vector<cv::Point2d> relevant_points);

  // vector that contains the velocity values
  std::vector<double> velocity_profile_;
  // set of waypoints (a reference path) that the profiles are calculated for
  std::vector<cv::Point2d> middle_line_;

  // Values for the area estimation. Determines how many points will be used for the curvature calculation
  int look_ahead_;
  int look_back_;

  // Friction value between rubber and asphalt
  double friction_;
  // The gravity. Usually 9.81 as we are currently based on earth
  double gravity_;
  // The maximum Acceleration Forces used for the restrictions
  double max_accel_;
  double max_decel_;
  // The mass of the vehicle
  double mass_;

  // Set of center points (circle center) from the curvature calculations
  std::vector<cv::Point2d> center_points_;
  // vector that contains the curvature values
  std::vector<double> curvature_profile_;
  // vector that contains the length values
  std::vector<double> length_profile_;
  // vector that contains the initial velocity values of the middle_line_
  std::vector<double> velocity_initial_;
  // vector that contains the curve acceleration limitations
  std::vector<double> accel_restriction_;

public:
  /**
   * @brief Normal Constructor
   *
   * @param middle_line A reference path for the initial estimations
   */
  VelocityProfile(std::vector<cv::Point2d> middle_line);

  /**
   * @brief Getter for the Velocity Profile
   */
  std::vector<double> getVelocityProfile();

  /**
   * @brief Getter for the Curvature Profile
   */
  std::vector<double> getCurvatureProfile();

  /**
   * @brief Getter for the Acceleration Restriction
   */
  std::vector<double> getAccelRestriction();

  /**
   * @brief Getter for the Initial Velocity Profile
   */
  std::vector<double> getInitialVelocity();
};

#endif