#ifndef RACELINE_OPTIMIZER_H
#define RACELINE_OPTIMIZER_H

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/core/types.hpp>

#include "../include/velocity_profile.hpp"
#include "../../bumblebob_navigation/src/splines.hpp"

/**
 *  Implementation of the RaceLine Optimizer Algorithm
 *
 *  Based on the Velocity Profile the algorithm iterate over the waypoints and tries to find a better solution based on
 *a costfunction which is the overall laptime.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.0
 */
class RacelineOptimizer
{
private:
  /**
   * @brief reads the rosparameters given in the /config/config.yaml file, from the rosparam server.
   */
  void readParams();

  /**
   * @brief calculates the trackwidth of certain positions given as waypoints
   *
   * @param middlepoint a set of waypoints
   *
   */
  double calculateTrackWidth(cv::Point2d middlepoint);

  /**
   * @brief smooths the raceline
   */
  void smoothRaceline();

  /**
   * @brief optimizes the raceline (main algorithm)
   */
  void optimizeRaceline();

  /**
   * @brief optimizes the raceline based on least squared acceleration
   */
  void optimizeRacelineAccel();

  /**
   * @brief calculates the overall lap time based on the velocty profile and the length of the raceline
   *
   * @param vel_prof The velocity_procile
   * @param raceline The current raceline
   *
   * @return the lap time
   */
  double calculateLapTime(VelocityProfile vel_prof, std::vector<cv::Point2d> raceline, int count);

  /**
   * @brief calculate the squared acceleration
   *
   * @param vel_prof The velocity profile
   * @param raceline The current raceline
   *
   * @return the squared acceleration
   */
  double calculateSquaredAcceleration(VelocityProfile vel_prof, std::vector<cv::Point2d> raceline);

  /**
   * @brief perform the spline on the current raceline
   */
  void useSpline();

  // Track restriction as yellow and blue cones
  std::vector<cv::Point2d> yellcones_;
  std::vector<cv::Point2d> bluecones_;
  // A set of waypoints
  std::vector<cv::Point2d> waypoints_;
  // The width of the vehicle
  double vehicle_width_;
  // The density of the Spline
  int ppm_;
  // The strenght parameter to smooth
  int strength_;
  // The initial displacement factor
  double dp_factor_;

  // the current velocity profile
  VelocityProfile vel_current_;

  // initial costfunction values
  double time_current_;
  double time_before_;
  double accel_current_;
  double accel_before_;

  int loop_;
  int counter_;
  double increaser_;
  double decreaser_;
  double max_displacement_;

public:
  // vector of the displcements of the specific waypoints
  std::vector<double> displacements_;
  // the current raceline
  std::vector<cv::Point2d> raceline_current_;
  /**
   * @brief Normal Constructor
   *
   * @param vel_prof The velocity profile
   * @param bluecones The blue cones of the restriction line
   * @param yellcones The yellow cones of the restriction line
   * @param waypoints A reference path for the initial estimations
   */
  RacelineOptimizer(VelocityProfile vel_prof, std::vector<cv::Point2d> bluecones, std::vector<cv::Point2d> yellcones,
                    std::vector<cv::Point2d> waypoints);
  /**
   * @brief Getter for the current Raceline
   */
  std::vector<cv::Point2d> getCurrentRaceline();

  /**
   * @brief Getter for the current Raceline
   */
  VelocityProfile getCurrentVelocityProfile();
};

#endif