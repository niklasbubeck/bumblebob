#ifndef FEEDBACK_H
#define FEEDBACK_H

#include <iostream>
#include <math.h>
#include <opencv2/core/types.hpp>
#include <eigen3/Eigen/Dense>

#include "feedforward.hpp"

/**
 *  Implementation of the feedback part from the lateral kinematic controller.
 * 	This class includes several methods for estimating the control errors for the three degrees of freedom. Lateral
 *  error, Heading error, Angular Velocity.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.1
 */
class Feedback
{
public:
  // The overall feedback_angle
  float feedback_angle_;

  // The heading of the path
  double road_heading_;

  // The difference between the neares path point and the position
  double lateral_error_;

  // The difference between the heading of the road and the vehicle
  double heading_error_;

  /**
   * @brief Constructor.
   *
   * @param ff A Pointer to the FeedForward instance
   * @param vehicle_position The position of the vehicle
   * @param vehicle_heading The heading of the vehicle
   * @param angular_velocity The angular velocity of the vehicle
   * @param waypoints A reference to the path
   * @param k_lat The lateral gain parameter
   * @param k_head The heading gain parameter
   * @param k_o The change of steering gain parameter
   */

  Feedback(FeedForward& ff, cv::Point2d vehicle_position, double vehicle_heading, double angular_velocity,
           std::vector<cv::Point2d>& waypoints, double k_lat, double k_head, double k_o);

  /**
   * @brief Getter method for the overall feedback angle
   *
   * @return The overall feedback angle
   */
  double getFeedbackAngle()
  {
    return feedback_angle_;
  }

  /**
   * @brief Calculates the lateral error with a direct solution by defining the shortest distance to the path as error
   *
   * @return the feedback of the lateral error
   *
   */
  double calculateLateralErrorDirect();

  /**
   * @brief Calculates the heading error with a geometric solution by defining the distance from the circles center
   * point and the path radius
   *
   * @return the feedback of the heading
   *
   */
  double calculateHeadingError();

  /**
   * @brief Calculates the feedback from the angular velocity
   *
   * @return the feedback of the angular velocity
   *
   */
  double calculateDeltaDotError();

private:
  // Pointer to the FeedForward instance
  FeedForward& ff_;

  // The vehicle position
  cv::Point2d vehicle_position_;

  // The heading of the vehicle
  double heading_;

  // The angular_velocity of the vehicle
  double angular_velocity_;

  // The lateral gain parameter
  double k_lat_;

  // The heading gain parameter
  double k_head_;

  // The change of steering gain parameter
  double k_o_;
};

#endif