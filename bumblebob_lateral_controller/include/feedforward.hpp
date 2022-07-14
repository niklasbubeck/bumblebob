#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <gtest/gtest_prod.h>
#include "../include/car_constants.hpp"

/**
 *  Implementation of the feedforward part from the lateral kinematic controller.
 * 	This class includes several methods for estimating the parameters and relevant data from the path such as curvature.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.1
 */
class FeedForward
{
  FRIEND_TEST(FeedForwardTest, testFindNearestPoint);
  FRIEND_TEST(FeedForwardTest, testDefineRelevantPoints);
  FRIEND_TEST(FeedForwardTest, testLeastSquareCircle);
  FRIEND_TEST(FeedForwardTest, testCalculateFeedforwardAngle);

private:
  // Helper functions
  int findNearestPoint();

  /**
   * @brief defines the relevant points that are necessary for the calculation and sets them to a global variable.
   * Hereby, relevant points are a part of the raceline starting from behind the vehicle and ending in front of it.
   */
  void defineRelevantPoints();

  /**
   * @brief An algebraic best fit solution to find the best fitting circle, such as the overall distance from each point
   * to the circles outline is minimized. Also it estimates the direction of the path.
   */
  void leastSquareCircle();

  /**
   * @brief calculates the feedforward angle that is defined from the kinematic vehicle model as the Ackermann steering.
   */
  void calculateFeedforwardAngle();

  // The position of the vehicle
  cv::Point2d vehicle_position_;

  // The velocity of the vehicle
  double speed_;

  // The path to follow as an array of waypoints
  std::vector<cv::Point2d> waypoints_;

  // The heading of the vehicle
  double heading_;

  // Current 3 points
  cv::Point2d circle_points_[3];

  // Value for the area estimation. This will be overwritten by the Node
  int look_front_;

  // Value for the area estimation. This will be overwritten by the Node
  int look_back_;

public:
  /**
   * @brief Normal Constructor
   *
   * @param vehicle_position The position of the vehicle
   * @param speed The velocity of the vehicle
   * @param waypoints The path as an array of waypoints
   * @param heading The heading of the vehicle
   */
  FeedForward(const cv::Point2d vehicle_position, const double speed, std::vector<cv::Point2d>& waypoints,
              double heading);

  /**
   * @brief Runs an Iteration and calculates all the relevant variables
   */
  void runIteration();

  /**
   * @brief Getter method for the overall feedforward angle
   */
  const double getFeedForwardAngle()
  {
    return feedforward_angle_;
  }

  /**
   * @brief Setter for the look_back parameter. Used by the dynamic reconfigure and the rosparam loader
   */
  void setLookBack(int new_value)
  {
    look_back_ = new_value;
  }

  /**
   * @brief Setter for the look_front parameter. Used by the dynamic reconfigure and the rosparam loader
   */
  void setLookFront(int new_value)
  {
    look_front_ = new_value;
  }

  // **** exclusive debug output ******
  // nearest point to the vehicles position
  cv::Point2d nearest_point_;

  // Curving left = 1, curving right = -1
  int circle_direction_;

  // Radius of the best fit circle. Radius of the curve for the interval
  // [vehicle_position - lookback, vehicle_position + lookfront]
  double radius_;

  // The circle's center
  cv::Point2d center_;

  // A list of relevant points from the raceline that the calculations are based on.
  std::vector<cv::Point2d> relevant_points_;

  // Check for circle points that make a straight line
  bool straight_line_;

  // Overall feedforward angle
  float feedforward_angle_;
};

#endif
