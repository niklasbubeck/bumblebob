#ifndef RACELINE_H
#define RACELINE_H

// #include "ros/ros.h"
#include <vector>
#include <opencv2/core/types.hpp>
#include <string>
#include <math.h>
#include <iostream>

/**
 *  Implementation of the Racingline.
 * 	This class includes several methods for estimating the optimal path for both, visual area and mapped course.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.3
 */

/**
 * Enum class for the color of the cones
 */
enum Color
{
  blue,
  yellow,
  orange,
  big_orange
};

class RaceLine
{
private:
  std::vector<cv::Point2d> bluecones_;
  std::vector<cv::Point2d> yellowcones_;
  double distance_;
  int strength_;
  int ppm_;

  /**
   * @brief Calculates the width of the track depending on the given middlepoint and the shortest distance to the blue
   * restriction and the yellow restriction.
   *
   * @param middlepoint the middlepoint that the calculation is based on.
   * @return the width of the track at the arc length of the middlepoint.
   *
   */
  double calculateTrackWidth(cv::Point2d middlepoint);

  /**
   * @brief smooths the raceline by integrating over a set of points defined by the value strength_.
   *
   * @param raceline the raceline to smooth.
   * @param clr enum class that defines the color
   *
   * @return smoothed raceline
   *
   */

  std::vector<cv::Point2d> smoothRaceline(std::vector<cv::Point2d> raceline);

  /**
   * @brief defines the index of the nearest point to the given middlepoint to find the point later within the
   * restriction vector.
   *
   * @param middlepoint the middlepoint that the calculation is based on.
   * @param clr enum class that defines the color
   * restriction.
   * @return index of the nearest point within the specified restriction vector.
   *
   */

  int getIndexOfNearestPoint(cv::Point2d middlepoint, Color clr);

  /**
   * @brief Calculates the distance of two points.
   *
   * @param pt1 point 1
   * @param pt2 point 2
   * @return the distance between the two points.
   *
   */
  double getDistFromPtToPt(cv::Point2d pt1, cv::Point2d pt2);

  /**
   * @brief Calculates the curvature that is defined by three points
   *
   * @param p1 point 1
   * @param p2 point 2
   * @param p3 point 3
   * @return the curvature
   *
   */
  double calculateCurvature(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3);

  /**
   * @brief calculates the middlepoints of bluecones_ and yellowcones_
   *
   * @return vector of the middlepoints
   *
   */
  std::vector<cv::Point2d> getMiddlePoints();

public:
  /**
   * @brief Constructor
   *
   */
  RaceLine(std::vector<cv::Point2d> bluecones, std::vector<cv::Point2d> yellowcones, double distance, int strength,
           int ppm);

  /**
   * @brief Calculates the restriction that the car is not allowed to cross due to its width. Basically the restriction
   * with an offset of distance_.
   *
   * @param clr enum class for the color
   * @param isLooped defines if the restriction will be calculated for the visual or mapped version.
   * @return an vector of points for the restriction
   *
   */
  std::vector<cv::Point2d> calculateRestriction(Color clr, bool isLooped);

  /**
   * @brief Calculates the racingline defined by the shortest line around the course. nIter should to be defined
   * above 15.
   *
   * @param nIter number of iterations
   * @return vector of points for the shortest racingline
   *
   */
  std::vector<cv::Point2d> calculateMappedRacinglineShortest(int nIter);

  /**
   * @brief Calculates the racingline based on the least overall curvature. nIter should be defined above 15.
   *
   * @param nIter number of iterations
   * @return vector of points for the least curvature racingline.
   *
   */
  std::vector<cv::Point2d> calculateMappedRacinglineLeastCurvature(int nIter);

  /**
   * @brief Calculates the visual racingline based on curvature optimization.
   *
   * @return vector of points for the visual curvature optimized racingline
   *
   */
  std::vector<cv::Point2d> calculateVisualRacinglineCurvature();

  /**
   * @brief Calculates the middleline as racingline for the visual area.
   *
   * @return vector of points for the middleline
   *
   */
  std::vector<cv::Point2d> calculateVisualRacingLineMiddle();

  std::vector<cv::Point2d> calculateMappedRacingLineMiddle();
};

#endif