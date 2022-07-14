#ifndef SPLINES_H
#define SPLINES_H

#include <string>
#include <vector>
#include <opencv2/core/types.hpp>
#include <iostream>

/**
 *  Implementation of a Catmull Rom Spline.
 * 	This class can be used for the path calculation, both for
 * 	visual range and mapped tracks.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.3
 */
class CatmullRomSplines
{
private:
  std::vector<cv::Point2d> points_;
  bool isLooped_;

public:
  /**
   *  @brief default constructor
   */
  CatmullRomSplines(std::vector<cv::Point2d>& points, bool blooped);

  /**
   * @brief copy constructor
   */
  CatmullRomSplines(const CatmullRomSplines& crs);

  /**
   * @brief Getter function for the initial points (middlepoints)
   */
  std::vector<cv::Point2d> getInitPoints();

  /**
   * @brief getter function for bool if calculation is based on looping
   */
  bool isLooped();

  /**
   * Estimation of a single point that lies on the spline.
   *
   * Calculates a single point based on the math explained here:
   * https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
   *
   *
   * A Splines weighing is dependent on its control points. Therefore as we imagine a Spline starting from Point0 and
   * ending at Point3, P1 and P2 control the curve of the spline but do not lie on the spline directly. We have to
   * change this as we also want P1 and P2 being part of the Spline. We think of it the other way around. We connect P1
   * and P2 whereas t lies on the curve between the points and its weight is dependent on the distance to P1 and P2. But
   * we want also P0 and P4 to have an influence on the curve. So basically the functions starting with q_ are the
   * weighing (yaxes) from the points on the point t between two points (scaled from 0 to 1 xaxes).
   *
   *
   * @param t describes the position on the sequence
   * @param isLooped defines rather the calculation, spline will generate
   * 		a loop or only takes a line into account.
   * @returns tx, ty the x and y position of the point
   */
  cv::Point2d getSplinePoint(double t, bool isLooped);

  /**
   * Estimation of the gradient to a point (simply the derivative).
   *
   * Calculates a single point based on the math explained here:
   * https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
   * and Point dependent weighing functions
   *
   *
   * @param t describes the position on the sequence
   * @param isLooped defines rather the calculation, spline will generate
   * 		a loop or only takes a line into account.
   * @return tx, ty output of the gradient (normal vector)
   */
  cv::Point2d getSplineGradient(double t, bool isLooped);

  /**
   * Estimates the entire line of the spline
   *
   *
   * @param points all points that the calculation shall be based on
   * @param increaser defines the output of points
   * @return spline array of points that define the spline
   */
  std::vector<cv::Point2d> getSplineLine(double increaser);
};

#endif
