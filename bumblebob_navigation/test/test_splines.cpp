#include "../src/splines.hpp"
#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>

/*
 * Test method getSplineLine
 * Main Test!
 * The defined points are randomly chosen
 */
TEST(TestSuite, testgetSplineLine)
{
  std::vector<cv::Point2d> points = { { 10, 41 } };
  CatmullRomSplines crs(points, true);
  std::vector<cv::Point2d> spline = crs.getSplineLine(1.0);
  ASSERT_EQ(points, spline) << "Numbers are equal";
}

/*
 * Test method getInitPoints
 * The defined points are randomly chosen
 */
TEST(TestSuite, testgetInitPoints)
{
  std::vector<cv::Point2d> points = { { 10, 41 }, { 40, 50 }, { 70, 60 }, { 100, 30 } };
  CatmullRomSplines crs(points, true);
  std::vector<cv::Point2d> spline = crs.getInitPoints();
  ASSERT_EQ(points, spline) << "Numbers are equal";
}
/*
 * Test method getSplinePoint
 * The defined points are randomly chosen
 */
TEST(TestSuite, testgetSplinePoint)
{
  std::vector<cv::Point2d> points = { { 10, 41 }, { 40, 50 }, { 70, 60 }, { 100, 30 } };
  CatmullRomSplines crs(points, true);
  cv::Point2d spline = crs.getSplinePoint(0.0, true);
  cv::Point2d res = { 10, 41 };
  ASSERT_EQ(res, spline) << "Numbers are equal";
}

/*
 * Test method getSplineGradient
 * The defined points are randomly chosen
 */

TEST(TestSuite, testgetSplineGradient)
{
  std::vector<cv::Point2d> points = { { 10, 41 }, { 40, 50 }, { 70, 60 }, { 100, 30 } };
  CatmullRomSplines crs(points, true);
  cv::Point2d spline = crs.getSplineGradient(0.0f, true);
  cv::Point2d res = { -30, 10 };
  ASSERT_EQ(res, spline) << "Numbers are equal";
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
