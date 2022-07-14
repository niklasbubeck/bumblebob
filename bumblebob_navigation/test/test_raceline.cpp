#include "../src/splines.hpp"
#include "../src/raceline.hpp"
#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>

/*
 * Test method calculateMappedRacinglineShortest
 * The defined points are randomly chosen
 * Also the points from the racingline are randomly chosen.
 */
TEST(TestSuite, testcalculateMappedRacinglineShortest)
{
  std::vector<cv::Point2d> bluecones = { { -8, 0 }, { -3, 3 }, { 2, 8 },  { 6, 3 },
                                         { 9, 0 },  { 5, -7 }, { 0, -5 }, { -7, -5 } };
  std::vector<cv::Point2d> yellowcones = { { -6, -0.5 }, { -2, 2 },     { 2, 6 },  { 4, 3 },
                                           { 7, 0 },     { 4.5, -4.5 }, { 0, -3 }, { -6, -4 } };

  RaceLine raceline(bluecones, yellowcones, 0.25, 10, 10);
  std::vector<cv::Point2d> racingline = raceline.calculateMappedRacinglineShortest(150);
  cv::Point2d ptIndex0 = { -6.4547, -0.561599 };
  cv::Point2d ptIndex148 = { 6.189, 2.41391 };
  ASSERT_NEAR(ptIndex0.x, racingline[0].x, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex0.y, racingline[0].y, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex148.x, racingline[148].x, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex148.y, racingline[148].y, 0.1) << "Points are equal";
}

/*
 * Test method calculateMappedRacinglineLeastCurvature
 * The defined points are randomly chosen
 * Also the points from the racingline are randomly chosen.
 */
TEST(TestSuite, testcalculateMappedRacinglineLeastCurvature)
{
  std::vector<cv::Point2d> bluecones = { { -8, 0 }, { -3, 3 }, { 2, 8 },  { 6, 3 },
                                         { 9, 0 },  { 5, -7 }, { 0, -5 }, { -7, -5 } };
  std::vector<cv::Point2d> yellowcones = { { -6, -0.5 }, { -2, 2 },     { 2, 6 },  { 4, 3 },
                                           { 7, 0 },     { 4.5, -4.5 }, { 0, -3 }, { -6, -4 } };

  RaceLine raceline(bluecones, yellowcones, 0.25, 10, 10);
  std::vector<cv::Point2d> racingline = raceline.calculateMappedRacinglineLeastCurvature(150);
  cv::Point2d ptIndex80 = { 0.849907, 5.80919 };
  cv::Point2d ptIndex118 = { 4.05368, 4.98019 };
  ASSERT_NEAR(ptIndex80.x, racingline[80].x, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex80.y, racingline[80].y, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex118.x, racingline[118].x, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex118.y, racingline[118].y, 0.1) << "Points are equal";
}

/*
 * Test method calculateVisualRacinglineMiddle
 * The defined points are randomly chosen
 * Also the points from the racingline are randomly chosen.
 */
TEST(TestSuite, testcalculateVisualRacinglineMiddle)
{
  std::vector<cv::Point2d> bluecones = { { -8, 0 }, { -3, 3 }, { 2, 8 },  { 6, 3 },
                                         { 9, 0 },  { 5, -7 }, { 0, -5 }, { -7, -5 } };
  std::vector<cv::Point2d> yellowcones = { { -6, -0.5 }, { -2, 2 },     { 2, 6 },  { 4, 3 },
                                           { 7, 0 },     { 4.5, -4.5 }, { 0, -3 }, { -6, -4 } };

  RaceLine raceline(bluecones, yellowcones, 0.25, 10, 10);
  std::vector<cv::Point2d> racingline = raceline.calculateVisualRacingLineMiddle();
  cv::Point2d ptIndex63 = { -1.14972, 3.93146 };
  cv::Point2d ptIndex96 = { 1.87031, 6.98075 };
  ASSERT_NEAR(ptIndex63.x, racingline[63].x, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex63.y, racingline[63].y, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex96.x, racingline[96].x, 0.1) << "Points are equal";
  ASSERT_NEAR(ptIndex96.y, racingline[96].y, 0.1) << "Points are equal";
}

/*
 * Test method calculateRestriction
 * The defined points are randomly chosen
 * Also the points from the racingline are randomly chosen.
 */
TEST(TestSuite, testcalculateRestriction)
{
  std::vector<cv::Point2d> bluecones = { { -8, 0 }, { -3, 3 }, { 2, 8 },  { 6, 3 },
                                         { 9, 0 },  { 5, -7 }, { 0, -5 }, { -7, -5 } };
  std::vector<cv::Point2d> yellowcones = { { -6, -0.5 }, { -2, 2 },     { 2, 6 },  { 4, 3 },
                                           { 7, 0 },     { 4.5, -4.5 }, { 0, -3 }, { -6, -4 } };

  RaceLine raceline(bluecones, yellowcones, 0.25, 10, 10);
  Color clr = blue;
  std::vector<cv::Point2d> racingline = raceline.calculateRestriction(clr, true);
  cv::Point2d ptIndex2 = { -7.15786, 0.293985 };
  cv::Point2d ptIndex158 = { 5.58775, 3.14518 };
  ASSERT_NEAR(ptIndex2.x, racingline[2].x, 0.1) << "Points are not equal";
  ASSERT_NEAR(ptIndex2.y, racingline[2].y, 0.1) << "Points are not equal";
  ASSERT_NEAR(ptIndex158.x, racingline[158].x, 0.1) << "Points are not equal";
  ASSERT_NEAR(ptIndex158.y, racingline[158].y, 0.1) << "Points are not equal";
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
