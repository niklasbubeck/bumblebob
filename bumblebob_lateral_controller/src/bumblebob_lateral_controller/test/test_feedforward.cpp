#include <gtest/gtest.h>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include "feedback.hpp"
#include "feedforward.hpp"
#include "lateral_kinematic_ROS_node.hpp"
#include "../../bumblebob_navigation/src/raceline.hpp"

/*
 * Init values
 */

std::vector<cv::Point2d> bluecones_ = {
  { -1.766, 1.47 },   { 2.76, 1.72 },     { 7.278, 1.727 },    { 12.034, 1.777 },
  { 16.539, 1.857 },  { 21.033, 1.905 },  { 25.824, 1.823 },   { 29.008, 1.688 },
  { 32.853, 1.532 },  { 36.054, 1.406 },  { 41.841, 0.4617 },  { 46.155, -4.625 },
  { 46.422, -7.103 }, { 46.532, -9.607 }, { 46.376, -11.952 }, { 45.84, -14.408 }
};
std::vector<cv::Point2d> yellowcones_ = { { -1.205, -2.434 }, { 3.17, -1.932 },   { 7.485, -1.813 },
                                          { 12.131, -1.683 }, { 16.790, -1.634 }, { 21.015, -1.577 },
                                          { 25.949, -1.68 },  { 29.048, -1.728 }, { 32.611, -1.836 },
                                          { 35.958, -2.025 }, { 39.908, -3.049 }, { 42.197, -5.731 },
                                          { 42.643, -7.457 }, { 42.622, -9.841 }, { 42.524, -11.47 },
                                          { 42.159, -13.486 } };

RaceLine raceline(bluecones_, yellowcones_, 0.25, 40, 20);
std::vector<cv::Point2d> racingline = raceline.calculateMappedRacingLineMiddle();
cv::Point2d vehicle_position = { 0, 0 };
double speed = 2;
double heading = 0.1;

/*
 * Test method findNearesPoint
 */

TEST(FeedForwardTest, testFindNearestPoint)
{
  FeedForward feedforward(vehicle_position, speed, racingline, heading);
  int index = feedforward.findNearestPoint();
  ASSERT_EQ(index, 50);
}

/*
 * Test method defineRelevantPoints
 */

TEST(FeedForwardTest, testDefineRelevantPoints)
{
  FeedForward ff(vehicle_position, speed, racingline, heading);
  ff.defineRelevantPoints();
  ASSERT_EQ(ff.relevant_points_.size(), 20);
  double distance_first = sqrt(pow(ff.relevant_points_[0].x - vehicle_position.x, 2) +
                               pow(ff.relevant_points_[0].y - vehicle_position.y, 2));
  double distance_last = sqrt(pow(ff.relevant_points_[ff.relevant_points_.size()].x - vehicle_position.x, 2) +
                              pow(ff.relevant_points_[ff.relevant_points_.size()].y - vehicle_position.y, 2));
  ASSERT_LT(distance_first, 10);
  ASSERT_LT(distance_last, 10);
}

/*
 * Test method leastSquareCircle
 */

TEST(FeedForwardTest, testLeastSquareCircle)
{
  FeedForward ff(vehicle_position, speed, racingline, heading);
  ff.defineRelevantPoints();
  ff.leastSquareCircle();

  ASSERT_NEAR(ff.center_.x, 0.18, 1);
  ASSERT_NEAR(ff.center_.y, 0.74, 1);
  ASSERT_NEAR(ff.radius_, 0.74, 1);
  ASSERT_FALSE(ff.straight_line_);
}

/*
 * Test method calculateFeedforwardAngle
 */

TEST(FeedForwardTest, testCalculateFeedforwardAngle)
{
  FeedForward ff(vehicle_position, speed, racingline, heading);
  ff.defineRelevantPoints();
  ff.leastSquareCircle();
  ff.calculateFeedforwardAngle();
  ASSERT_NEAR(ff.feedforward_angle_, -1.12, 0.01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
