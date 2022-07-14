#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../src/raceline.hpp"
#include "../src/splines.hpp"
#include "geometry_msgs/Point.h"
#include <bumblebob_msgs/PointArray.h>

/*
 *   This file visualizes the racingline by publishing on the topics below. The topics are subscribed by the
 * lane_visualizer_node which is responsible for the visualisation of an array of points.
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_line_publisher");

  ros::NodeHandle n;

  ros::Publisher race_line_pub = n.advertise<bumblebob_msgs::PointArray>("bumblebob/raceline", 1000);
  ros::Publisher blue_line_pub = n.advertise<bumblebob_msgs::PointArray>("bumblebob/cone_restriction_blue", 1000);
  ros::Publisher yellow_line_pub = n.advertise<bumblebob_msgs::PointArray>("bumblebob/cone_restriction_yellow", 1000);

  ros::Rate loop_rate(10);

  int count = 0;

  std::vector<cv::Point2d> bluecones = { { -8, 0 }, { -3, 3 }, { 2, 8 },  { 6, 3 },
                                         { 9, 0 },  { 5, -7 }, { 0, -5 }, { -7, -5 } };
  std::vector<cv::Point2d> yellowcones = { { -6, -0.5 }, { -2, 2 },     { 2, 6 },  { 4, 3 },
                                           { 7, 0 },     { 4.5, -4.5 }, { 0, -3 }, { -6, -4 } };

  std::vector<cv::Point2d> bluecones1 = { { -2, 0 }, { -1, 2 },  { 1, 4 },   { 2, 6 },
                                          { -1, 8 }, { -4, 10 }, { -2, 12 }, { -2, 14 } };

  std::vector<cv::Point2d> yellowcones1 = { { 2, 0 }, { 3, 2 },  { 5, 4 },  { 6, 6 },
                                            { 3, 8 }, { 0, 10 }, { 2, 12 }, { 2, 14 } };

  RaceLine raceline(bluecones, yellowcones, 0.25, 40, 20);

  // uncomment to visualize the shortest raceline
  std::vector<cv::Point2d> racingline = raceline.calculateMappedRacinglineShortest(150);

  /*uncomment to visualize the least curvature raceline */
  // std::vector<cv::Point2d> racingline = raceline.calculateMappedRacinglineLeastCurvature(150);

  // uncomment to visualize the middle raceline for the visual area. Also use the bluecones1 and yellowcones1.
  // std::vector<cv::Point2d> racingline = raceline.calculateVisualRacingLineMiddle();

  // uncomment to visualize the restriction line: true --> blue ; false --> yellow
  // std::vector<cv::Point2d> racingline = raceline.calculateRestriction(true, false);

  // uncomment for mapped area.

  CatmullRomSplines crsBlue(bluecones, true);
  CatmullRomSplines crsYellow(yellowcones, true);
  std::vector<cv::Point2d> blueline = crsBlue.getSplineLine(0.05);
  std::vector<cv::Point2d> yellowline = crsYellow.getSplineLine(0.05);

  // uncomment for visual area.

  // CatmullRomSplines crsBlue(bluecones1, false);
  // CatmullRomSplines crsYellow(yellowcones1, false);
  // std::vector<cv::Point2d> blueline = crsBlue.getSplineLine(0.05);
  // std::vector<cv::Point2d> yellowline = crsYellow.getSplineLine(0.05);

  while (ros::ok())
  {
    bumblebob_msgs::PointArray pointArrayRaceLine;

    for (int i = 0; i < racingline.size(); i++)
    {
      geometry_msgs::Point point;
      point.x = racingline[i].x;
      point.y = racingline[i].y;
      point.z = 0.0;

      pointArrayRaceLine.position.push_back(point);
    }

    race_line_pub.publish(pointArrayRaceLine);

    bumblebob_msgs::PointArray pointArrayBlueLine;
    for (int i = 0; i < blueline.size(); i++)
    {
      geometry_msgs::Point point;
      point.x = blueline[i].x;
      point.y = blueline[i].y;
      point.z = 0.0;

      pointArrayBlueLine.position.push_back(point);
    }

    blue_line_pub.publish(pointArrayBlueLine);

    bumblebob_msgs::PointArray pointArrayYellowLine;
    for (int i = 0; i < yellowline.size(); i++)
    {
      geometry_msgs::Point point;
      point.x = yellowline[i].x;
      point.y = yellowline[i].y;
      point.z = 0.0;

      pointArrayYellowLine.position.push_back(point);
    }

    yellow_line_pub.publish(pointArrayYellowLine);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
