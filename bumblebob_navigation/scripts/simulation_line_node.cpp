#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "../src/raceline.hpp"
#include "../src/splines.hpp"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>
#include <bumblebob_msgs/PointArray.h>
#include <bumblebob_msgs/ConeArray.h>
/*
 *   This file visualizes the racingline by publishing on the topics below. The topics are subscribed by the
 * lane_visualizer_node which is responsible for the visualisation of an array of points.
 */

std::vector<cv::Point2d> bluecones_;
std::vector<cv::Point2d> yellowcones_;
std::vector<cv::Point2d> orangecones_;
std::vector<cv::Point2d> bigorangecones_;

std::vector<cv::Point2d> racingline_;
std::vector<cv::Point2d> blueline_;
std::vector<cv::Point2d> yellowline_;

// publisher
ros::Publisher race_line_pub;
ros::Publisher blue_line_pub;
ros::Publisher yellow_line_pub;
// subscriber
ros::Subscriber camera_cone_sub;

void calculateLine()
{
  // // adding a row of cones to the front
  // cv::Point2d diffblue = bluecones_[1] - bluecones_[0];
  // cv::Point2d newblue = bluecones_[0] - diffblue;

  // cv::Point2d diffyellow = yellowcones_[1] - yellowcones_[0];
  // cv::Point2d newyellow = yellowcones_[0] - diffyellow;

  // bluecones_.insert(bluecones_.begin(), newblue);
  // yellowcones_.insert(yellowcones_.begin(), newyellow);

  // bluecones_ = { { -1.766, 1.470 }, { 25.82, 1.82 },    { 43.84, 0.46 },   { 47, -11.1 },     { 43.66, -19.0 },
  //                { 21.93, -28.53 }, { 11.27, -21.8 },   { 4.9, -17.4 },    { -1.69, -18.2 },  { -4.45, -34.1 },
  //                { -6.45, -40.37 }, { -6.0, -42.8 },    { -2.2, -47.3 },   { 11.9, -47.6 },   { 22.25, -51.9 },
  //                { 29.11, -61.94 }, { 30.16, -68.77 },  { 27.02, -73.90 }, { 11.74, -71.71 }, { -2.95, -67.04 },
  //                { -24.45, -43.8 }, { -22.05, -13.68 }, { -18.4, -6.44 } };
  // yellowcones_ = { { -1.20, -2.43 },   { 25.94, -1.68 },   { 41.20, -3.04 },  { 43.5, -10.5 },   { 40.53, -16.87 },
  //                  { 22.96, -24.49 },  { 13.45, -18.8 },   { 6.75, -13.8 },   { -4.8, -15.4 },   { -8.0, -31.1 },
  //                  { -10.45, -39.1 },  { -9.2, -43.9 },    { -2.45, -51.6 },  { 12.8, -51.9 },   { 21.0, -56.9 },
  //                  { 25.26, -63.42 },  { 26.16, -68.29 },  { 24.64, -70.23 }, { 12.78, -68.22 }, { 1.88, -65.31 },
  //                  { -20.82, -41.28 }, { -18.84, -14.36 }, { -15.56, -8.8 } };

  bluecones_ = {
    { -1.766, 1.47 },     { 2.76, 1.72 },       { 7.278, 1.727 },     { 12.034, 1.777 },    { 16.539, 1.857 },
    { 21.033, 1.905 },    { 25.824, 1.823 },    { 29.008, 1.688 },    { 32.853, 1.532 },    { 36.054, 1.406 },
    { 41.841, 0.4617 },   { 46.155, -4.625 },   { 46.422, -7.103 },   { 46.532, -9.607 },   { 46.376, -11.952 },
    { 45.84, -14.408 },   { 45.092, -16.519 },  { 43.633, -19.095 },  { 41.37, -21.745 },   { 37.89, -24.083 },
    { 34.619, -25.537 },  { 27.435, -27.594 },  { 24.486, -28.271 },  { 21.939, -28.539 },  { 17.776, -27.499 },
    { 14.973, -25.15 },   { 13.464, -23.703 },  { 11.352, -21.93 },   { 9.285, -20.39 },    { 7.4833, -18.919 },
    { 4.775, -17.489 },   { 1.287, -17.333 },   { -1.633, -18.275 },  { -3.179, -21.729 },  { -3.273, -25.020 },
    { -3.077, -27.428 },  { -2.77, -30.082 },   { -3.418, -34.082 },  { -5.170, -37.148 },  { -6.550, -40.391 },
    { -5.864, -44.861 },  { -4.377, -44.948 },  { -2.172, -47.478 },  { 0.147, -48.763 },   { 4.0886, -48.87 },
    { 8.055, -47.916 },   { 11.959, -47.655 },  { 14.959, -47.935 },  { 17.607, -48.675 },  { 19.570, -49.775 },
    { 22.606, -52.18 },   { 25.340, -55.391 },  { 27.526, -58.727 },  { 29.163, -62.033 },  { 30.1, -69.022 },
    { 29.283, -72.073 },  { 26.989, -73.962 },  { 23.764, -74.25 },   { 15.7, -73.03 },     { 11.524, -71.985 },
    { 7.554, -71.014 },   { 3.99, -70.1475 },   { -0.06, -69.102 },   { -3.442, -67.230 },  { -5.490, -65.111 },
    { -7.245, -62.596 },  { -9.36, -60.314 },   { -11.545, -58.312 }, { -14.415, -54.934 }, { -16.518, -52.561 },
    { -18.579, -50.139 }, { -21.812, -46.621 }, { -24.726, -43.475 }, { -25.750, -40.398 }, { -26.047, -36.970 },
    { -25.931, -32.619 }, { -25.72, -28.644 },  { -25.001, -24.676 }, { -24.292, -20.863 }, { -23.305, -17.203 },
    { -22.132, -13.64 },  { -20.788, -9.280 },  { -18.463, -6.412 },  { -15.602, -3.687 },  { -13.574, -2.166 },
    { -11.514, -0.893 },  { -7.043, 0.440 },    { -4.109, 1.109 }
  };
  yellowcones_ = {
    { -1.205, -2.434 },   { 3.17, -1.932 },     { 7.485, -1.813 },    { 12.131, -1.683 },   { 16.790, -1.634 },
    { 21.015, -1.577 },   { 25.949, -1.68 },    { 29.048, -1.728 },   { 32.611, -1.836 },   { 35.958, -2.025 },
    { 39.908, -3.049 },   { 42.197, -5.731 },   { 42.643, -7.457 },   { 42.622, -9.841 },   { 42.524, -11.47 },
    { 42.159, -13.486 },  { 41.456, -15.182 },  { 40.532, -16.879 },  { 38.518, -18.998 },  { 36.203, -20.47 },
    { 32.758, -21.973 },  { 28.473, -23.481 },  { 25.443, -24.171 },  { 22.964, -24.495 },  { 18.79, -23.46 },
    { 16.17, -21.45 },    { 13.56, -18.86 },    { 11.89, -17.364 },   { 10.30, -16.099 },   { 6.799, -13.86 },
    { 4.32, -13.511 },    { 1.12, -13.713 },    { -2.75, -14.16 },    { -4.81, -15.676 },   { -7.152, -19.083 },
    { -8.18, -23.21 },    { -7.91, -27.24 },    { -7.00, -31.045 },   { -8.42, -34.921 },   { -10.34, -39.31 },
    { -9.215, -43.81 },   { -6.92, -47.63 },    { -4.60, -50.54 },    { -2.35, -51.88 },    { -0.165, -52.89 },
    { 2.04, -53.272 },    { 4.157, -53.364 },   { 8.75, -53.057 },    { 12.996, -52.165 },  { 16.446, -52.55 },
    { 19.589, -54.879 },  { 21.404, -57.288 },  { 25.26, -63.393 },   { 26.204, -65.976 },  { 26.147, -68.355 },
    { 24.710, -69.465 },  { 21.122, -70.259 },  { 16.893, -69.464 },  { 12.55, -68.42 },    { 8.614, -67.52 },
    { 5.129, -66.636 },   { 1.29, -65.519 },    { -0.91, -64.00 },    { -3.192, -61.759 },  { -4.889, -59.92 },
    { -7.342, -57.379 },  { -9.324, -55.36 },   { -11.927, -52.585 }, { -13.991, -50.405 }, { -15.855, -48.29 },
    { -17.42, -46.34 },   { -18.96, -44.47 },   { -20.86, -41.12 },   { -21.783, -39.55 },  { -22.12, -36.98 },
    { -22.37, -33.37 },   { -22.28, -29.80 },   { -21.68, -25.67 },   { -20.84, -21.799 },  { -19.733, -18.067 },
    { -18.799, -14.369 }, { -17.489, -10.844 }, { -15.561, -8.869 },  { -13.941, -7.27 },   { -11.83, -5.82 },
    { -9.603, -4.786 },   { -6.357, -3.627 },   { -3.796, -2.904 }
  };

  RaceLine raceline(bluecones_, yellowcones_, 0.25, 40, 20);
  racingline_ = raceline.calculateMappedRacingLineMiddle();
  std::ofstream csv_file;
  csv_file.open("/home/niklas/Documents/data/raceline.csv");
  for (int i = 0; i < racingline_.size(); i++)
  {
    csv_file << -racingline_[i].y << "," << racingline_[i].x << std::endl;
  }
  csv_file.close();
  CatmullRomSplines crsBlue(bluecones_, false);
  CatmullRomSplines crsYellow(yellowcones_, false);
  blueline_ = crsBlue.getSplineLine(0.05);
  yellowline_ = crsYellow.getSplineLine(0.05);
}

void buildMessage()
{
  bumblebob_msgs::PointArray pointArrayRaceLine;

  for (int i = 0; i < racingline_.size(); i++)
  {
    geometry_msgs::Point point;
    point.x = racingline_[i].x;
    point.y = racingline_[i].y;
    point.z = 0.0;

    pointArrayRaceLine.position.push_back(point);
  }

  race_line_pub.publish(pointArrayRaceLine);

  //   bumblebob_msgs::PointArray pointArrayBlueLine;
  //   for (int i = 0; i < blueline_.size(); i++)
  //   {
  //     geometry_msgs::Point point;
  //     point.x = blueline_[i].x;
  //     point.y = blueline_[i].y;
  //     point.z = 0.0;

  //     pointArrayBlueLine.position.push_back(point);
  //   }

  //   blue_line_pub.publish(pointArrayBlueLine);

  //   bumblebob_msgs::PointArray pointArrayYellowLine;
  //   for (int i = 0; i < yellowline_.size(); i++)
  //   {
  //     geometry_msgs::Point point;
  //     point.x = yellowline_[i].x;
  //     point.y = yellowline_[i].y;
  //     point.z = 0.0;

  //     pointArrayYellowLine.position.push_back(point);
  //   }

  //   yellow_line_pub.publish(pointArrayYellowLine);
}

void cameraConeCallback(const bumblebob_msgs::ConeArray::ConstPtr& cone_array)
{
  // std::cout << "test 3 \n";
  // std::vector<cv::Point2d> bluecones;
  // std::vector<cv::Point2d> yellowcones;
  // std::vector<cv::Point2d> orangecones;
  // std::vector<cv::Point2d> bigorangecones;
  // for (std::vector<bumblebob_msgs::Cone>::const_iterator iterator = cone_array->cones.begin();
  //      iterator < cone_array->cones.end(); iterator++)
  // {
  //   cv::Point2d point = { iterator->position.x, iterator->position.y };
  //   if (iterator->confidence.yellow > 0.4)
  //   {
  //     yellowcones.push_back(point);
  //   }
  //   else if (iterator->confidence.blue > 0.4)
  //   {
  //     bluecones.push_back(point);
  //   }
  //   else if (iterator->confidence.orange_small > 0.4)
  //   {
  //     orangecones.push_back(point);
  //   }
  //   else if (iterator->confidence.orange_big > 0.4)
  //   {
  //     bigorangecones.push_back(point);
  //   }
  //   else
  //   {
  //     continue;
  //   }
  // }
  // bluecones_ = bluecones;
  // yellowcones_ = yellowcones;
  // orangecones_ = orangecones;
  // bigorangecones_ = bigorangecones;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_line_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  // publisher
  race_line_pub = n.advertise<bumblebob_msgs::PointArray>("/bumblebob/raceline", 1000);
  blue_line_pub = n.advertise<bumblebob_msgs::PointArray>("/bumblebob/cone_restriction_blue", 1000);
  yellow_line_pub = n.advertise<bumblebob_msgs::PointArray>("/bumblebob/cone_restriction_yellow", 1000);

  calculateLine();

  // subscriber
  // camera_cone_sub = n.subscribe("/bumblebob/lidar/cones", 1, cameraConeCallback);
  while (ros::ok())
  {
    ros::spinOnce();
    buildMessage();
    loop_rate.sleep();
  }

  return 0;
}
