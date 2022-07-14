#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <opencv2/core/types.hpp>

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
ros::Publisher map_array_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_line_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  // publisher
  race_line_pub = n.advertise<bumblebob_msgs::PointArray>("/bumblebob/reference", 1000);
  map_array_pub = n.advertise<bumblebob_msgs::ConeArray>("/bumblebob/map", 1000);

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

  std::vector<cv::Point2d> reference_line;
  for (int i = 0; i < bluecones_.size(); i++)
  {
    reference_line.push_back((bluecones_[i] + yellowcones_[i]) / 2);
  }

  bumblebob_msgs::ConeArray cone_array = {};
  for (int i = 0; i < bluecones_.size(); i++)
  {
    bumblebob_msgs::Cone yell_cone;
    bumblebob_msgs::Cone blue_cone;

    yell_cone.position.x = yellowcones_[i].x;
    yell_cone.position.y = yellowcones_[i].y;
    yell_cone.type = 1;

    blue_cone.position.x = bluecones_[i].x;
    blue_cone.position.y = bluecones_[i].y;
    blue_cone.type = 2;

    cone_array.cones.push_back(blue_cone);
    cone_array.cones.push_back(yell_cone);
  }

  bumblebob_msgs::PointArray pointArrayRaceLine;

  for (int i = 0; i < reference_line.size(); i++)
  {
    geometry_msgs::Point point;
    point.x = reference_line[i].x;
    point.y = reference_line[i].y;
    point.z = 0.0;

    pointArrayRaceLine.position.push_back(point);
  }

  while (ros::ok())
  {
    ros::spinOnce();
    map_array_pub.publish(cone_array);
    race_line_pub.publish(pointArrayRaceLine);
    loop_rate.sleep();
  }

  return 0;
}
