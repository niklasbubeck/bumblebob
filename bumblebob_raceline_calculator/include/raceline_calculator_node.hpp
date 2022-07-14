#ifndef RACELINENODE_H
#define RACELINENODE_H

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/core/types.hpp>

// msgs imports
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "bumblebob_msgs/PointArray.h"
#include "bumblebob_msgs/ConeArray.h"
#include "velocity_profile.hpp"
#include "states_calculations.hpp"
#include "../../bumblebob_navigation/src/raceline.hpp"

/**
 *  Implementation of the RaceLine Calculator Node
 *
 *  This class takes care of the ROS stuff within the bumblebob_raceline_calculator package
 *
 *	@author Niklas Bubeck
 *	@version 1.0.0
 */
class RacelineCalculatorNode
{
private:
  /**
   * @brief gets the map from the slam as a ConeArray and saves the data for later purposes
   *
   * @param msg the rosmsg of the callback
   *
   */
  void mapCallback(const bumblebob_msgs::ConeArray& msg);

  /**
   * @brief gets the reference path from the pathplanner and saves the data for later purposes
   *
   * @param msg the rosmsg of the callback
   *
   */
  void referenceCallback(const bumblebob_msgs::PointArray& msg);

  /**
   * @brief reads the rosparameters given in the /config/config.yaml file, from the rosparam server.
   */
  void readParams();

  /**
   * @brief waits until the neccessary information from other nodes arrived
   *
   */
  void waitForReference();

  /**
   * @brief writes all the important data into a csv file
   *
   */
  void writeDataToCSV();

  /**
   * @brief doubles the amount of waypoints from the reference line to get a better representation
   *
   */
  void doubleTheWaypoints();

  /**
   * @brief builds the raceline msg which will be published later
   *
   */
  void buildRacelineMsg();

  /**
   * @brief builds the velocity profile msg which will be published later
   *
   */
  void buildVelocityProfileMsg();
  // csv data

  // ros node handle
  ros::NodeHandle node_;

  // node frequency
  ros::Rate loop_rate_;

  bool write_csv_;

  // Publishers
  ros::Publisher pub_raceline_;
  ros::Publisher pub_velocity_profile_;

  // Subscribers
  ros::Subscriber sub_map_;
  ros::Subscriber sub_reference_;

  // Pub and Sub strings
  std::string read_pub_raceline_;
  std::string read_sub_map_;
  std::string read_sub_reference_;
  std::string read_pub_velocity_profile_;
  std::string file_path_;

  // stored input data
  std::vector<bumblebob_msgs::Cone> cone_array_;
  std::vector<cv::Point2d> cone_positions_;
  std::vector<cv::Point2d> reference_line_;
  std::vector<cv::Point2d> waypoints_;
  std::vector<cv::Point2d> blue_cones_;
  std::vector<cv::Point2d> yellow_cones_;

  // the initial estimations
  std::vector<double> initial_velocity_;
  std::vector<double> initial_curv_;

  // the quit criteria for the optimization
  double quit_criteria;

  std::vector<cv::Point2d> restriction_yell_;
  std::vector<cv::Point2d> restriction_blue_;
  std::vector<cv::Point2d> test_;

  // the current optimized raceline
  std::vector<cv::Point2d> raceline_current_;

  // the profiles of the raceline
  std::vector<double> velocity_;
  std::vector<double> curvature_;
  std::vector<double> acceleration_;

  bumblebob_msgs::PointArray pointArrayRaceLine_;
  std_msgs::Float64MultiArray velocity_array_;

  std::ofstream csv_file_;

public:
  /**
   * @brief Normal Constructor
   */
  RacelineCalculatorNode();
  /**
   * @brief updates the loop of the rosnode
   */
  void updateLoop();

  /**
   * @brief initializes the rosnode
   */
  void initialize();
};

#endif
