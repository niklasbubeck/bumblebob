#ifndef LATERALROSNODE_H
#define LATERALROSNODE_H

// general imports
#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <bumblebob_lateral_controller/DynamicConfig.h>
#include <dynamic_reconfigure/server.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/core/types.hpp>

// local imports
#include "../include/car_constants.hpp"
#include "../include/feedback.hpp"
#include "../include/feedforward.hpp"

// msgs imports
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "bumblebob_msgs/PointArray.h"

/**
 * @brief struct for the euler angles within the global frame
 */
struct EulerAngles
{
  double roll, pitch, yaw;
};

/**
 *  Implementation of the kinematic lateral controller
 * 	This class includes the ros communication and the callback handling.
 *
 *	@author Niklas Bubeck
 *	@version 1.0.1
 */
class LateralROSNode
{
public:
  /**
   * @brief Normal constructor of the LateralROSNode class
   */
  LateralROSNode();

  /**
   * @brief initializes the subscribers, publisher, gains and restrictions as also the dynamic reconfigure
   */
  void initialize();

  /**
   * @brief the typical ROS loop
   */
  void updateLoop();

private:
  // Callbacks
  /**
   * @brief callback for the raceline
   */
  void waypointsCallback(const bumblebob_msgs::PointArray::ConstPtr& point_array);

  /**
   * @brief Print out debug output if the param in the config is set to True
   * @param ff Pointer to FeedForward instance
   * @param fb Pointer to Feedback instance
   */
  void printDebugData(FeedForward* ff, Feedback* fb);

  /**
   * @brief callbacks for the states of the simulation like: Position, heading, angular velocity and speed
   */
  void statesCallback(const gazebo_msgs::ModelStates::ConstPtr& states);

  void dynamicReconfigureCallback(bumblebob_lateral_controller::DynamicConfig& config, uint32_t level);

  /**
   * @brief A CSV writer to save all the values.
   *
   * @param position Position of the vehicle
   * @param car_heading Heading of the path
   * @param vehicle_heading Heading of the vehicle
   * @param feedback_angle The angle of the overall feedback loop
   * @param feedforward_angle The angle of the overall feedforward loop
   * @param steerign_cmd The output of the control, steering angle of the steering wheel in deg
   * @param radius The radius of the curves
   * @param lateral_error The differenc between the vehicles position and the path
   * @param heading_error The difference between the vehicle heading and the path heading
   * @param angular_velocity The angular velocity of the vehicle
   */
  void writeDataToCSV(cv::Point2d position, double car_heading, double vehicle_heading, double feedback_angle,
                      double feedforward_angle, double steering_cmd, double radius, double lateral_error,
                      double heading_error, double angular_velocity);

  /**
   * @brief waits for the relevant callbacks to be called
   */
  void waitForWaypoints();

  /**
   * @brief reads all the rosparameters
   */
  void readParams();

  /// Server to listen for dynamix reconfigurations
  dynamic_reconfigure::Server<bumblebob_lateral_controller::DynamicConfig> reconfigure_server_;

  // csv data
  std::ofstream csv_file;

  // ros node handle
  ros::NodeHandle node_;

  // node frequency
  ros::Rate loop_rate_;

  // defines if the debug mode is on or off
  bool debug_mode_;

  // defines if the data is written to the csv file
  bool write_csv_;

  // the velocity of the vehicle
  double speed_;

  // the heading of the vehicle
  double heading_;

  // the angular velocity of the vehicle
  double angular_velocity_;

  // the lateral gain parameter
  double k_lat_;

  // the heading gain parameter
  double k_head_;

  // the change of steering gain parameter
  double k_o_;

  // maximum possible steering of the vehicle
  float max_steering_;

  // the steering command
  float steering_cmd_;

  // the position of the vehicle
  cv::Point2d position_;

  // the path given as an array of waypoints
  std::vector<cv::Point2d> waypoints_;

  // restriction for the x_velocity
  double highest_gains_speed_;

  // restriction for the y_velocity
  double highest_angular_velocity_;

  // the path to the csv file
  std::string filepath_;

  // init the struct that includes the euler angles
  EulerAngles euler_angles_;

  // the value that defines the number of waypoints to the front
  int look_front_;

  // the value that defines the number of waypoints to the back
  int look_back_;

  // Publishers
  ros::Publisher pub_steering_;

  // Subscribers
  ros::Subscriber sub_waypoints_;
  ros::Subscriber sub_pose_;

  // topic reader
  std::string read_pub_steering_;
  std::string read_sub_states_;
  std::string read_sub_raceline_;
};

#endif
