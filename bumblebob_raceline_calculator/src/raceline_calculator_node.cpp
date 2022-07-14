#include "../include/raceline_calculator_node.hpp"
#include "../include/raceline_optimizer.hpp"
#include <fstream>

RacelineCalculatorNode::RacelineCalculatorNode() : loop_rate_(20)
{
}

void RacelineCalculatorNode::referenceCallback(const bumblebob_msgs::PointArray& msg)
{
  std::vector<cv::Point2d> reference_line;
  for (geometry_msgs::Point point : msg.position)
  {
    reference_line.push_back(cv::Point2d(point.x, point.y));
  }
  reference_line_ = reference_line;
}

/* Convert incoming Cone coordinates from ConeArray message to input vector for delaunator and fill coneArray. */
void RacelineCalculatorNode::mapCallback(const bumblebob_msgs::ConeArray& msg)
{
  std::vector<bumblebob_msgs::Cone> cone_array = {};
  std::vector<cv::Point2d> cone_positions = {};
  std::vector<cv::Point2d> restriction_blue = {};
  std::vector<cv::Point2d> restriction_yell = {};

  for (bumblebob_msgs::Cone cone : msg.cones)
  {
    cone_array.push_back(cone);

    cone_positions.push_back(cv::Point2d(cone.position.x, cone.position.y));

    if (cone.type == 1)
    {
      restriction_yell.push_back(cv::Point2d(cone.position.x, cone.position.y));
    }
    else if (cone.type == 2)
    {
      restriction_blue.push_back(cv::Point2d(cone.position.x, cone.position.y));
    }
  }

  cone_array_ = cone_array;
  cone_positions_ = cone_positions;

  restriction_blue.erase(unique(restriction_blue.begin(), restriction_blue.end()), restriction_blue.end());
  restriction_yell.erase(unique(restriction_yell.begin(), restriction_yell.end()), restriction_yell.end());

  restriction_blue_ = restriction_blue;
  test_ = restriction_blue;
  restriction_yell_ = restriction_yell;
}

void RacelineCalculatorNode::readParams()
{
  ros::param::get("/raceline_optimization/general/write_csv", write_csv_);
  ros::param::get("/raceline_optimization/general/file_path", file_path_);

  ros::param::get("/raceline_optimization/publishers/pub_raceline", read_pub_raceline_);
  ros::param::get("/raceline_optimization/publishers/pub_velocity_profile", read_pub_velocity_profile_);

  ros::param::get("/raceline_optimization/subscribers/sub_map", read_sub_map_);
  ros::param::get("/raceline_optimization/subscribers/sub_reference", read_sub_reference_);
}

void RacelineCalculatorNode::initialize()
{
  readParams();

  // publishers
  pub_raceline_ = node_.advertise<bumblebob_msgs::PointArray>(read_pub_raceline_, 1);
  pub_velocity_profile_ = node_.advertise<std_msgs::Float64MultiArray>(read_pub_velocity_profile_, 1);

  // subscriptions
  sub_map_ = node_.subscribe(read_sub_map_, 1, &RacelineCalculatorNode::mapCallback, this);
  sub_reference_ = node_.subscribe(read_sub_reference_, 1, &RacelineCalculatorNode::referenceCallback, this);

  ROS_INFO("Raceline Calculator");
}

void RacelineCalculatorNode::waitForReference()
{
  ROS_INFO("Ready and waiting for Subscribers");
  while (reference_line_.size() < 3 || cone_array_.size() < 3)
  {
    ros::spinOnce();
    continue;
  }
  ROS_INFO("Subscriber Data Received ... Lets Go");
}

void RacelineCalculatorNode::writeDataToCSV()
{
  csv_file_.open(file_path_);
  if (csv_file_.is_open())
  {
    for (int i = 0; i < waypoints_.size(); i++)
    {
      csv_file_ << i << "," << velocity_[i] << "," << curvature_[i] << "," << acceleration_[i] << ","
                << initial_velocity_[i % initial_velocity_.size()] << "," << initial_curv_[i % initial_curv_.size()]
                << "," << restriction_blue_[i % restriction_blue_.size()].x << ","
                << restriction_blue_[i % restriction_blue_.size()].y << ","
                << restriction_yell_[i % restriction_yell_.size()].x << ","
                << restriction_yell_[i % restriction_yell_.size()].y << "," << raceline_current_[i * 35].x << ","
                << raceline_current_[i * 35].y << "," << reference_line_[i % reference_line_.size()].x << ","
                << reference_line_[i % reference_line_.size()].y << std::endl;
    }
  }
  else
  {
    ROS_WARN("Data File Could Not Be Opened");
  }
}

void RacelineCalculatorNode::doubleTheWaypoints()
{
  waypoints_ = reference_line_;
  for (int i = 0; i < reference_line_.size() - 1; i++)
  {
    cv::Point2d new_point = (reference_line_[i] + reference_line_[i + 1]) / 2;
    waypoints_.insert(waypoints_.begin() + 2 * i + 1, new_point);
  }
}

void RacelineCalculatorNode::buildRacelineMsg()
{
  // build raceline msg

  for (int i = 0; i < raceline_current_.size(); i++)
  {
    geometry_msgs::Point point;
    point.x = raceline_current_[i].x;
    point.y = raceline_current_[i].y;
    point.z = 0.0;

    pointArrayRaceLine_.position.push_back(point);
  }
}

void RacelineCalculatorNode::buildVelocityProfileMsg()
{
  for (int i = 0; i < velocity_.size(); i++)
  {
    velocity_array_.data.push_back(velocity_[i]);
  }
}

void RacelineCalculatorNode::updateLoop()
{
  waitForReference();
  // Begin the main loop
  ROS_INFO("Starting Raceline Optimizer");

  doubleTheWaypoints();

  std::cout << "waypoints size: " << waypoints_.size() << std::endl;

  // Calculate The initial values
  VelocityProfile initial_vel_prof(waypoints_);
  initial_velocity_ = initial_vel_prof.getVelocityProfile();
  initial_curv_ = initial_vel_prof.getCurvatureProfile();

  // optimize raceline
  RacelineOptimizer ro(initial_vel_prof, restriction_blue_, restriction_yell_, waypoints_);
  raceline_current_ = ro.getCurrentRaceline();

  velocity_ = ro.getCurrentVelocityProfile().getVelocityProfile();
  curvature_ = ro.getCurrentVelocityProfile().getCurvatureProfile();
  acceleration_ = ro.getCurrentVelocityProfile().getAccelRestriction();
  ROS_INFO("Raceline Optimizer: Raceline Was Optimized");

  buildRacelineMsg();
  buildVelocityProfileMsg();
  if (write_csv_)
  {
    writeDataToCSV();
  }

  while (ros::ok())
  {
    ros::spinOnce();

    pub_raceline_.publish(pointArrayRaceLine_);
    pub_velocity_profile_.publish(velocity_array_);

    loop_rate_.sleep();
  }
}
