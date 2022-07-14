#include "../include/lateral_kinematic_ROS_node.hpp"

LateralROSNode::LateralROSNode() : loop_rate_(20), speed_(0), heading_(0), angular_velocity_(0), highest_gains_speed_(0)
{
}

void LateralROSNode::waypointsCallback(const bumblebob_msgs::PointArray::ConstPtr& point_array)
{
  std::vector<cv::Point2d> points;
  int counter = 0;
  for (std::vector<geometry_msgs::Point>::const_iterator iterator = point_array->position.begin();
       iterator != point_array->position.end(); iterator++)
  {
    points.push_back(cv::Point2d(-iterator->y, iterator->x));
  }
  waypoints_ = points;
}

void LateralROSNode::dynamicReconfigureCallback(bumblebob_lateral_controller::DynamicConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %f %d %d", config.lateral_gain, config.heading_gain, config.delta_dot_gain,
           config.look_front, config.look_back);

  k_lat_ = config.lateral_gain;
  k_head_ = config.heading_gain;
  k_o_ = config.delta_dot_gain;

  look_back_ = config.look_back;
  look_front_ = config.look_front;
}

void LateralROSNode::statesCallback(const gazebo_msgs::ModelStates::ConstPtr& states)
{
  // find index of byssa
  int index = 0;
  std::vector<std::string>::const_iterator it = std::find(states->name.begin(), states->name.end(), "byssa");
  if (it != states->name.end())
  {
    index = it - states->name.begin();
  }
  else
  {
    ROS_INFO("Couldnt find Element named byssa in gazebo");
  }

  // position
  double pos_x = states->pose[index].position.x;
  double pos_y = states->pose[index].position.y;

  double cog_diff = 0.766;
  double rotatedX = -std::sin(heading_) * cog_diff;
  double rotatedY = std::cos(heading_) * cog_diff;

  position_ = cv::Point2d(-pos_y, pos_x);

  // heading
  double head_x = states->pose[index].orientation.x;
  double head_y = states->pose[index].orientation.y;
  double head_z = states->pose[index].orientation.z;
  double head_w = states->pose[index].orientation.w;

  // EulerAngles conversion
  // roll (x axis)
  double sinr_cosp = 2 * (head_w * head_x + head_y * head_z);
  double cosr_cosp = 1 - 2 * (head_x * head_x + head_y * head_y);
  euler_angles_.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y axis)
  double sinp = 2 * (head_w * head_y - head_z * head_x);
  if (std::abs(sinp) >= 1)
    euler_angles_.pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    euler_angles_.pitch = std::asin(sinp);

  // yaw (z axis)
  double siny_cosp = 2 * (head_w * head_z + head_x * head_y);
  double cosy_cosp = 1 - 2 * (head_y * head_y + head_z * head_z);
  euler_angles_.yaw = std::atan2(siny_cosp, cosy_cosp);

  heading_ = -euler_angles_.yaw;
  ;

  // velocity
  speed_ = states->twist[index].linear.x;

  // angular velocity
  angular_velocity_ = states->twist[index].angular.z;
}

void LateralROSNode::waitForWaypoints()
{
  ROS_INFO("Ready and waiting for Waypoints");
  while (waypoints_.size() < 3)
  {
    ros::spinOnce();
    continue;
  }
  ROS_INFO("Waypoints received...Lets GO");
}

void LateralROSNode::writeDataToCSV(cv::Point2d position, double car_heading, double road_heading,
                                    double feedback_angle, double feedforward_angle, double steering_cmd, double radius,
                                    double lateral_error, double heading_error, double angular_velocity)
{
  csv_file << position.x << "," << position.y << "," << car_heading << "," << road_heading << "," << feedback_angle
           << "," << feedforward_angle << "," << steering_cmd << "," << radius << "," << lateral_error << ","
           << heading_error << "," << angular_velocity << std::endl;
}

void LateralROSNode::printDebugData(FeedForward* ff, Feedback* fb)
{
  ROS_DEBUG_STREAM("Look Back: " << look_back_);
  ROS_DEBUG_STREAM("Look Front: " << look_front_);

  ROS_DEBUG_STREAM("Lateral Gain: " << k_lat_);
  ROS_DEBUG_STREAM("Heading Gain: " << k_head_);
  ROS_DEBUG_STREAM("Steering Change Gain: " << k_o_);

  ROS_DEBUG_STREAM("Vehicle Velocity: " << speed_);
  ROS_DEBUG_STREAM("Vehicle Angular Velocity: " << angular_velocity_);
  ROS_DEBUG_STREAM("Vehicle Postion:   x: " << position_.x << " y: " << position_.y);
  ROS_DEBUG_STREAM("Nearest Point:   x: " << ff->nearest_point_.x << " y: " << ff->nearest_point_.y);
  ROS_DEBUG_STREAM("Circle Center:   x: " << ff->center_.x << " y: " << ff->center_.y);
  ROS_DEBUG_STREAM("Curvature: " << 1 / ff->radius_);
  ROS_DEBUG_STREAM("Circle Direction: " << ff->circle_direction_);

  ROS_DEBUG_STREAM("Lateral Error: " << fb->lateral_error_);
  ROS_DEBUG_STREAM("Road Heading: " << fb->road_heading_);
  ROS_DEBUG_STREAM("Car Heading: " << heading_);
  ROS_DEBUG_STREAM("Heading Error: " << fb->heading_error_);

  ROS_DEBUG_STREAM("FeedForward Angle: " << ff->feedforward_angle_);
  ROS_DEBUG_STREAM("Feedback Angle: " << fb->feedback_angle_);
  ROS_DEBUG_STREAM("Steering Command: " << steering_cmd_);
}

void LateralROSNode::readParams()
{
  // read gains
  ros::param::get("/gains/lateral", k_lat_);
  ros::param::get("/gains/heading", k_head_);
  ros::param::get("/gains/delta_dot", k_o_);

  // read estimation area
  ros::param::get("/estimation_area/look_front", look_front_);
  ros::param::get("/estimation_area/look_back", look_back_);

  // read pubs and subs
  ros::param::get("/publisher/steering", read_pub_steering_);
  ros::param::get("/subscriber/states", read_sub_states_);
  ros::param::get("/subscriber/raceline", read_sub_raceline_);

  // read inits
  ros::param::get("/inits/debug_mode", debug_mode_);
  ros::param::get("/inits/write_csv", write_csv_);
  int rate;
  ros::param::get("/inits/loop_rate", rate);
  loop_rate_ = rate;

  ros::param::get("/car/tire/max_steering", max_steering_);

  // restrictions
  ros::param::get("/restrictions/highest_speed", highest_gains_speed_);

  // csv data
  ros::param::get("file/path", filepath_);
}

void LateralROSNode::initialize()
{
  readParams();

  // publishers
  pub_steering_ = node_.advertise<std_msgs::Float32>(read_pub_steering_, 1);

  // subscriptions
  sub_waypoints_ = node_.subscribe(read_sub_raceline_, 1, &LateralROSNode::waypointsCallback, this);
  sub_pose_ = node_.subscribe(read_sub_states_, 1, &LateralROSNode::statesCallback, this);

  if (write_csv_)
  {
    csv_file.open(filepath_);
  }

  dynamic_reconfigure::Server<bumblebob_lateral_controller::DynamicConfig>::CallbackType dynamic_reconfigure_callback;
  dynamic_reconfigure_callback = boost::bind(&LateralROSNode::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_.setCallback(dynamic_reconfigure_callback);

  ROS_INFO("Lateral ROS Node initialized");
}

void LateralROSNode::updateLoop()
{
  // wait_for_waypoints();
  waitForWaypoints();

  // Begin the main loop
  ROS_INFO("Starting Controller");
  while (ros::ok())
  {
    FeedForward feedforward(position_, speed_, waypoints_, heading_);

    feedforward.setLookBack(look_back_);
    feedforward.setLookFront(look_front_);
    feedforward.runIteration();

    double rounded_speed = round(speed_);
    if (rounded_speed <= 0)
      rounded_speed = 1;

    if (rounded_speed > highest_gains_speed_)
      rounded_speed = highest_gains_speed_;

    Feedback feedback(feedforward, position_, heading_, angular_velocity_, waypoints_, k_lat_, k_head_, k_o_);

    steering_cmd_ = (feedforward.getFeedForwardAngle() + feedback.getFeedbackAngle()) * 180 / M_PI;

    // normalize angle
    if (steering_cmd_ > max_steering_)
    {
      steering_cmd_ = max_steering_;
    }
    else if (steering_cmd_ < -max_steering_)
    {
      steering_cmd_ = -max_steering_;
    }
    // Construct new steering msg and publish to the simulation
    std_msgs::Float32 msg;
    msg.data = steering_cmd_;
    pub_steering_.publish(msg);

    if (debug_mode_)
    {
      printDebugData(&feedforward, &feedback);
    }

    if (write_csv_)
    {
      writeDataToCSV(position_, heading_, feedback.road_heading_, feedback.feedback_angle_,
                     feedforward.feedforward_angle_, steering_cmd_, feedforward.radius_, feedback.lateral_error_,
                     feedback.heading_error_, angular_velocity_);
    }

    ros::spinOnce();
    loop_rate_.sleep();
  }
}
