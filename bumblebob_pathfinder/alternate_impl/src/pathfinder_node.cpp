#include "../include/pathfinder_node.hpp"

PathFinderNode::PathFinderNode() : loop_rate_(20), steering_cmd_(0), debug_mode_(true)
{
}

/* Convert incoming Cone coordinates from ConeArray message to input vector for delaunator and fill coneArray. */
void PathFinderNode::conesCallback(const bumblebob_msgs::ConeArray::ConstPtr& msg)
{
  bumblebob_msgs::ConeArray arr = *msg;
  std::vector<bumblebob_msgs::Cone> coneArray = {};
  std::vector<cv::Point2d> conePositions = {};
  std::vector<cv::Point2d> yellow_cones = {};
  std::vector<cv::Point2d> blue_cones = {};

  for (bumblebob_msgs::Cone cone : arr.cones)
  {
    coneArray.push_back(cone);

    conePositions.push_back(cv::Point2d(cone.position.x, cone.position.y));

    if (cone.type == 1)
    {
      yellow_cones.push_back(cv::Point2d(cone.position.x, cone.position.y));
    }
    else if (cone.type == 2)
    {
      blue_cones.push_back(cv::Point2d(cone.position.x, cone.position.y));
    }
  }

  coneArray_ = coneArray;
  conePositions_ = conePositions;

  blue_cones.erase(unique(blue_cones.begin(), blue_cones.end()), blue_cones.end());
  yellow_cones.erase(unique(yellow_cones.begin(), yellow_cones.end()), yellow_cones.end());

  blue_cones_ = blue_cones;
  yellow_cones_ = yellow_cones;
}

void PathFinderNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& states)
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
    ROS_INFO("Name Byssa Not Found In Gazebo");
  }
  // position
  vehicle_position_ = { states->pose[index].position.x, states->pose[index].position.y };
  vx_ = states->twist[index].linear.x;
}

void PathFinderNode::steeringCallback(const std_msgs::Float32& cmd)
{
  steering_cmd_ = cmd.data / 180 * M_PI;
}

void PathFinderNode::readROSParams()
{
  ROS_INFO("Reading ROS Params");
  // Publishers
  ros::param::get("/pathfinder/publishers/pathfinder_path", pathfinder_path_);
  ros::param::get("/pathfinder/publishers/visualization_markers", visualization_markers_);
  ros::param::get("/pathfinder/publishers/raceline_topic", raceline_topic_);
  // Subscriber
  ros::param::get("/pathfinder/subscribers/cones_sub", cones_sub_);
  ros::param::get("/pathfinder/subscribers/states_sub", states_sub_);
  ros::param::get("/pathfinder/subscribers/steer_sub", steer_sub_);
  // Others
  ros::param::get("/pathfinder/params/threshold", threshold_);
  ros::param::get("/pathfinder/params/ppm", ppm_);
  ros::param::get("/pathfinder/params/density", density_);
  ros::param::get("/pathfinder/inits/debug_mode", debug_mode_);

  ROS_INFO("ROS Params Read");
}

void PathFinderNode::waitForConeMessages()
{
  ROS_INFO("Ready And Waiting For Cone Messages");
  while (coneArray_.size() < 1)
  {
    ros::spinOnce();
    continue;
  }
  ROS_INFO("Cones Received! Let's GO");
}

void PathFinderNode::visualizeMeshPoints(std::vector<cv::Point2d> point_array)
{
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = "/fssim/vehicle/camera";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "spheres";
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.id = 0;

  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  sphere_list.scale.x = 0.1;
  sphere_list.scale.y = 0.1;
  sphere_list.scale.z = 0.1;

  // Points are green
  sphere_list.color.r = 1.0f;
  sphere_list.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int i = 0; i < point_array.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = point_array[i].x;
    p.y = point_array[i].y;
    p.z = -1;

    sphere_list.points.push_back(p);
  }
  mesh_pub_.publish(sphere_list);
}

void PathFinderNode::visualizeRaceLine(std::vector<cv::Point2d> raceline)
{
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = "/fssim/vehicle/camera";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "points";
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;

  sphere_list.id = 0;

  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  sphere_list.scale.x = 0.1;
  sphere_list.scale.y = 0.1;
  sphere_list.scale.z = 0.1;

  // Points are green
  sphere_list.color.r = 1.0f;
  sphere_list.color.a = 1.0;

  // Create the vertices for the points and lines
  for (int i = 0; i < raceline.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = raceline[i].x;
    p.y = raceline[i].y;
    p.z = -1;

    sphere_list.points.push_back(p);
  }
  path_pub_.publish(sphere_list);
}

void PathFinderNode::publishRaceline()
{
  bumblebob_msgs::PointArray raceline;
  for (int i = 0; i < raceline_.size(); i++)
  {
    geometry_msgs::Point point;
    point.x = raceline_[i].x;
    point.y = raceline_[i].y;
    point.z = 0;

    raceline.position.push_back(point);
  }
  raceline_pub_.publish(raceline);
}

void PathFinderNode::initialize()
{
  readROSParams();
  // Subscribers
  cones_subscriber_ = node_.subscribe(cones_sub_, 1000, &PathFinderNode::conesCallback, this);
  model_states_subscriber_ = node_.subscribe(states_sub_, 1000, &PathFinderNode::modelStatesCallback, this);

  // Publishers
  mesh_pub_ = node_.advertise<visualization_msgs::Marker>(visualization_markers_, 1000);
  path_pub_ = node_.advertise<visualization_msgs::Marker>(pathfinder_path_, 1000);
  raceline_pub_ = node_.advertise<bumblebob_msgs::PointArray>(raceline_topic_, 1000);
  pub_steering_ = node_.advertise<std_msgs::Float32>("/bumblebob/steering", 1);
  ROS_INFO("Pathfinder Node Initialized");
}

void PathFinderNode::debug()
{
  std::cout << "--------------------DEBUG MODE----------------------" << std::endl;
  std::cout << "Steering Cmd: " << steering_cmd_ << std::endl;
  std::cout << "Velocity: " << vx_ << std::endl;
  std::cout << "Vehicle Postion:   x: " << vehicle_position_.x << " y: " << vehicle_position_.y << std::endl;
  std::cout << "Array Sizes: "
            << "Conearray: " << coneArray_.size() << "  Raceline: " << raceline_.size()
            << "  Meshline: " << mesh_points_.size() << std::endl;
}

void PathFinderNode::updateLoop()
{
  // wait for Cone Messages
  waitForConeMessages();

  // Begin the main loop
  ROS_INFO("Starting Pathfinder");
  while (ros::ok())
  {
    Mesh mesh(conePositions_, blue_cones_, yellow_cones_, density_, ppm_, threshold_);
    mesh.calculateMesh();
    mesh_points_ = mesh.getMeshPoints();

    // visualize the points

    PathFinder pf(conePositions_, blue_cones_, yellow_cones_, 100);
    std::vector<cv::Point2d> middlelane = pf.middleLane();
    // LaneAdaption la(middlelane, previous_waypoints_, cv::Point2d(0, 0), vx_, steering_cmd_);
    // middlelane = la.adaptLane();
    visualizeMeshPoints(mesh_points_);
    visualizeRaceLine(middlelane);
    raceline_ = middlelane;
    previous_waypoints_ = middlelane;
    if (debug_mode_)
    {
      debug();
    }
    publishRaceline();

    double distance = 2.5;
    // find point on raceline with the distance
    int index = 0;
    double smallest_dist = 100000;
    for (int i = 0; i < raceline_.size(); i++)
    {
      double distancetest = sqrt(pow(raceline_[i].x - 0, 2) + pow(raceline_[i].y - 0, 2));
      if (distancetest - distance < smallest_dist)
      {
        smallest_dist = distance;
        index = i;
      }
    }

    std::cout << "Punkt:  x: " << raceline_[index].x << "  y: " << raceline_[index].y << std::endl;

    double alpha = -atan(raceline_[index].y / raceline_[index].x);
    std::cout << "Alpha: " << alpha << std::endl;
    double delta = atan((2 * 1.53 * sin(alpha)) / distance) * 180 / M_PI;

    std::cout << "Steering Command: " << delta << std::endl;

    std_msgs::Float32 msg;
    msg.data = delta;
    pub_steering_.publish(msg);

    ros::spinOnce();
    loop_rate_.sleep();
  }
}