#include "Visualizer.hpp"

// generate RVIZ data from pathfinder output

namespace bumblebob_pathfinder
{
Visualizer::Visualizer(ros::NodeHandle& nh)
{
  ros::param::get("/pathfinder/topics/pub/cone_mesh_marker", cone_mesh_topic);
  ros::param::get("/pathfinder/topics/pub/path_mesh_marker", path_mesh_topic);
  ros::param::get("/pathfinder/topics/pub/best_path_marker", best_path_topic);
  ros::param::get("/pathfinder/links/base_link", base_link_name);

  cone_mesh_pub = nh.advertise<visualization_msgs::Marker>(cone_mesh_topic, 10);
  path_mesh_pub = nh.advertise<visualization_msgs::Marker>(path_mesh_topic, 10);
  best_path_pub = nh.advertise<visualization_msgs::Marker>(best_path_topic, 10);

  points.header.frame_id = base_link_name;
  target.header.frame_id = base_link_name;
  cone_undefined.header.frame_id = base_link_name;
  cone_blue.header.frame_id = base_link_name;
  cone_yellow.header.frame_id = base_link_name;
  cone_orange.header.frame_id = base_link_name;
  line_list_conemesh.header.frame_id = base_link_name;
  line_list_pathmesh.header.frame_id = base_link_name;
  line_list_path.header.frame_id = base_link_name;

  ros::Time now = ros::Time::now();
  points.header.stamp = now;
  target.header.stamp = now;
  cone_undefined.header.stamp = now;
  cone_blue.header.stamp = now;
  cone_yellow.header.stamp = now;
  cone_orange.header.stamp = now;
  line_list_conemesh.header.stamp = now;
  line_list_pathmesh.header.stamp = now;
  line_list_path.header.stamp = now;

  points.ns = "points";
  target.ns = "target";
  cone_undefined.ns = "cones";
  cone_blue.ns = "cones";
  cone_yellow.ns = "cones";
  cone_orange.ns = "cones";
  line_list_conemesh.ns = "lines";
  line_list_pathmesh.ns = "lines";
  line_list_path.ns = "lines";

  points.action = visualization_msgs::Marker::ADD;
  target.action = visualization_msgs::Marker::ADD;
  cone_undefined.action = visualization_msgs::Marker::ADD;
  cone_blue.action = visualization_msgs::Marker::ADD;
  cone_yellow.action = visualization_msgs::Marker::ADD;
  cone_orange.action = visualization_msgs::Marker::ADD;
  line_list_conemesh.action = visualization_msgs::Marker::ADD;
  line_list_pathmesh.action = visualization_msgs::Marker::ADD;
  line_list_path.action = visualization_msgs::Marker::ADD;

  points.pose.orientation.w = 1.0;
  target.pose.orientation.w = 1.0;
  cone_undefined.pose.orientation.w = 1.0;
  cone_blue.pose.orientation.w = 1.0;
  cone_yellow.pose.orientation.w = 1.0;
  cone_orange.pose.orientation.w = 1.0;
  line_list_conemesh.pose.orientation.w = 1.0;
  line_list_pathmesh.pose.orientation.w = 1.0;
  line_list_path.pose.orientation.w = 1.0;

  points.id = 0;
  target.id = 1;
  cone_undefined.id = 2;
  cone_blue.id = 3;
  cone_yellow.id = 4;
  cone_orange.id = 5;
  line_list_conemesh.id = 6;
  line_list_pathmesh.id = 7;
  line_list_path.id = 8;

  points.type = visualization_msgs::Marker::POINTS;
  target.type = visualization_msgs::Marker::POINTS;
  cone_undefined.type = visualization_msgs::Marker::POINTS;
  cone_blue.type = visualization_msgs::Marker::POINTS;
  cone_yellow.type = visualization_msgs::Marker::POINTS;
  cone_orange.type = visualization_msgs::Marker::POINTS;
  line_list_conemesh.type = visualization_msgs::Marker::LINE_LIST;
  line_list_pathmesh.type = visualization_msgs::Marker::LINE_LIST;
  line_list_path.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = points.scale.y = 0.2;
  target.scale.x = target.scale.y = 0.5;
  cone_undefined.scale.x = cone_undefined.scale.y = 0.3;
  cone_blue.scale.x = cone_blue.scale.y = 0.3;
  cone_yellow.scale.x = cone_yellow.scale.y = 0.3;
  cone_orange.scale.x = cone_orange.scale.y = 0.3;

  // LINE_STRIP and LINE_LIST markers use only the x component of scale, for the line width
  line_list_conemesh.scale.x = 0.04;
  line_list_pathmesh.scale.x = 0.04;
  line_list_path.scale.x = 0.2;

  // Possible path points are green
  points.color.g = 0.7;
  points.color.b = 1.0;
  points.color.a = 1.0;

  // target point is magenta
  target.color.r = 1.0;
  target.color.b = 0.8;
  target.color.a = 1.0;

  // Undefined cones are light grey
  cone_undefined.color.r = 0.8;
  cone_undefined.color.g = 0.8;
  cone_undefined.color.b = 0.8;
  cone_undefined.color.a = 1.0;

  // Blue cones are blue
  cone_blue.color.b = 1.0;
  cone_blue.color.a = 1.0;

  // Yellow cones are yellow
  cone_yellow.color.r = 1.0;
  cone_yellow.color.g = 1.0;
  cone_yellow.color.a = 1.0;

  // Orange cones are orange
  cone_orange.color.r = 1.0;
  cone_orange.color.g = 0.5;
  cone_orange.color.b = 0.0;
  cone_orange.color.a = 1.0;

  // cone mesh is red
  line_list_conemesh.color.r = 1.0;
  line_list_conemesh.color.g = 0.1;
  line_list_conemesh.color.a = 1.0;

  // path mesh is cyan
  line_list_pathmesh.color.g = 0.7;
  line_list_pathmesh.color.b = 1.0;
  line_list_pathmesh.color.a = 1.0;

  // best path is green
  line_list_path.color.g = 1.0;
  line_list_path.color.a = 1.0;
}

void Visualizer::visualize_coneMesh(delaunator::Delaunator& d, std::vector<bumblebob_msgs::Cone>& coneArray)
{
  line_list_conemesh.points.clear();
  // Create the vertices for the points and lines
  for (std::size_t i = 0; i < d.triangles.size(); i += 3)
  {
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;

    p1.x = d.coords[2 * d.triangles[i]];
    p1.y = d.coords[2 * d.triangles[i] + 1];

    p2.x = d.coords[2 * d.triangles[i + 1]];
    p2.y = d.coords[2 * d.triangles[i + 1] + 1];

    p3.x = d.coords[2 * d.triangles[i + 2]];
    p3.y = d.coords[2 * d.triangles[i + 2] + 1];

    p1.z = 0;
    p2.z = 0;
    p3.z = 0;

    // The line list needs two points for each line
    line_list_conemesh.points.push_back(p1);
    line_list_conemesh.points.push_back(p2);
    line_list_conemesh.points.push_back(p2);
    line_list_conemesh.points.push_back(p3);
    line_list_conemesh.points.push_back(p3);
    line_list_conemesh.points.push_back(p1);
  }

  cone_undefined.points.clear();
  cone_blue.points.clear();
  cone_yellow.points.clear();
  cone_orange.points.clear();
  // Publish cones with according color
  for (bumblebob_msgs::Cone c : coneArray)
  {
    geometry_msgs::Point pt;

    pt.x = c.position.x;
    pt.y = c.position.y;
    pt.z = 0;

    switch (c.type)
    {
      case coneType::undefined:
        cone_undefined.points.push_back(pt);
        break;
      case coneType::blue:
        cone_blue.points.push_back(pt);
        break;
      case coneType::yellow:
        cone_yellow.points.push_back(pt);
        break;
      case coneType::ORANGE:
        cone_orange.points.push_back(pt);
        break;
    }
  }

  cone_mesh_pub.publish(line_list_conemesh);
  cone_mesh_pub.publish(cone_undefined);
  cone_mesh_pub.publish(cone_blue);
  cone_mesh_pub.publish(cone_yellow);
  cone_mesh_pub.publish(cone_orange);
}

void Visualizer::visualize_pathMesh(delaunator::Delaunator& pathMesh)
{
  points.points.clear();
  line_list_pathmesh.points.clear();
  // Create the vertices for the points and lines
  for (std::size_t i = 0; i < pathMesh.triangles.size(); i += 3)
  {
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;

    p1.x = pathMesh.coords[2 * pathMesh.triangles[i]];          // tx0
    p1.y = pathMesh.coords[2 * pathMesh.triangles[i] + 1];      // ty0
    p2.x = pathMesh.coords[2 * pathMesh.triangles[i + 1]];      // tx1
    p2.y = pathMesh.coords[2 * pathMesh.triangles[i + 1] + 1];  // ty1
    p3.x = pathMesh.coords[2 * pathMesh.triangles[i + 2]];      // tx2
    p3.y = pathMesh.coords[2 * pathMesh.triangles[i + 2] + 1];  // ty2
    p1.z = 0;
    p2.z = 0;
    p3.z = 0;

    points.points.push_back(p1);
    points.points.push_back(p2);
    points.points.push_back(p3);

    // The line list needs two points for each line
    line_list_pathmesh.points.push_back(p1);
    line_list_pathmesh.points.push_back(p2);
    line_list_pathmesh.points.push_back(p2);
    line_list_pathmesh.points.push_back(p3);
    line_list_pathmesh.points.push_back(p3);
    line_list_pathmesh.points.push_back(p1);
  }

  path_mesh_pub.publish(points);
  path_mesh_pub.publish(line_list_pathmesh);
}

void Visualizer::visualize_bestPath_and_target(Path& p, geometry_msgs::Point t)
{
  line_list_path.points.clear();
  target.points.clear();

  for (auto it = p.nodes.begin(); it + 1 != p.nodes.end(); ++it)
  {
    geometry_msgs::Point pt1;
    geometry_msgs::Point pt2;

    pt1.x = (*it)[0];
    pt1.y = (*it)[1];
    pt1.z = 0;

    pt2.x = (*(it + 1))[0];
    pt2.y = (*(it + 1))[1];
    pt2.z = 0;

    line_list_path.points.push_back(pt1);
    line_list_path.points.push_back(pt2);
  }

  target.points.push_back(t);

  best_path_pub.publish(line_list_path);
  best_path_pub.publish(target);
}

}  // namespace bumblebob_pathfinder
