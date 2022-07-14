/**
 *  @file PurePursuitNode.hpp
 *
 *  @author Dominik Prossel
 *  @version 1.0
 */

// Theory:
// https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081

#include "bumblebob_pure_pursuit/PurePursuitNode.hpp"
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace bumblebob_pure_pursuit
{
PurePursuitNode::PurePursuitNode(ros::NodeHandle& node_handle) : node_handle_(node_handle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // path_sub_ = node_handle_.subscribe(path_topic_, 1, &PurePursuitNode::pathCallback, this);
  // target_point_pub_ = node_handle.advertise<geometry_msgs::PointStamped>(target_point_topic_, 1);
  target_point_sub_ = node_handle_.subscribe(target_point_topic_, 1, &PurePursuitNode::targetCallback, this);
  steering_pub_ = node_handle.advertise<std_msgs::Float32>(steering_topic_, 1);
};

void PurePursuitNode::targetCallback(const geometry_msgs::PointStamped& point)
{
  target_ = point.point;

  double dist = norm(target_.x, target_.y);
  double sin_alpha = target_.y / dist;
  double radius = dist / (2 * sin_alpha);

  // left: negative, right: positive
  steering_angle_ = std::atan(2 * 1.53 * sin_alpha / dist) * 180 / M_PI;

  std_msgs::Float32 pub_msg;
  pub_msg.data = steering_angle_;
  steering_pub_.publish(pub_msg);
}

double PurePursuitNode::norm(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

void PurePursuitNode::pathCallback(const bumblebob_msgs::PointArray& msg)
{
  if (msg.position.empty() || msg.position.size() < 4)
  {
    return;
  }

  auto target_it =
      std::upper_bound(msg.position.begin(), msg.position.end(), target_dist_,
                       [](double value, const auto& point) { return value < PurePursuitNode::norm(point.x, point.y); });
  if (target_it == msg.position.end())
  {
    --target_it;
  }

  double avg_dist = std::numeric_limits<double>::infinity();
  int count = 0;
  while (avg_dist > dist_thresh_ && target_it > msg.position.begin() + count)
  {
    target_ = *(target_it - count);

    double dist = norm(target_.x, target_.y);
    double sin_alpha = target_.y / dist;
    double radius = dist / (2 * sin_alpha);

    avg_dist = 0.0;
    for (auto p = msg.position.begin(); p < target_it; ++p)
    {
      avg_dist += std::abs(norm(p->x, p->y - radius) - std::abs(radius));
    }
    avg_dist /= msg.position.size();

    // left: negative, right: positive
    steering_angle_ = std::atan(2 * 1.53 * sin_alpha / dist) * 180 / M_PI;
    ++count;
  }

  std_msgs::Float32 pub_msg;
  pub_msg.data = steering_angle_;
  steering_pub_.publish(pub_msg);
  // publish target point
  geometry_msgs::PointStamped target_point;
  target_point.header = msg.header;
  target_point.header.stamp = ros::Time::now();
  target_point.point.x = target_.x;
  target_point.point.y = target_.y;
  target_point.point.z = 0.0;
  target_point_pub_.publish(target_point);
}

void PurePursuitNode::run()
{
  ros::spin();
};

bool PurePursuitNode::readParameters()
{
  return node_handle_.getParam("path_topic", path_topic_) && node_handle_.getParam("steering_topic", steering_topic_) &&
         node_handle_.getParam("target_distance", target_dist_) &&
         node_handle_.getParam("distance_threshold", dist_thresh_) &&
         node_handle_.getParam("target_point_topic", target_point_topic_);
};

}  // namespace bumblebob_pure_pursuit

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PurePursuitNode");
  ros::NodeHandle node_handle("~");
  bumblebob_pure_pursuit::PurePursuitNode pure_pursuit_node(node_handle);
  pure_pursuit_node.run();

  return 0;
}