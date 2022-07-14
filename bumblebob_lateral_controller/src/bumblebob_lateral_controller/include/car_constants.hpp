#ifndef CARCONSTANTS_H
#define CARCONSTANTS_H

#include "ros/ros.h"

/**
 *  Config for the 2020 Byssa
 *
 *	@author Niklas Bubeck
 *	@version 1.0.1
 */
class CarConstants
{
public:
  const static float mass_;
  const static float yaw_inertia_;
  const static float front_length_;
  const static float rear_length_;
  const static float front_stiffness_;
  const static float rear_stiffness_;
  const static float gravity_;
  const static float front_weight_;
  const static float rear_weight_;
  const static float steering_gradient_;
};

#endif