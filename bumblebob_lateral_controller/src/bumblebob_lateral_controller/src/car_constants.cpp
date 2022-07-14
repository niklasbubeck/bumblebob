#include <car_constants.hpp>

// Total mass of the 2020 Byssa
ros::param::get("/lat_kin_controller/car/inertia/m", CarConstants::mass_);

// Yaw inertia of the 2020 Byssa
ros::param::get("/lat_kin_controller/car/inertia/I_z", CarConstants::yaw_inertia_);

// Length from CG (Center of Gravity) to the front wheels of the 2020 Byssa
ros::param::get("/lat_kin_controller/car/kinematics/b_f", CarConstants::front_length_);

// Length from CG to the rear wheels of the 2020 Byssa
ros::param::get("/lat_kin_controller/car/kinematics/b_r", CarConstants::rear_length_);

// Gravity in m/s
const float CarConstants::gravity_ = 9.81f;

// Front weight of the vehicle in N.
const float CarConstants::front_weight_ = (CarConstants::mass_ * CarConstants::rear_length_ * CarConstants::gravity_) /
                                          (CarConstants::front_length_ + CarConstants::rear_length_);

// Rear weight of the vehicle in N.
const float CarConstants::rear_weight_ = (CarConstants::mass_ * CarConstants::front_length_ * CarConstants::gravity_) /
                                         (CarConstants::front_length_ + CarConstants::rear_length_);

// Steering Gradient of the vehicle in radians.
const float CarConstants::steering_gradient_ = (CarConstants::front_weight_ / CarConstants::front_stiffness_) -
                                               (CarConstants::rear_weight_ / CarConstants::rear_stiffness_);
