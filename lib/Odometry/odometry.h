#include <Arduino.h>

#ifndef ODOMETRY
#define ODOMETRY

// Structure containing Encoder's Odometry
struct Odometry {
  float x;
  float y;
  float theta;
};

Odometry encoder_odometry(float left_wheel_velocity, float right_wheel_velocity, float delta_t);

#endif