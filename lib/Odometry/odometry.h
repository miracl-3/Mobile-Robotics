#include <Arduino.h>

#ifndef ODOMETRY
#define ODOMETRY


// Structure containing Encoder's Odometry
struct Odometry {
  float x;
  float y;
  float theta;
  float linear_velocity;   // m/s
  float angular_velocity;  // rad/s
};

Odometry updateOdometry(float delta_t);

#endif