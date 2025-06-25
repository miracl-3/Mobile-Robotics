#include <Arduino.h>

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Left Motor Control Pins
#define IN1 26
#define IN2 33
#define ENA 27

// Right Motor Control Pins
#define IN3 25
#define IN4 32
#define ENB 14

// Right Encoder Pins
#define enR1_A 23 
#define enR2_B 22 

// Right Encoder Pins
#define enL1_A 18 
#define enL2_B 19 

// Define PWM's stats
#define freq 5000
#define reso 8
#define channel_right 0 // ENB
#define channel_left 1  // ENA

// Robot parameters
#define gear_ratio 46.8
#define pulse_per_revo 11
#define wheel_distance  0.211
#define wheel_radius  0.0325

// Initialize pulses counter
extern volatile long rightPulses;
extern volatile long leftPulses;
extern const int pulsesPerRevolution;

// Initialize motor pins and encoder interrupts
void motorSetUp();

// Movement commands with PWM speed input (0–255)
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop();

// Interrupt Service Routines for encoder tick counting
void IRAM_ATTR handleRightEncoder();
void IRAM_ATTR handleLeftEncoder();

// Utility: Convert pulses to RPM
float pulsesToRPM(int pulses);

// Utility: Estimate required pulses to reach distance in meters
int DistancetoPulse(float distance);

// Set motor speed function
void setMotorSpeed(float left_wheel_velocity, float right_wheel_velocity);

#endif