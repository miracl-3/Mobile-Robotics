#include <Arduino.h>
#include "motor_control.h"
#include <math.h>

volatile long rightPulses = 0;
volatile long leftPulses = 0;

const int pulsesPerRevolution = 2000;
const int ticksperRevolution = GEAR_RATIO * PULSES_PER_REV;  // 46.8 × 11

// PID control values (not currently used here, but can be applied later)
float kp = 10;
float ki = 5;
float kd = 1;
float proportional_error;
float integral_error; 
float derivative_error;
float total_error;
int base_pwm = 190;

int left_pwm, right_pwm;
const float wheel_circumference = 2 * PI * WHEEL_RADIUS;

const int balance_factor = 15;  // For turning adjustment

// ========================
// Initialization
// ========================
void motorSetUp() {
    // Motor control pins
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    // Encoder pins
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

    // PWM setup
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_RIGHT_EN, PWM_CHANNEL_RIGHT);

    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_LEFT_EN, PWM_CHANNEL_LEFT);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), handleRightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), handleLeftEncoder, RISING);
}

// ========================
// Motor Motion
// ========================
void moveForward(int speed) { 
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void moveBackward(int speed) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnLeft(int speed) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, speed - balance_factor);

    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnRight(int speed) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, speed);

    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, speed - balance_factor);
}

void stop() {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, 0);

    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

// ========================
// Encoder Interrupts
// ========================
void IRAM_ATTR handleRightEncoder() {
    bool A = digitalRead(ENCODER_RIGHT_A);
    bool B = digitalRead(ENCODER_RIGHT_B);
    if (A == B) {
        rightPulses++;
    } else {
        rightPulses--;
    }
}

void IRAM_ATTR handleLeftEncoder() {
    bool A = digitalRead(ENCODER_LEFT_A);
    bool B = digitalRead(ENCODER_LEFT_B);
    if (A == B) {
        leftPulses--;
    } else {
        leftPulses++;
    }
}

// ========================
// Utility Functions
// ========================
float pulsesToRPM(int pulsesCount) {
    float revolutions = (float)pulsesCount / ticksperRevolution;
    float rpm = revolutions * 60.0;
    return rpm;
}

int DistancetoPulse(float distance) {
    return (int)(GEAR_RATIO * PULSES_PER_REV * distance / wheel_circumference);
}

// To be implemented
void setMotorSpeed(float left_wheel_velocity, float right_wheel_velocity) {
    // Left as placeholder for future velocity-based control
}
