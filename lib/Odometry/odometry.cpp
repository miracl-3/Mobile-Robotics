#include <Arduino.h>
#include <odometry.h>
#include <motor_control.h>
#include <BNO08x.h>


static Odometry pose = {0.0, 0.0, 0.0};
long prevLeftTicks = 0;
long prevRightTicks = 0;
const float DISTANCE_PER_TICK = (2 * PI * WHEEL_RADIUS) / (GEAR_RATIO * PULSES_PER_REV);
Odometry updateOdometry(float delta_t) {
    long leftTicks = leftPulses;
    long rightTicks = rightPulses;

    long deltaLeft = leftTicks - prevLeftTicks;
    long deltaRight = rightTicks - prevRightTicks;

    float dLeft = deltaLeft * DISTANCE_PER_TICK;
    float dRight = deltaRight * DISTANCE_PER_TICK;

    float dCenter = (dLeft + dRight) / 2.0;
    float dTheta = (dRight - dLeft) / WHEEL_BASE;

    // Updating the pose
    pose.x += dCenter * cos(pose.theta + dTheta / 2.0);
    pose.y += dCenter * sin(pose.theta + dTheta / 2.0);
    pose.theta += dTheta;
    pose.linear_velocity = dCenter / delta_t;
    pose.angular_velocity = dTheta / delta_t;

    // Normalize angle
    if (pose.theta > M_PI) {
        pose.theta -= 2 * M_PI;
    } else if (pose.theta < -M_PI) {
        pose.theta += 2 * M_PI;
    }

    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;
    return pose;
}

