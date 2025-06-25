#include <Arduino.h>
#include <motor_control.h>
#include <math.h>

volatile long rightPulses = 0;
volatile long leftPulses = 0;
const int pulsesPerRevolution = 2000;
const int ticksperRevolution = 515; // 46.8 x 11

float kp = 10;
float ki = 5;
float kd = 1;
float proportional_error;
float integral_error; 
float derivative_error;
float total_error;
int base_pwm = 190;
int left_pwm, right_pwm;
const float wheel_circumference = 2 * 3.14 * wheel_radius;

//
const int balance_factor = 15;

void motorSetUp(){
    // Motor pin setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Encoder pin setup
    pinMode(enR1_A, INPUT_PULLUP);
    pinMode(enR2_B, INPUT_PULLUP);
    pinMode(enL1_A, INPUT_PULLUP);
    pinMode(enL2_B, INPUT_PULLUP);

    // PWM set up - Right motor 
    ledcSetup(channel_right, freq, reso);
    ledcAttachPin(ENB, channel_right);
    // PWM set up - Left Motor
    ledcSetup(channel_left, freq, reso);
    ledcAttachPin(ENA, channel_left);

    // Attaching interrupts in each encoder
    attachInterrupt(digitalPinToInterrupt(enR1_A), handleRightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(enL1_A), handleLeftEncoder, RISING);
}


void setMotorSpeed(float left_wheel_velocity, float right_wheel_velocity){



}

void moveForward(int speed){ 
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(channel_left, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(channel_right, speed);
}

void moveBackward(int speed){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(channel_left, speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(channel_right, speed);
}

void turnLeft(int speed){ 
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(channel_left, speed - balance_factor);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(channel_right, speed);
}

void turnRight(int speed){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(channel_left, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(channel_right, speed - balance_factor);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(channel_left, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(channel_right, 0);
}

void handleRightEncoder(){
    bool A = digitalRead(enR1_A);
    bool B = digitalRead(enR2_B);
    if(A == B){
        rightPulses++;
    }else{ 
        rightPulses--;
    }
}

void handleLeftEncoder(){
    bool A = digitalRead(enL1_A);
    bool B = digitalRead(enL2_B);
    if(A == B){
        leftPulses--;
    }else{
        leftPulses++;
    }
}

float pulsesToRPM(int pulsesCount){
    float revolutions = (float)pulsesCount / ticksperRevolution;
    float rpm = revolutions * 60.0;
    return rpm;
}

int DistancetoPulse(float distance){
    // 46.8 : gear ratio
    // 11 : number of pulses generated each rotation
    return (int)(gear_ratio * pulse_per_revo * distance / wheel_circumference);
}

