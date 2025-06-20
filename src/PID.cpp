// #include <math.h>
// #include <Arduino.h>

// // Pin definitions for motors
// const int enL = 7;    // Left motor PWM pin
// const int inL1 = 8;   // Left motor direction pin 1
// const int inL2 = 9;   // Left motor direction pin 2
// const int enR = 12;   // Right motor PWM pin
// const int inR1 = 10;  // Right motor direction pin 1
// const int inR2 = 11;  // Right motor direction pin 2

// // Encoder pins
// int enLA = 2;   // Left encoder A channel
// int enLB = 3;   // Left encoder B channel
// int enRA = 18;  // Right encoder A channel
// int enRB = 19;  // Right encoder B channel

// volatile int leftEnCount = 0;
// volatile int rightEnCount = 0;
// const int ticksPerRevolution = 350;  // Ticks per full revolution of the motor

// unsigned long previousMillis = 0;
// const long interval = 100;  // Interval for RPM calculation (100 milliseconds)

// // Robot parameters
// const float WHEEL_DISTANCE = 0.185;  // Distance between wheels in meters
// const float WHEEL_RADIUS = 0.0215;   // Corrected wheel radius (in meters)

// // PID gains for rho (distance) control
// float Kp_rho = 2.75;
// float Ki_rho = 0.0;
// float Kd_rho = 0.0;

// // PID gains for alpha (heading) control
// float Kp_alpha = 28.75;
// float Ki_alpha = 0.1;
// float Kd_alpha = 27.5;

// // Control gains
// const float gamma = 0.3;
// const float lamda = 0.25;
// const float h = -0.3;     // Gain for steering

// // Current position and orientation of the robot
// float x = 0.0, y = 0.0, theta = 0.0;
// float v1, v2;
// // Goal position and orientation
// const float x_goal = 1.0, y_goal = 1.0, theta_goal = 0.0;

// // Time step
// const float dt = 0.1;  // 100ms
// int disabled = 1;

// // PID control variables
// float prevError_rho = 0, integral_rho = 0;
// float prevError_alpha = 0, integral_alpha = 0;

// void setup() {
//   Serial.begin(9600);

//   // Set up encoder interrupts
//   attachInterrupt(digitalPinToInterrupt(enLA), leftEnISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(enLB), leftEnISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(enRA), rightEnISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(enRB), rightEnISR, RISING);

//   // Motor control pin setup
//   pinMode(enR, OUTPUT);
//   pinMode(enL, OUTPUT);
//   pinMode(inR1, OUTPUT);
//   pinMode(inR2, OUTPUT);
//   pinMode(inL1, OUTPUT);
//   pinMode(inL2, OUTPUT);
// }

// void loop() {
//   // Calculate errors for rho (distance) and alpha (heading)

//   float error_rho = sqrt(pow(x_goal - x, 2) + pow(y_goal - y, 2));  // Distance to the goal
//   float error_alpha = atan2(y_goal - y, x_goal - x) - theta;  // Heading error
//   float phi = atan2(y_goal - y, x_goal - x) - theta_goal;


//   // PID control for rho (distance)
//   integral_rho += error_rho ;
//   float derivative_rho = (error_rho - prevError_rho);
//   float output_rho = Kp_rho * error_rho + Ki_rho * integral_rho + Kd_rho * derivative_rho;
//   prevError_rho = error_rho;

//   // PID control for alpha (heading)
//   integral_alpha += error_alpha;
//   float derivative_alpha = (error_alpha - prevError_alpha);
//   float output_alpha = Kp_alpha * error_alpha + Ki_alpha * integral_alpha + Kd_alpha * derivative_alpha;
//   prevError_alpha = error_alpha;

//   // Use the PID outputs to calculate control velocities
//   float v = gamma * output_rho * cos(error_alpha);  // Linear velocity adjusted by distance error
//   float omega = lamda * output_alpha + (gamma * cos(error_alpha) * sin(error_alpha) * (error_alpha + h * phi)) / error_alpha;  // Angular velocity adjusted by heading error

//   // Calculate left and right wheel velocities
//   float v_r = (2 * v + omega * WHEEL_DISTANCE) / 2;
//   float v_l = (2 * v - omega * WHEEL_DISTANCE) / 2;

//   // Apply motor control
//   if (disabled == 1) {
//     setMotorSpeeds(v_l, v_r);
//   }

//   unsigned long currentMillis = millis();

//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;

//     // Measure RPM for both motors
//     float leftRPM = calculateRPM(leftEnCount);
//     float rightRPM = calculateRPM(rightEnCount);

//     // Reset the encoder counts for the next measurement interval
//     leftEnCount = 0;
//     rightEnCount = 0;

//     // Convert RPM to angular velocity (rad/s)
//     float omegaL = (leftRPM * 2.0 * PI) / 60.0;   // Left wheel angular velocity in rad/s
//     float omegaR = (rightRPM * 2.0 * PI) / 60.0;  // Right wheel angular velocity in rad/s

//     // Calculate linear velocities for left and right wheels
//     v1 = omegaL * WHEEL_RADIUS;  // Linear velocity of the left wheel
//     v2 = omegaR * WHEEL_RADIUS;  // Linear velocity of the right wheel
//     v = (v1 + v2) / 2;           // Average velocity
//   }

//   // Simulate movement using the calculated velocities from both wheels
//   float theta_dot = (v2 - v1) / WHEEL_DISTANCE;  // Angular velocity
//   theta += theta_dot * dt;

//   // Update x and y positions based on the current orientation and velocity
//   float v_avg = (v1 + v2) / 2;
//   x += v_avg * cos(theta) * dt;
//   y += v_avg * sin(theta) * dt;

//   // Check if the robot reached the goal
//   if (error_rho < 0.08) {  // Within 5 cm
//     disabled = 0;
//     digitalWrite(inL1, LOW);
//     digitalWrite(inL2, LOW);
//     digitalWrite(inR1, LOW);
//     digitalWrite(inR2, LOW);
//   }

//   delay(20);  // Wait for the next control cycle
//   Serial.println(error_rho);
// }

// // Set motor speeds with PWM values (0-255)
// void setMotorSpeeds(float v_l, float v_r) {
//   int left_pwm = int(255 * abs(v_l) / 1.0);  // Assuming 1 m/s is max speed
//   int right_pwm = int(255 * abs(v_r) / 1.0);  // Assuming 1 m/s is max speed

//   // Constrain the PWM values to ensure they are within limits
//   left_pwm = constrain(left_pwm, 40, 255);  // Increase minimum to 100 for heavy loads
//   right_pwm = constrain(right_pwm, 40, 255);

//   digitalWrite(inL1, LOW);
//   digitalWrite(inL2, HIGH);
//   digitalWrite(inR1, HIGH);
//   digitalWrite(inR2, LOW);

//   // Set the PWM values for the motors
//   analogWrite(enL, left_pwm);
//   analogWrite(enR, right_pwm);
// }

// // Calculate RPM from encoder pulse count
// float calculateRPM(int pulseCount) {
//   float revolutions = (float)pulseCount / ticksPerRevolution;
//   float rpm = revolutions * 60.0;  // Convert to RPM
//   return rpm;
// }

// // Encoder interrupt service routines
// void leftEnISR() {
//   leftEnCount++;
// }

// void rightEnISR() {
//   rightEnCount++;
// }
