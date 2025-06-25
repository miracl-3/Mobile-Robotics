#include <Arduino.h>
#include <Wire.h>
#include <BNO08x.h>
#include <odometry.h>
#include <motor_control.h>

BNO08x myIMU;

#define SDA_PIN 21
#define SCL_PIN 15

void setup() {
  Serial.begin(115200);
  delay(500);  // Let IMU stabilize after power-up

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);  // Allow I2C bus to settle

  Serial.println("Initializing BNO08X IMU (no INT pin)...");

  if (!myIMU.begin(0x4A, Wire)) {
    Serial.println("IMU init failed.");
    while (1);
  }

  Serial.println("IMU initialized successfully!");

  if (!myIMU.enableRotationVector(50)) {
    Serial.println("Failed to enable Rotation Vector.");
  } else {
    Serial.println("Rotation Vector enabled.");
  }
}

void loop() {
  if (myIMU.getSensorEvent()) {
    // Get raw quaternion (optional - keep if needed)
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float accuracy = myIMU.getQuatRadianAccuracy();

    // Convert to degrees
    float yaw   = myIMU.getYaw()   * 180.0 / PI;
    float pitch = myIMU.getPitch() * 180.0 / PI;
    float roll  = myIMU.getRoll()  * 180.0 / PI;

    Serial.printf("Quat -> i: %.4f, j: %.4f, k: %.4f, r: %.4f (±%.4f rad)\n",
      quatI, quatJ, quatK, quatReal, accuracy);

    Serial.printf("Orientation -> Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°\n",
      yaw, pitch, roll);
  }

  delay(500);
}