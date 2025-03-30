/*
 * Project: A2R3 - AI Autonomous Room Rover Robot A2R3
 * Description:    AI mobile robotic base for any other robotic platforms to build upon. With these basic functionalities and creating a learning experience for mobile robotic development.
 * Author: Michael Seyoum (https://www.hackster.io/mikroller)
 * Created: March 30, 2025
 * License: GPL-3.0-only - See LICENSE file for details
 * Repository: https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3/
 */

#include <Wire.h>

#define AS5600_ADDR 0x36           // I2C address for AS5600
#define TCA9548A_ADDR 0x70         // I2C address for TCA9548A
#define WHEEL_RADIUS 3.0           // Wheel radius in cm (adjust this based on your robot's wheel)
#define ENCODER_RESOLUTION 4096.0  // AS5600 12-bit resolution (4096 steps per revolution)
#define PI 3.14159

// Function to select the channel on the TCA9548A
void tca9548aSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Function to read the raw angle from AS5600 (no filters)
uint16_t readAS5600Angle() {
  uint16_t angle = 0;
  
  // Start communication with AS5600
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);  // High byte of angle register
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: I2C communication failed!");
    return 0; // Return 0 if communication fails
  }
  
  // Request 2 bytes from AS5600 (high and low angle bytes)
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    angle = Wire.read() << 8; // High byte
    angle |= Wire.read();     // Low byte
  } else {
    Serial.println("Error: Failed to read from AS5600!");
    return 0; // Return 0 if reading fails
  }
  
  return angle;
}

// Function to read the encoder on the selected channel (no filters)
float readEncoder(uint8_t channel) {
  tca9548aSelect(channel);  // Select the correct channel on the multiplexer
  uint16_t angleRaw = readAS5600Angle();
  return angleRaw * (360.0 / ENCODER_RESOLUTION);  // Convert raw value to degrees
}

// Function to calculate distance traveled by the wheel in cm based on the angle
float calculateWheelDistance(float angleDegrees) {
  float wheelCircumference = 2 * PI * WHEEL_RADIUS;  // Circumference = 2 * pi * radius
  return (angleDegrees / 360.0) * wheelCircumference; // Distance = (angle / 360) * circumference
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C communication
  
  // Optional: Check if AS5600 is connected properly for both channels
  tca9548aSelect(0);  // Select left encoder
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("AS5600 not detected on channel 0. Check wiring!");
    while (1); // Stop the program if AS5600 is not found on channel 0
  }

  tca9548aSelect(1);  // Select right encoder
  Wire.beginTransmission(AS5600_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("AS5600 not detected on channel 1. Check wiring!");
    while (1); // Stop the program if AS5600 is not found on channel 1
  }
}

void loop() {
  // Read angle from left encoder on channel 0
  float leftAngle = readEncoder(0);
  float leftDistance = calculateWheelDistance(leftAngle);  // Distance traveled by left wheel
  
  // Read angle from right encoder on channel 1
  float rightAngle = readEncoder(1);
  float rightDistance = calculateWheelDistance(rightAngle);  // Distance traveled by right wheel

  // Print both angles and distances in a format suitable for the Serial Plotter
  Serial.print("Left Wheel: Angle (deg) = ");
  Serial.print(leftAngle);
  Serial.print(", Distance (cm) = ");
  Serial.print(leftDistance);
  Serial.print(" | Right Wheel: Angle (deg) = ");
  Serial.print(rightAngle);
  Serial.print(", Distance (cm) = ");
  Serial.println(rightDistance);

  // Optional: Add a small delay to avoid flooding the Serial Monitor/Plotter
  delay(100);
}
