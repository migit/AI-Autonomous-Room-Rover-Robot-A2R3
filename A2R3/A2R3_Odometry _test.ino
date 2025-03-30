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
// Function to select the channel on the TCA9548A
void tca9548aSelect(uint8_t channel) {
if (channel > 7) return;
Wire.beginTransmission(TCA9548A_ADDR);
Wire.write(1 << channel);
Wire.endTransmission();
}
// Function to read the angle from AS5600
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
void setup() {
Serial.begin(115200);
Wire.begin(); // Initialize I2C communication
// Optional: Check if AS5600 is connected properly
Wire.beginTransmission(AS5600_ADDR);
if (Wire.endTransmission() != 0) {
Serial.println("AS5600 not detected. Check wiring!");
while (1); // Stop the program if AS5600 is not found
}
// Select TCA9548A channel 0 where AS5600 is connected
tca9548aSelect(0);
}
void loop() {
// Ensure TCA9548A channel 0 is selected before reading
tca9548aSelect(0);
// Read the raw angle from AS5600
uint16_t angleRaw = readAS5600Angle();
// Convert the raw angle to degrees (12-bit resolution)
float angle = angleRaw * (360.0 / 4096.0);
// Print the angle in degrees to the Serial Monitor
Serial.println(angle);
// Optional: Add a small delay to avoid flooding the Serial Monitor
delay(100);
}
