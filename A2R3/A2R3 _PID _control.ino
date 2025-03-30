/*
 * Project: A2R3 - AI Autonomous Room Rover Robot A2R3
 * Description:    AI mobile robotic base for any other robotic platforms to build upon. With these basic functionalities and creating a learning experience for mobile robotic development.
 * Author: Michael Seyoum (https://www.hackster.io/mikroller)
 * Created: March 30, 2025
 * License: GPL-3.0-only - See LICENSE file for details
 * Repository: https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3/
 */

#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <AS5600.h>

// Pin Definitions
#define SDA_PIN 21
#define SCL_PIN 22
#define PWMA 13
#define AIN2 14
#define AIN1 26
#define STBY 32
#define BIN1 25
#define BIN2 27
#define PWMB 19
#define BUZZER_PIN 15
#define MULTIPLEXER_ADDRESS 0x70
#define ENCODER_LEFT_CHANNEL 0
#define ENCODER_RIGHT_CHANNEL 1

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Motor & Sensor Settings
#define HIGH_SPEED 100       // Base speed
#define PWM_FREQUENCY 18000
#define PWM_RESOLUTION 8
#define DISTANCE_THRESHOLD 600

// PID Constants
#define KP 1.5
#define KI 0.5
#define KD 0.1
#define MAX_INTEGRAL 50.0
#define MAX_OUTPUT 50.0

// Initialize Motor, Display, Sensors
Motor motorLeft(AIN1, AIN2, PWMA, -1, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 3);
Motor motorRight(BIN1, BIN2, PWMB, 1, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
VL53L0X sensor;
AS5600 encoderLeft;
AS5600 encoderRight;

// PID Control Variables
float setPoint = 0.0;
float output;
float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;

void selectMultiplexerChannel(uint8_t channel) {
    Wire.beginTransmission(MULTIPLEXER_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

int readEncoderLeft() {
    selectMultiplexerChannel(ENCODER_LEFT_CHANNEL);
    return encoderLeft.readAngle();
}

int readEncoderRight() {
    selectMultiplexerChannel(ENCODER_RIGHT_CHANNEL);
    return encoderRight.readAngle();
}

void moveForward(int speedLeft, int speedRight) {
    motorLeft.drive(speedLeft);
    motorRight.drive(speedRight);
}

void stopMotors() {
    motorLeft.brake();
    motorRight.brake();
}

void displaySensorData(int leftAngle, int rightAngle) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("Left Enc: ");
    display.println(leftAngle);
    display.print("Right Enc: ");
    display.println(rightAngle);
    display.display();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!sensor.init()) {
        Serial.println("Failed to initialize VL53L0X sensor!");
        while (1);
    }
    sensor.setTimeout(500);
    sensor.setMeasurementTimingBudget(200000);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        while (1);
    }

    encoderLeft.begin();
    encoderRight.begin();

    lastTime = millis();
}

void loop() {
    unsigned int distance = sensor.readRangeSingleMillimeters();
    if (distance == 0) {
        distance = DISTANCE_THRESHOLD + 1;
    }

    int leftAngle = readEncoderLeft();
    int rightAngle = readEncoderRight();

    // PID Calculation
    float error = leftAngle - rightAngle;  // Difference between left and right encoders
    integral += error * (millis() - lastTime) / 1000.0;

    // Anti-windup: Clamp the integral term
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float deltaTime = (millis() - lastTime) / 1000.0;
    float derivative = (error - lastError) / deltaTime;

    output = KP * error + KI * integral + KD * derivative;
    
    // Clamp output to avoid excessive speed correction
    output = constrain(output, -MAX_OUTPUT, MAX_OUTPUT);

    lastError = error;
    lastTime = millis();

    // Adjust motor speeds based on PID output
    int baseSpeed = HIGH_SPEED;
    int speedLeft = constrain(baseSpeed - output, 0, 255);
    int speedRight = constrain(baseSpeed + output, 0, 255);

    if (distance < DISTANCE_THRESHOLD) {
        stopMotors();
        delay(500);
        // Turn or avoid obstacle here
    } else {
        moveForward(speedLeft, speedRight);
    }

    Serial.print("Left Enc: ");
    Serial.print(leftAngle);
    Serial.print(", Right Enc: ");
    Serial.print(rightAngle);
    Serial.print(", Error: ");
    Serial.print(error);
    Serial.print(", Output: ");
    Serial.println(output);

    displaySensorData(leftAngle, rightAngle);

    delay(100);
}
