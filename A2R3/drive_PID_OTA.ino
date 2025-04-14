#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <AS5600.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// WiFi Credentials
const char* ssid = "";
const char* password = "";

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
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define TCA9548A_ADDR 0x70
#define MPU6050_CHANNEL 7

// Parameters
#define HIGH_SPEED 120
#define TURN_SPEED 200
#define PWM_FREQUENCY 18000
#define PWM_RESOLUTION 8
#define DISTANCE_THRESHOLD 500
#define TURN_ANGLE 90
#define YAW_TOLERANCE 2.0
#define SPEED_ADJUSTMENT_FACTOR 5
#define BACKUP_TIME 400
#define WHEEL_DIAMETER 20.0   // cm
#define WHEEL_BASE 40.0       // cm
#define DEBUG_MODE true

// PID Parameters
const float KP = 1.5;
const float KI = 0.05;
const float KD = 0.5;
const float MAX_SPEED_CORRECTION = 40.0;

const int offsetA = -1;
const int offsetB = 1;

// Objects
Motor motorLeft(AIN1, AIN2, PWMA, offsetA, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 3);
Motor motorRight(BIN1, BIN2, PWMB, offsetB, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
VL53L0X sensor;
AS5600 encoderLeft;
AS5600 encoderRight;

// Variables
float prevEncoderLeft = 0.0;
float prevEncoderRight = 0.0;
float distanceTravelledLeft = 0.0;
float distanceTravelledRight = 0.0;
float errorSum = 0.0;
float prevError = 0.0;
float lastLeftSpeed = 0.0;
float lastRightSpeed = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  initializeDisplay();
  setupWiFi();
  setupOTA();
  initializeSensor();
  initializeMotors();
  initializeEncoders();
  displaySystemStatus("Ready to Cruise");
}

void loop() {
  ArduinoOTA.handle(); // Handle OTA updates in the loop
  
  uint16_t distance = readDistanceSensor();
  updateEncoders();
  
  float leftSpeed = HIGH_SPEED;
  float rightSpeed = HIGH_SPEED;

  if (distance < DISTANCE_THRESHOLD && distance != 0) {
    avoidObstacle();
  } else {
    driveStraight(leftSpeed, rightSpeed);
  }

  if (DEBUG_MODE) {
    updateDisplay(distance);
  }

  delay(20);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Connecting to:");
  display.setCursor(0, 10);
  display.print(ssid);
  display.display();
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    display.setCursor(attempts * 6, 20);
    display.print(".");
    display.display();
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("WiFi Connected");
    display.setCursor(0, 10);
    display.print("IP: ");
    display.print(WiFi.localIP());
    display.display();
    delay(2000);
  } else {
    Serial.println("Failed to connect to WiFi");
    displaySystemStatus("WiFi Failed");
  }
}

void setupOTA() {
  ArduinoOTA.setHostname("ESP32-Robot"); // Optional: set a hostname
  // ArduinoOTA.setPassword("admin");    // Optional: set a password
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
    displaySystemStatus("OTA Update Start");
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    displaySystemStatus("OTA Update Done");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("OTA Progress: ");
    display.print((progress / (total / 100)));
    display.print("%");
    display.display();
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    displaySystemStatus("OTA Error");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("OTA Ready");
    display.setCursor(0, 10);
    display.print("IP: ");
    display.print(WiFi.localIP());
    display.display();
    delay(2000);
  }
}

void driveStraight(float &leftSpeed, float &rightSpeed) {
  float leftAngle = readEncoderLeft();
  float rightAngle = readEncoderRight();
  
  float leftCorrection = leftAngle - prevEncoderLeft;
  float rightCorrection = rightAngle - prevEncoderRight;
  
  prevEncoderLeft = leftAngle;
  prevEncoderRight = rightAngle;
  
  distanceTravelledLeft += (leftCorrection / 360.0) * PI * WHEEL_DIAMETER;
  distanceTravelledRight += (rightCorrection / 360.0) * PI * WHEEL_DIAMETER;
  
  float error = distanceTravelledLeft - distanceTravelledRight;
  
  errorSum += error;
  float errorRate = error - prevError;
  prevError = error;
  float correction = KP * error + KI * errorSum + KD * errorRate;
  
  correction = constrain(correction, -MAX_SPEED_CORRECTION, MAX_SPEED_CORRECTION);
  
  float targetLeftSpeed = HIGH_SPEED - correction;
  float targetRightSpeed = HIGH_SPEED + correction;
  
  leftSpeed = 0.8 * lastLeftSpeed + 0.2 * targetLeftSpeed;
  rightSpeed = 0.8 * lastRightSpeed + 0.2 * targetRightSpeed;
  
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;
  
  leftSpeed = constrain(leftSpeed, 0, HIGH_SPEED);
  rightSpeed = constrain(rightSpeed, 0, HIGH_SPEED);
  
  motorLeft.drive((int)leftSpeed;
  motorRight.drive((int)rightSpeed);
}

void avoidObstacle() {
  motorLeft.brake();
  motorRight.brake();
  delay(200);
  
  motorLeft.drive(-HIGH_SPEED);
  motorRight.drive(-HIGH_SPEED);
  delay(BACKUP_TIME);
  
  randomTurn();
  delay(500);
}

void randomTurn() {
  int turnDirection = random(0, 2);
  if (turnDirection == 0) {
    motorLeft.drive(-TURN_SPEED);
    motorRight.drive(TURN_SPEED);
  } else {
    motorLeft.drive(TURN_SPEED);
    motorRight.drive(-TURN_SPEED);
  }
  
  delay(500);
  motorLeft.brake();
  motorRight.brake();
}

void initializeDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display init failed!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void updateDisplay(uint16_t distance) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("D: "); display.print(distance); display.print("mm");
  display.setCursor(0, 10);
  display.print("L: "); display.print(distanceTravelledLeft, 1); display.print("cm");
  display.setCursor(0, 20);
  display.print("R: "); display.print(distanceTravelledRight, 1); display.print("cm");
  display.display();
}

void initializeSensor() {
  if (!sensor.init()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
}

uint16_t readDistanceSensor() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    return 0;
  }
  return distance;
}

void initializeMotors() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  motorLeft.brake();
  motorRight.brake();
}

void initializeEncoders() {
  tca9548aSelect(0);
  if (!encoderLeft.begin()) {
    Serial.println("Left encoder init failed!");
    while (1);
  }
  
  tca9548aSelect(1);
  if (!encoderRight.begin()) {
    Serial.println("Right encoder init failed!");
    while (1);
  }
}

void displaySystemStatus(const char* message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(message);
  display.display();
  delay(2000);
}

float readEncoderLeft() {
  tca9548aSelect(0);
  return encoderLeft.readAngle() / 4096.0 * 360.0;
}

float readEncoderRight() {
  tca9548aSelect(1);
  return encoderRight.readAngle() / 4096.0 * 360.0;
}

void updateEncoders() {
  // Empty function maintained for structure
}

void tca9548aSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}
