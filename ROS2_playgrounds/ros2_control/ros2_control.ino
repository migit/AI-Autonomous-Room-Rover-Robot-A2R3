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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <AS5600.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define TCA9548A_ADDR 0x70
#define MPU6050_CHANNEL 7
#define ENCODER_LEFT_CHANNEL 0
#define ENCODER_RIGHT_CHANNEL 1

// Parameters
#define HIGH_SPEED 180
#define PWM_FREQUENCY 18000
#define PWM_RESOLUTION 8
#define WHEEL_DIAMETER 20.0 // Set your wheel diameter here
#define WHEEL_BASE 40.0 
#define ENCODER_RESOLUTION 4096
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
const int offsetA = -1;
const int offsetB = 1;

// Toggle error beeps (0 to disable)
#define ENABLE_ERROR_BEEPS 0
#define SENSOR_READ_INTERVAL 50 // ms
#define OLED_UPDATE_INTERVAL 500 // ms

// Wi-Fi
const char* ssid = "holy spot"; // use your local wifi username and password 
const char* password = "12345678";
WiFiServer server(80);

// Objects
Motor motorLeft(AIN1, AIN2, PWMA, offsetA, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 3);
Motor motorRight(BIN1, BIN2, PWMB, offsetB, STBY, PWM_FREQUENCY, PWM_RESOLUTION, 4);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
VL53L0X sensor;
AS5600 encoderLeft;
AS5600 encoderRight;
Adafruit_MPU6050 mpu;

// Shared data
struct SensorData {
  float leftPos;
  float rightPos;
  uint16_t distance;
  float yaw;
  bool valid;
};
SensorData sensorData = {0.0, 0.0, 0, 0.0, false};
SemaphoreHandle_t sensorMutex;

// Yaw tracking
float yaw = 0.0;
unsigned long lastTime = 0;
unsigned long sensCount = 0; // Track SENS commands

// Buzzer functions
void shortBeep() {
  tone(BUZZER_PIN, 1000, 100);
}

void longBeep() {
  if (ENABLE_ERROR_BEEPS) {
    tone(BUZZER_PIN, 1000, 500);
  }
}

void doubleBeep() {
  tone(BUZZER_PIN, 1000, 100);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  tone(BUZZER_PIN, 1000, 100);
}

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Sensor task (Core 1)
void sensorTask(void *pvParameters) {
  while (1) {
    bool sensorError = false;

    // Read VL53L0X first (prioritize for obstacle avoidance)
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
      Serial.println("VL53L0X timeout");
      distance = 0;
      sensorError = true;
    } else if (distance < 8192 && distance > 0) {
      Serial.println("VL53L0X: " + String(distance));
    } else {
      Serial.println("VL53L0X error: " + String(distance));
      distance = 0;
      sensorError = true;
    }

    // Read left encoder
    tcaSelect(ENCODER_LEFT_CHANNEL);
    float leftPos = 0.0;
    if (encoderLeft.isConnected()) {
      leftPos = encoderLeft.readAngle() / (float)ENCODER_RESOLUTION;
      Serial.println("Left encoder: " + String(leftPos, 3));
    } else {
      Serial.println("Left encoder error");
      sensorError = true;
    }

    // Read right encoder
    tcaSelect(ENCODER_RIGHT_CHANNEL);
    float rightPos = 0.0;
    if (encoderRight.isConnected()) {
      rightPos = encoderRight.readAngle() / (float)ENCODER_RESOLUTION;
      Serial.println("Right encoder: " + String(rightPos, 3));
    } else {
      Serial.println("Right encoder error");
      sensorError = true;
    }

    // Read MPU6050 (skip if delayed)
    tcaSelect(MPU6050_CHANNEL);
    sensors_event_t a, g, temp;
    float gyroZ = 0.0;
    unsigned long startTime = millis();
    if (mpu.getEvent(&a, &g, &temp)) {
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTime) / 1000.0;
      yaw += g.gyro.z * dt;
      gyroZ = g.gyro.z;
      lastTime = currentTime;
      Serial.println("MPU6050 yaw: " + String(yaw, 1));
    } else if (millis() - startTime > 20) { // Skip if > 20ms
      Serial.println("MPU6050 error: skipped");
      sensorError = true;
    }

    // Update shared data
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      sensorData.leftPos = leftPos;
      sensorData.rightPos = rightPos;
      sensorData.distance = distance;
      sensorData.yaw = yaw;
      sensorData.valid = !sensorError;
      xSemaphoreGive(sensorMutex);
    }

    vTaskDelay(SENSOR_READ_INTERVAL / portTICK_PERIOD_MS);
  }
}

// OLED task (Core 1)
void oledTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Dist: " + String(sensorData.distance) + " mm");
      display.setCursor(0, 8);
      display.println("Yaw: " + String(sensorData.yaw));
      display.setCursor(0, 24);
      display.println("IP: " + WiFi.localIP().toString());
      display.display();
      xSemaphoreGive(sensorMutex);
    }
    vTaskDelay(OLED_UPDATE_INTERVAL / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize mutex
  sensorMutex = xSemaphoreCreateMutex();

  // Initialize Wi-Fi
  IPAddress local_IP(192, 168, 82, 100);
  IPAddress gateway(192, 168, 82, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);
  shortBeep();

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 failed");
    longBeep();
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Robot Ready");
  display.setCursor(0, 24);
  display.println("IP: " + WiFi.localIP().toString());
  display.display();
  shortBeep();

  // Initialize VL53L0X
  if (!sensor.init()) {
    Serial.println("VL53L0X failed");
    longBeep();
    while (1);
  }
  sensor.setTimeout(50);
  sensor.setMeasurementTimingBudget(20000); // 20ms budget
  sensor.startContinuous();
  shortBeep();

  // Initialize encoders
  tcaSelect(ENCODER_LEFT_CHANNEL);
  encoderLeft.begin();
  if (!encoderLeft.isConnected()) {
    Serial.println("Left encoder failed");
    longBeep();
    while (1);
  }
  tcaSelect(ENCODER_RIGHT_CHANNEL);
  encoderRight.begin();
  if (!encoderRight.isConnected()) {
    Serial.println("Right encoder failed");
    longBeep();
    while (1);
  }
  shortBeep();

  // Initialize MPU6050
  tcaSelect(MPU6050_CHANNEL);
  if (!mpu.begin()) {
    Serial.println("MPU6050 failed");
    longBeep();
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastTime = millis();
  shortBeep();

  // Start tasks on Core 1
  xTaskCreatePinnedToCore(
    sensorTask, "SensorTask", 4096, NULL, 2, NULL, 1); // Higher priority
  xTaskCreatePinnedToCore(
    oledTask, "OledTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Wi-Fi reconnect
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
    }
    Serial.println("WiFi reconnected");
    Serial.println(WiFi.localIP());
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("WiFi Reconnected");
      display.setCursor(0, 24);
      display.println("IP: " + WiFi.localIP().toString());
      display.display();
      xSemaphoreGive(sensorMutex);
    }
    doubleBeep();
    delay(2000);
  }

  // Handle TCP clients
  WiFiClient client = server.available();
  static WiFiClient activeClient;
  if (client) {
    if (activeClient) {
      activeClient.stop();
      Serial.println("Previous client disconnected");
    }
    activeClient = client;
    activeClient.setTimeout(100);
    Serial.println("Client connected: " + client.remoteIP().toString());
  }

  if (activeClient.connected()) {
    String cmd = "";
    unsigned long timeout = millis() + 500;
    while (activeClient.connected() && millis() < timeout) {
      if (activeClient.available()) {
        unsigned long startTime = millis();
        while (activeClient.available()) {
          char c = activeClient.read();
          if (c == '\n') {
            cmd.trim();
            Serial.println("Command received: [" + cmd + "]");
            if (cmd.startsWith("MOT:")) {
              int comma = cmd.indexOf(',');
              String leftCmd = cmd.substring(4, comma);
              String rightCmd = cmd.substring(comma + 1);
              int leftPwm = leftCmd.toInt();
              int rightPwm = rightCmd.toInt();
              motorLeft.drive(leftPwm);
              motorRight.drive(rightPwm);
              activeClient.println("OK");
              Serial.println("MOT executed: " + leftCmd + "," + rightCmd);
              shortBeep();
            } else if (cmd == "SENS") {
              String response;
              bool sensorValid = false;
              if (xSemaphoreTake(sensorMutex, 20 / portTICK_PERIOD_MS) == pdTRUE) {
                response = String(sensorData.leftPos, 3) + "," +
                          String(sensorData.rightPos, 3) + "," +
                          String(sensorData.distance) + "," +
                          String(sensorData.yaw, 1);
                sensorValid = sensorData.valid;
                xSemaphoreGive(sensorMutex);
              } else {
                response = "0.000,0.000,0,0.0";
                Serial.println("Mutex timeout, sending fallback");
              }
              activeClient.println(response);
              Serial.println("SENS response: " + response + ", time: " + String(millis() - startTime) + " ms");
              activeClient.flush();
              if (sensorValid) {
                sensCount++;
                if (sensCount % 10 == 0) {
                  shortBeep();
                }
              } else {
                longBeep(); // Silent if ENABLE_ERROR_BEEPS is 0
              }
            } else {
              Serial.println("Unknown command: " + cmd);
              activeClient.println("ERROR: Unknown command");
              longBeep();
            }
            cmd = "";
            activeClient.flush();
            break;
          } else {
            cmd += c;
            if (cmd.length() > 50) { // Prevent buffer overrun
              cmd = "";
              Serial.println("Command buffer overrun, cleared");
              break;
            }
          }
        }
      }
    }
    if (!activeClient.connected()) {
      activeClient.stop();
      Serial.println("Client disconnected");
      shortBeep();
    }
  }
}
