#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "icons.h"
#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 34
#define SCL_PIN 35

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define FRAME_DELAY 42
#define FRAME_WIDTH 32
#define FRAME_HEIGHT 32


#define AIN1 13 // ESP32 Pin D13 to TB6612FNG Pin AIN1
#define BIN1 12 // ESP32 Pin D12 to TB6612FNG Pin BIN1
#define AIN2 14 // ESP32 Pin D14 to TB6612FNG Pin AIN2
#define BIN2 27 // ESP32 Pin D27 to TB6612FNG Pin BIN2
#define PWMA 26 // ESP32 Pin D26 to TB6612FNG Pin PWMA
#define PWMB 25 // ESP32 Pin D25 to TB6612FNG Pin PWMB
#define STBY 32 // ESP32 Pin D32 to TB6612FNG Pin STBY
#define BUZZER_PIN 15


#define HIGH_SPEED 255
#define LOW_SPEED 125
#define VLOW_SPEED 100


// Define the distance threshold (in mm)
const int distanceThreshold = 600; // 0.6 meters = 600 mm
const int offsetA = 1;
const int offsetB = 1;

int frame = 0;
int speed = 255;

Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 20000, 8, 3);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 20000, 8, 4);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
VL53L0X sensor;

void BEEPOnce() {
    tone(BUZZER_PIN, 1175, 50);
    noTone(BUZZER_PIN);
}

void BEEPinfo() {
    tone(BUZZER_PIN, 1047, 255);
    tone(BUZZER_PIN, 1175, 255);
    noTone(BUZZER_PIN);
}


void OLEDinfo(String text, int size = 2) {
    display.clearDisplay();
    display.setTextSize(size);
    display.setTextColor(WHITE);
    display.setCursor(20, 15);
    display.println(text);
    display.display();
    delay(FRAME_DELAY);
}

void OLED_Status(String text, int size = 1, int x = 0, int y = 0) {
    display.setTextSize(size);
    display.setTextColor(WHITE);
    display.setCursor(x, y);
    display.println(text);
    display.display();
    delay(FRAME_DELAY);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    sensor.setAddress(0x29);
    
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1)
    {
    }
  }
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(200000); // increase timing budget to 200 ms for more accuracy

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    BEEPinfo();
    speed = VLOW_SPEED;
}

void moveForward(int speed) {
    motorLeft.drive(speed);
    motorRight.drive(speed);
}

void moveRight(int speed) {
    motorLeft.drive(speed);
    motorRight.drive(-speed);
}

void moveLeft(int speed) {
    motorLeft.drive(-speed);
    motorRight.drive(speed);
}

void moveBackward(int speed) {
    motorLeft.drive(-speed);
    motorRight.drive(-speed);
}

void stopMotors() {
    motorLeft.brake();
    motorRight.brake();
}

unsigned int tryCount = 0;
void autonomous(unsigned int distance,int speed) {
  //If an obstacle is detected within a certain range, stop and choose a different direction (reverse).Move forward if the path is clear.
    if (distance > 0 && distance < distanceThreshold) {
    stopMotors();
    BEEPOnce();
    OLED_Status("OBSTACLE");
    delay(1000);
    moveBackward(speed); // Reverse
    delay(1000);
    moveForward(speed); // Move forward again
    tryCount ++;
    if (tryCount>=2) // try two times if the obstacle has moved (Animal or person usually moves away)
    {
      speed = HIGH_SPEED;
      moveRight(speed);
      delay(500);
      tryCount=0;
    
    }
  } else {
    speed = VLOW_SPEED;
    moveForward(speed); // Move forward
  }
  delay (50);
}
unsigned int measurement;

void loop() {
  measurement = sensor.readRangeSingleMillimeters();
  autonomous(measurement,speed);
  OLEDinfo("[ AUTO ]");
}


