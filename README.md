# A2R3 — Modular Open-Source Mobile Robot
![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=migit.AI-Autonomous-Room-Rover-Robot-A2R3)

[![License: OSHW](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)](#license)
[![Languages: C++ | Python](https://img.shields.io/badge/languages-C%2B%2B%20%7C%20Python-blue.svg)](#tech-stack)
[![GitHub repo size](https://img.shields.io/github/repo-size/migit/AI-Autonomous-Room-Rover-Robot-A2R3)](#)


![A2R3 Robot Demo](https://github.com/user-attachments/assets/846c617b-8a6f-4b1c-bb5e-ec2b91144fe9)

---

## Introduction

**A2R3** is a modular, open-source mobile robot built on the ESP32 platform. It is a hacker’s playground for robotics, featuring Wi-Fi, Bluetooth, LiDAR, IMU, odometry, OLED + buzzer feedback, and full autonomy potential. Designed for enthusiasts, it is cost-effective and fully customizable with open-source software and affordable components.

Full documentation is available on the [Hackster page](https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5).

---

## Features

* ESP32 Core — Wi-Fi + Bluetooth ready
* Modular Sensor System — I²C bus for expansions
* Obstacle Avoidance — VL53L0X laser distance sensor
* Motion Control — TB6612 / TMC2209 drivers
* Odometry — AS5600 encoders
* Feedback — SSD1306 OLED + buzzer
* Cooling — PWM fan control
* SLAM + ROS2 integration (coming soon)
* AI-driven behaviors (future module)

---

## Roadmap

The core firmware in the `A2R3` directory is fully operational. ROS2 integration with RViz visualization is available in `ROS2_playgrounds`. Step-by-step documentation is in progress. Future development includes Micro-ROS integration and advanced remote telemetry.

**Core Drive & Obstacle Avoidance:**

* Autonomous mission logic
* Remote control via PS3 pad
* ROS2 + SLAM integration (planned)
* Web Dashboard + Telemetry (beta v1.1.0 [here](https://a2r3.42web.io/))

---

## Hardware

* **Brain**: ESP32-WROOM / ESP32-S3 (slave microcontroller), SLAM via Orange Pi 3B / Raspberry Pi / other SBC
* **Motor Drivers**: TB6612 / TMC2209
* **Sensors**: MPU6050 (IMU), VL53L0X (ToF), AS5600 (encoders)
* **Display**: SSD1306 OLED + status LEDs
* **Power**: 20V Li-ion → 5V / 3.3V regulated (full BOM on Hackster page)
* **Actuators**: Plastic gearbox RS390, 6V DC motor (up to 10V), payload up to 15kg
* **Wheels**: Plastic foam tires for lightweight and durable grip

---

## Firmware & Software Setup

Clone the repository:

```bash
git clone https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3.git
cd AI-Autonomous-Room-Rover-Robot-A2R3
```

**Recommended Environment:**

* OS: Linux (Ubuntu preferred) or Windows with Arduino-compatible IDE
* Software: [ROS2](https://docs.ros.org/en/foxy/index.html) or [Micro-ROS](https://micro.ros.org/) for Nav2 development
* Languages: C++ / Python
* ESP32 handles obstacle-avoidance logic autonomously

---

## Hardware & CAD Tips

* Use plastic foam tires for lightweight and durable grip.
* The first build includes fully tested firmware and hardware.
* Full BOM is available on the [Hackster page](https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5).

---

## Useful Links

* [PID Control Demo Video](https://www.youtube.com/watch?v=NbiJPMn4Qm0)
* [Full Documentation](https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5)
* [OSHW Certification](https://certification.oshwa.org/fi000003.html)

![OSHWA Certified](https://github.com/user-attachments/assets/599f7daa-bc83-4e1d-ba89-f66ca4a2cc97)

---

## License

This project is certified as **Open Source Hardware (OSHW)** and uses open-source software licenses. Refer to the Hackster page and OSHWA listing for details.

