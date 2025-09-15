<p align="center">


![Screencastfrom2025-09-1521-18-30-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/846c617b-8a6f-4b1c-bb5e-ec2b91144fe9)

</p>

### Introduction

A2R3 is a modular open-source mobile robot built on the ESP32.
Think of it as a hacker’s playground for robotics — Wi-Fi, Bluetooth, LiDAR, IMU, odometry, OLED + buzzer feedback, and full autonomy potential.

A2R3 is an open-source, cost-effective, DIY mobile robotic platform designed for anyone eager to customize and dive into robotics with a hands-on approach. This project guides you through building your own intelligent robot using affordable, readily available components and open-source software. For simplicity, I’ve broken the project into three parts. You can find the complete project description on my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">Hackster </a> page.

### Features
- [✓] ESP32 Core — Wi-Fi + Bluetooth ready
- [✓] Modular Sensor System — I²C bus for expansions
- [✓] Obstacle Avoidance — VL53L0X laser distance
- [✓] Motion Control — TB6612 / TMC2209 drivers
- [✓] Odometry — AS5600 encoders
- [✓] Feedback — SSD1306 OLED + buzzer
- [✓] Cooling — PWM fan control
- [-] SLAM + ROS2 integration (coming soon...)
- [-] AI-driven behaviors (future module)


### Roadmap

The core functionality of the A2R3 robot is complete, with all firmware in the A2R3 directory thoroughly tested and fully operational. ROS2 integration, including visualization with RViz, is implemented and available in the ROS2_playgrounds directory. Step-by-step documentation is currently in progress, and I appreciate your patience as I tackle this solo project—writing documentation isn’t exactly thrilling! Next up is Micro-ROS integration, which I’m excited about, as it aims to make single-board computers (SBCs) obsolete for projects like this. Contributions or sponsorships of any kind are greatly appreciated to help drive this project forward!
 Core drive + obstacle avoidance
 
 - [✓] Autonomous mission logic
 - [✓] Remote control via PS3 pad
 - [-] ROS2 + SLAM integration
 - [-] Web Dashboard + Telemetry (check out the beta version 1.1.0 <a href="https://a2r3.42web.io/"> here</a>)


### Hardware

- Brain: ESP32-WROOM / ESP32-S3 (slave micro-controller), SLAM : Orange Pi 3B (cheaper) Raspbery Pi or any other SBC.
- Muscle: TB6612 / TMC2209 motor drivers
- Senses: MPU6050 (IMU), VL53L0X (ToF), AS5600 (encoders)
- Face: SSD1306 OLED + status LEDs
- Core Power: 20V Li-ion → 5V / 3.3V regulation for a full BOM go <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">here </a>

### Firmware

    git clone https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3.git
    cd AI-Autonomous-Room-Rover-Robot-A2R3

### Softwares and Dev environments:
> [!NOTE]
> - Recommended Operating System: Linux (Ubuntu preferred for navigation and SLAM development) or Windows with Arduino-compatible IDE.
>- Software: <a href="https://docs.ros.org/en/foxy/index.html" target="_blank"> ROS2 </a> or <a href="https://micro.ros.org/" target="_blank">Micro-ROS </a> for <a href="https://docs.nav2.org/" target="_blank"> Nav2 </a>development.
>- The robot autonomously navigates using a custom obstacle-avoidance algorithm on its <a href="https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/index.html" target="_blank">ESP32 </a>slave microcontroller.
>- Programming Language: C++/Python

### Hardware and CAD design - Main drive:
> [!TIP]
> Plastic foam tires are ideal for the A2R3 rover, offering a lightweight, durable design with excellent grip at a relatively low cost. The rover is powered by a plastic gearbox RS390 actuator, driven by a 6V DC motor (tolerant up to 10V), delivering impressive speed and sufficient torque to handle payloads of up to 15kg. In the future, I plan to conduct additional scientific tests to optimize performance. For now, the first build, including firmware and hardware, is complete, and you can find the Bill of Materials (BOM) on my Hackster page linked below.

### Here are useful and important links for your guidance:      
> [!IMPORTANT]                                                                                                                                                   
    <a href="https://www.youtube.com/watch?v=NbiJPMn4Qm0" target="_blank">PID control build Demo Video</a> <br/>
    <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">Documentation</a> page.

    
This open source project has been officially certified as open source hardware by the Open Source Hardware Association under the project <a href="https://certification.oshwa.org/fi000003.html">listing </a>: [^2].
software and hardware Licenses: [^1]

[^1]: ![oshw_facts](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)
[^2]: ![Screenshot from 2025-03-30 15-58-53](https://github.com/user-attachments/assets/599f7daa-bc83-4e1d-ba89-f66ca4a2cc97)



