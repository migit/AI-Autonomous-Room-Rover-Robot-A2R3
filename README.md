Story

In an era where artificial intelligence (AI) and robotics are transforming our daily lives, creating an AI-powered autonomous room rover is an exciting project for any tech enthusiast. I have always wondered how it feels like to build this idea from the ground up alone with limited resources i have available. This rover bot is designed to navigate a room autonomously, avoiding obstacles and responding to voice commands , express emotions and so much more... the sky is the limit.

Imagine having a companion rover bot who actually knows you by name and serves you tirelessly (well while 🔋 is full :) ). I am designing A2R3 as an open source cost effective diy robotic platform for all who want to customize their own rover bot.

This project will guide you through the process of building your own intelligent rover using readily available components and open-source software step by step. I divided this project in three parts for simplicity. You can also find the whole project decscription in my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">hackster</a> page.


![Screenshot from 2025-03-30 13-17-48](https://github.com/user-attachments/assets/4ee1b69b-6227-4713-8048-b6f405b76334)

![Screenshot from 2025-03-30 13-21-11](https://github.com/user-attachments/assets/72a08aba-d7cd-4382-9b98-51fbddf1370f)

![Screenshot from 2025-03-30 13-24-28](https://github.com/user-attachments/assets/899983e6-f91d-4695-83cb-cb75ad7cc514)

Objectives

    Autonomous Navigation: Enable the rover to move around a room without human intervention, avoiding obstacles.
    Voice Control: Incorporate voice recognition to control the rover’s movements.![Screenshot from 2025-03-30 13-17-48](https://github.com/user-attachments/assets/8a4b4a74-97f4-43c7-b711-db44b8575d1e)

    Real-Time Feedback: Implement sensors for real-time data collection and response.

Software Requirements

    Operating System: Linux or windows Arduino IDE installed ( I recommend working on Linux (Ubuntu) for future navigation and SLAM development system to use 
    ROS2 or micro ROS) However the bot drives by itself with built-in obstacle avoidance alogorithim through its slave microcontoller that is ESP32. 
    Programming Language: C++
Obstacle avoidance system demo

[![Watch the video](https://img.youtube.com/vi/NbiJPMn4Qm0/hqdefault.jpg)](https://youtu.be/NbiJPMn4Qm0?si=QGYEltaEz_qwInCY&t=107)

Main drive Hardware Requirements

    Plastic foam tires are great for this project and they are lightweight, hard and have a good grip. Relatively less costy.
    The actuator to drive the rover is plastic agearbox RS390. It drives with a 6V DC motor inside and produce a fair amount of speed with a good torque strong enough to drive the rover with up to 15kg amount of payload. In the future I will do some other scientific tests on this...
    
![hardware and parts](https://github.com/user-attachments/assets/13497e88-f66b-438f-93a4-5240f2e5cabc)

<a href="https://youtu.be/83nP3b_AAgo" target="_blank">Body Build demo</a>

Demonstration                                                                                                                                                        
    <a href="https://youtu.be/E3wDgulsSTU?si=qFQs4_kfr9r9EPV2" target="_blank">Obstacle avoidance system demo</a> <br/>
    <a href="https://www.youtube.com/watch?v=NbiJPMn4Qm0" target="_blank">PID control build Demo</a> <br/>

![oshw_facts](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)
This open source project has been officially certified as open source hardware by the Open Source Hardware Association
under the project listing: https://certification.oshwa.org/fi000003.html
![certification-mark-FI000003-wide](https://github.com/user-attachments/assets/9b97acd2-7789-40f4-9523-363fbe388f1e)
