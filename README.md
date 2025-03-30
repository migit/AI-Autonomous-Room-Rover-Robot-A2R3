Story

In an era where artificial intelligence (AI) and robotics are transforming our daily lives, creating an AI-powered autonomous room rover is an exciting project for any tech enthusiast. I have always wondered how it feels like to build this idea from the ground up alone with limited resources i have available. This rover bot is designed to navigate a room autonomously, avoiding obstacles and responding to voice commands , monitoring any activity in the environment, express emotions and so much more... The sky is the limit!.

I am designing A2R3 as an open source cost effective diy mobile robotic platform for all who want to customize their own bot.

This project will guide you through the process of building your own intelligent mobile robot using readily available components, very low cost modules, and open-source softwares.
I have divided this project in three parts for simplicity. You can also find the whole project decscription in my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">hackster</a> page.

<p align="center">
![output-onlinegiftools-ezgif com-resize](https://github.com/user-attachments/assets/bbc6c5a3-ab26-46e3-b967-9705cc1c35a4)
</p>

Firmware

    git clone https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3.git

Software Dev environments:
> [!TIP]
> I recommend Operating System: Linux or windows Arduino compatibe IDE installed ( I recommend working on Linux (Ubuntu) for future navigation and SLAM development system.
> ROS2 or micro ROS for NAV2 development although the robot drives by itself with built-in obstacle avoidance alogorithim through its ESP32 slave microcontoller.
> Programming Language: C/C++ 

Hardware Requirements - Main drive:
> [!TIP]
> Plastic foam tires are great for this project and they are lightweight, hard and have a good grip. Relatively less costy.
> The actuator to drive the rover is plastic agearbox RS390. It drives with a 6V DC motor inside and produce a fair amount of speed with a good torque strong enough to drive the rover with up to 15kg amount of payload. In the future I will do some other scientific tests on this...Anyways; I have the first build (firmware and hardware) that you can find the BOM in my hackster page link down below.

Here are useful and important links for your guidance:      
> [!IMPORTANT]                                                                                                                                                   
    <a href="https://youtu.be/E3wDgulsSTU?si=qFQs4_kfr9r9EPV2" target="_blank">Obstacle avoidance system demo</a> <br/>
    <a href="https://www.youtube.com/watch?v=NbiJPMn4Qm0" target="_blank">PID control build Demo</a> <br/>
    <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">hackster</a> page.

    
This open source project has been officially certified as open source hardware by the Open Source Hardware Association under the project listing: https://certification.oshwa.org/fi000003.html [^2].
software and hardware License: [^1]

[^1]: ![oshw_facts](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)
[^2]: ![Screenshot from 2025-03-30 15-58-53](https://github.com/user-attachments/assets/599f7daa-bc83-4e1d-ba89-f66ca4a2cc97)



