Story

In an era where artificial intelligence (AI) and robotics are transforming our daily lives, creating an AI-powered autonomous room rover is an exciting project for any tech enthusiast. I have always wondered how it feels like to build this idea from the ground up alone with limited resources I have available. I have designed tgis mobile robot to navigate a room autonomously, avoiding obstacles and responding to voice commands , monitoring any activity in the environment, express emotions and so much more... The sky is the limit! Well in this case the AI brain is the limit.

I am designing A2R3 as an open source cost effective diy mobile robotic platform for anybody whishes to customize and learn robotics with a "do it yourself in mind".

This project will guide you through the process of building your own intelligent mobile robot using readily available components, very low cost modules, and open-source softwares.
I have divided this project in three parts for simplicity. You can also find the whole project decscription in my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">hackster</a> page.

<p align="center">
    <img  src="https://github.com/user-attachments/assets/bbc6c5a3-ab26-46e3-b967-9705cc1c35a4">
</p>

Firmware

    git clone https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3.git

Software Dev environments:
> [!TIP]
> Recommended Operating System: Linux or windows Arduino compatibe IDE installed ( I recommend working on Linux (Ubuntu) for future navigation and SLAM development systems.
> <a href="https://docs.ros.org/en/foxy/index.html" target="_blank"> ROS2 </a> or <a href="https://micro.ros.org/" target="_blank"> Micro ROS </a> for <a href="https://docs.nav2.org/" target="_blank"> Nav2 </a>development although the robot drives by itself with built-in firmware obstacle avoidance alogorithim I wrote through its slave microcontoller <a href="https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/index.html" target="_blank"> ESP32. </a>
> Programming Language: C/C++ 

Hardware Requirements - Main drive:
> [!TIP]
> Plastic foam tires are great for this project and they are lightweight, hard and have a good grip. Relatively less costy.
> The actuator to drive the rover is plastic agearbox RS390. It drives with a 6V DC motor inside (they are upto 10v tolerant) and produce a fair amount of speed with a good torque strong enough to drive the rover with up to 15kg amount of payload. In the future I will do some other scientific tests on this...Anyways; I have the first build (firmware and hardware) that you can find the BOM in my hackster page link down below.

Here are useful and important links for your guidance:      
> [!IMPORTANT]                                                                                                                                                   
    <a href="https://www.youtube.com/watch?v=NbiJPMn4Qm0" target="_blank">PID control build Demo</a> <br/>
    <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">hackster</a> page.

    
This open source project has been officially certified as open source hardware by the Open Source Hardware Association under the project listing: https://certification.oshwa.org/fi000003.html [^2].
software and hardware Licenses: [^1]

[^1]: ![oshw_facts](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)
[^2]: ![Screenshot from 2025-03-30 15-58-53](https://github.com/user-attachments/assets/599f7daa-bc83-4e1d-ba89-f66ca4a2cc97)



