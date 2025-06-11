### Story

In an era where artificial intelligence (AI) and robotics are revolutionizing daily life, building an AI-powered autonomous room rover is a thrilling project for tech enthusiasts. I’ve always been curious about creating such a robot from scratch, using only the limited resources at my disposal. My vision is to design A2R3, a mobile robot that navigates rooms autonomously, avoids obstacles, responds to voice commands, monitors its environment, and even expresses emotions. The possibilities are endless—limited only by the AI "brain" we give it!

A2R3 is an open-source, cost-effective, DIY mobile robotic platform designed for anyone eager to customize and dive into robotics with a hands-on approach. This project guides you through building your own intelligent robot using affordable, readily available components and open-source software. For simplicity, I’ve broken the project into three parts. You can find the complete project description on my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">Hackster </a> page.

<p align="center">
    <img  src="https://github.com/user-attachments/assets/bbc6c5a3-ab26-46e3-b967-9705cc1c35a4">
</p>

### Roadmap

The basic functionality of the Robot is completed.Any firmware in the A2R3 directory works and tested carefully.ROS2 integration with visualization (Rviz) has been integrated and you can find it in the ROS2_playgrounds directory. Step by step documentation is underconstruction and your pateince is vital as I am working alone in this project and writing documentation is no fun task. Micro-ROS integration is next up and I am excited to make any SBCs market obselete in this kind of project. A contribution and sponsorship of any kind is most welcome! 

### Firmwares

    git clone https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3.git

### Softwares and Dev environments:
> [!TIP]
> Recommended Operating System: Linux or windows Arduino compatibe IDE installed ( I recommend working on Linux (Ubuntu) for future navigation and SLAM development systems.
> <a href="https://docs.ros.org/en/foxy/index.html" target="_blank"> ROS2 </a> or <a href="https://micro.ros.org/" target="_blank"> Micro ROS </a> for <a href="https://docs.nav2.org/" target="_blank"> Nav2 </a>development although the robot drives by itself with built-in firmware obstacle avoidance alogorithim I wrote through its slave microcontoller <a href="https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/index.html" target="_blank"> ESP32. </a>
> Programming Language: C/C++ 

### Hardware and CAD design - Main drive:
> [!TIP]
> Plastic foam tires are great for this project and they are lightweight, hard and have a good grip. Relatively less costy.
> The actuator to drive the rover is plastic agearbox RS390. It drives with a 6V DC motor inside (they are upto 10v tolerant) and produce a fair amount of speed with a good torque strong enough to drive the rover with up to 15kg amount of payload. In the future I will do some other scientific tests on this...Anyways; I have the first build (firmware and hardware) that you can find the BOM in my hackster page link down below.

### Here are useful and important links for your guidance:      
> [!IMPORTANT]                                                                                                                                                   
    <a href="https://www.youtube.com/watch?v=NbiJPMn4Qm0" target="_blank">PID control build Demo Video</a> <br/>
    <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">Documentation</a> page.

    
This open source project has been officially certified as open source hardware by the Open Source Hardware Association under the project listing: https://certification.oshwa.org/fi000003.html [^2].
software and hardware Licenses: [^1]

[^1]: ![oshw_facts](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)
[^2]: ![Screenshot from 2025-03-30 15-58-53](https://github.com/user-attachments/assets/599f7daa-bc83-4e1d-ba89-f66ca4a2cc97)



