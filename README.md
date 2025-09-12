### Story

In an era where artificial intelligence (AI) and robotics are revolutionizing daily life, building an AI-powered autonomous room rover is a thrilling project for tech enthusiasts. I’ve always been curious about creating such a robot from scratch, using only the limited resources at my disposal. My vision is to design A2R3, a mobile robot that navigates rooms autonomously, avoids obstacles, responds to voice commands, monitors its environment, and even expresses emotions. The possibilities are endless—limited only by the AI "brain" we give it!

A2R3 is an open-source, cost-effective, DIY mobile robotic platform designed for anyone eager to customize and dive into robotics with a hands-on approach. This project guides you through building your own intelligent robot using affordable, readily available components and open-source software. For simplicity, I’ve broken the project into three parts. You can find the complete project description on my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">Hackster </a> page.

<p align="center">
    


<img width="929" height="669" alt="Screenshot from 2025-09-12 21-47-57" src="https://github.com/user-attachments/assets/36c78a3c-8048-45e1-9b0c-1c5b9d2ad642" />

</p>
### Roadmap

The core functionality of the A2R3 robot is complete, with all firmware in the A2R3 directory thoroughly tested and fully operational. ROS2 integration, including visualization with RViz, is implemented and available in the ROS2_playgrounds directory. Step-by-step documentation is currently in progress, and I appreciate your patience as I tackle this solo project—writing documentation isn’t exactly thrilling! Next up is Micro-ROS integration, which I’m excited about, as it aims to make single-board computers (SBCs) obsolete for projects like this. Contributions or sponsorships of any kind are greatly appreciated to help drive this project forward!

### Firmwares

    git clone https://github.com/migit/AI-Autonomous-Room-Rover-Robot-A2R3.git

### Softwares and Dev environments:
> [!TIP]
> Recommended Operating System: Linux (Ubuntu preferred for navigation and SLAM development) or Windows with Arduino-compatible IDE.
Software: <a href="https://docs.ros.org/en/foxy/index.html" target="_blank"> ROS2 </a> or <a href="https://micro.ros.org/" target="_blank">Micro-ROS </a> for <a href="https://docs.nav2.org/" target="_blank"> Nav2 </a>development. The robot autonomously navigates using a custom obstacle-avoidance algorithm on its <a href="https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/index.html" target="_blank">ESP32 </a>slave microcontroller.
Programming Language: C/C++

### Hardware and CAD design - Main drive:
> [!TIP]
> Plastic foam tires are ideal for the A2R3 rover, offering a lightweight, durable design with excellent grip at a relatively low cost. The rover is powered by a plastic gearbox RS390 actuator, driven by a 6V DC motor (tolerant up to 10V), delivering impressive speed and sufficient torque to handle payloads of up to 15kg. In the future, I plan to conduct additional scientific tests to optimize performance. For now, the first build, including firmware and hardware, is complete, and you can find the Bill of Materials (BOM) on my Hackster page linked below.

### Here are useful and important links for your guidance:      
> [!IMPORTANT]                                                                                                                                                   
    <a href="https://www.youtube.com/watch?v=NbiJPMn4Qm0" target="_blank">PID control build Demo Video</a> <br/>
    <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">Documentation</a> page.

    
This open source project has been officially certified as open source hardware by the Open Source Hardware Association under the project listing: https://certification.oshwa.org/fi000003.html [^2].
software and hardware Licenses: [^1]

[^1]: ![oshw_facts](https://github.com/user-attachments/assets/8db5b921-7199-43b5-9edd-f96adf9e9eec)
[^2]: ![Screenshot from 2025-03-30 15-58-53](https://github.com/user-attachments/assets/599f7daa-bc83-4e1d-ba89-f66ca4a2cc97)



