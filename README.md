Story

Introduction

In an era where artificial intelligence (AI) and robotics are transforming our daily lives, creating an AI-powered autonomous room rover is an exciting project for any tech enthusiast. I have always wondered how it feels like to build this idea from the ground up alone with limited resources i have available. This rover bot is designed to navigate a room autonomously, avoiding obstacles and responding to voice commands.

Imagine having a companion rover bot who actually knows you by name and serves you tirelessly (well while 🔋 is full :) ). I am designing A2R3 as an open source cost effective diy robotic platform for all who want to customize their own rover bot.

This project will guide you through the process of building your own intelligent rover using readily available components and open-source software step by step. I divided this project in three parts for simplicity. You can also find the whole project decscription in my <a href="https://www.hackster.io/mikroller/ai-autonomous-room-rover-robot-a2r3-part-2-48f5a5" target="_blank">hackster</a> page.

WELCOME to the first series part1: building the obstacle avoidance system.

Objectives

    Autonomous Navigation: Enable the rover to move around a room without human intervention, avoiding obstacles.
    Voice Control: Incorporate voice recognition to control the rover’s movements.
    Real-Time Feedback: Implement sensors for real-time data collection and response.

Software Requirements

    Operating System: Linux or windows Arduino IDE installed ( I recommend working on Linux (Ubuntu) for future navigation and SLAM development system to use 
    ROS2 or micro ROS)
    Programming Language: C++

Main drive Hardware Requirements

    Plastic foam tires are great for this project and they are lightweight, hard and have a good grip. Relatively less costy.
    The actuator to drive the rover is plastic agearbox. It has a 6V DC motor inside and produce a fair amount of speed with a   high torque strong enough to d 
    drive the rover with up to 15kg amount of payload. (I Iet my 15kg son drive on it). In the        future I will do some other scientific tests.

Algorithm

    obstacle avoidance

    Drive forward until obstacle is detected. If Obstacle is detected then stop for few seconds then move backwards for few        centimeters. Then move again try if still there is the obstacle. If the obstacle is there move backwards and turn right        for some degrees to avoid the obstacle and continue straight forward.

Demonstration
    <a href="https://youtu.be/E3wDgulsSTU?si=qFQs4_kfr9r9EPV2" target="_blank">Part 1</a>
    <a href="https://youtu.be/E3wDgulsSTU?si=qFQs4_kfr9r9EPV2" target="_blank">Part 1</a>
