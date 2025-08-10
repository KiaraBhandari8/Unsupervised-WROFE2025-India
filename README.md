# Unsupervised-WROFE2025-India
This is a repository for our WRO Future Engineers 2025 documentation. 

## Table of Contents
- [Unsupervised-WROFE2025-India](#unsupervised-wrofe2025-india)
  - [Table of Contents](#table-of-contents)
  - [Team](#team)
  - [Design, Build, Code and Evaluate Process](#design-build-code-and-evaluate-process)
    - [Design](#design)
    - [Build](#build)
      - [Chassis](#chassis)
      - [Steering Mechanism](#steering-mechanism)
      - [Drive Mechanism](#drive-mechanism)
    - [Sensors and Perception](#sensors-and-perception)
    - [Power Management](#power-management)
    - [Algorithm and Code](#algorithm-and-code)
      - [Open Round Algorithm](#open-round-algorithm)
      - [Obstacle Round Algorithm](#obstacle-round-algorithm)
    - [Final Evaluation \& Scores](#final-evaluation--scores)

## Team

|Name|Profile|Role|Contribution|
|---|---|---|---|
|1. Arnav Ramtake| 3rd year undergrad @ IIT Bombay|Team Member|Project development & Testing|
|2. Kiara Bhandari| Grade 9 @ Oberoi International School|Team Member|Project development & Testing|
|3. Shubh Gupta| Grade 9 @ Chatrabhuj Narsee School|Team Member|Project development & Testing|
|4. Vinay Ummadi|Mentor @ MakerWorks Lab|Team Mentor|Provided guidance and support|

Team picture:
![Team Picture](/t-photos/team_photo1.jpeg)

## Design, Build, Code and Evaluate Process

### Design 
We started understading the given constraints and physical requirements of the robot and then designed the robot to meet these requirements. The design process included:
- **Understanding the Problem Statement**: We analyzed the requirements and constraints of the WRO Future Engineers 2025 competition.
- **Conceptualizing the Robot**: We brainstormed ideas for the robot's design, focusing on its chassis, steering mechanism, drive mechanism, sensors, and power management.
- **Creating Design Diagrams**: We created detailed design diagrams to visualize the robot's structure and components. These diagrams included:
  - Chassis design
  ![Team Picture](/schemes/addl/chasis_design.jpeg)
  .jpeg)
  - Steering mechanism design
  - Drive mechanism design      

### Build


#### Chassis
The robot consisted of a layered design composed of three sheets of acrylic, supported by a base produced by 3D printing. Every structural component was fastened firmly with bolts and nuts to ensure durability and longevity in application. For enhanced performance, the bottom acrylic sheet was carefully trimmed along the wheel areas, allowing for free wheel rotation and better maneuverability. Instead of expanding the robot's overall design that would impact maneuverability, we opted for a three-layer vertical configuration. Not only did this choice in design improve space efficiency, but it also boosted the robot's performance by allowing better integration of its mechanical and electronic components in a compact footprint
#### Steering Mechanism
A differential is a gear arrangement that divides power from a single motor or axle into two outputs, enabling them to rotate at varying speeds. This is crucial when a robot or vehicle makes a turn, as the outer wheel has to cover a greater distance than the inner wheel. In our robot, we implemented a differential gear to enable sharper and more accurate turns in a shorter timeframe, enhancing both speed and maneuverability during operation. The primary components consist of a drive gear, ring gear, small spider gears, and side gears linked to the wheels
#### Drive Mechanism
An N20 DC gear motor powers the driving mechanism of the robot,transferring motion to the rear axle of the wheels via a gear system.The N20 motor is a small and light device (around 12 mm in diameter and weighing 10–12 g), frequently utilized in miniature robotics and DIY projects. It comes in 3 V, 6 V, and 12 V options, featuring gear ratios from 10:1 to 1000:1, allowing for a balance of speed and torque—lower ratios yield increased speed, while higher ratios offer enhanced torque. Featuring a 3 mm D-shaped output shaft and a robust metal gearbox, the N20 motor provides easy mounting affordability, and versatility, making it ideal for applications like line-following robots,remote-controlled vehicles, and small actuation systems.
### Sensors and Perception
LiDAR (YD LiDAR T-mini plus)  
The LiDAR sensor measures distances by emitting laser pulses and calculating their time-of-flight, providing precise range and angle data for mapping and navigation. With high accuracy and fast scan rates, it enables reliable obstacle detection, SLAM, and terrain mapping in real time.
Camera – Raspberry Pi Camera Module 3 (Wide)
The Raspberry Pi Camera Module 3 Wide features a 12 MP Sony IMX708 sensor with phase-detect autofocus, a 120° ultra-wide field of view, and HDR support. It delivers high-quality stills and 1080p video at up to 50 fps, making it well-suited for wide-angle imaging in robotics and vision-based applications.

### Power Management

### Algorithm and Code

#### Open Round Algorithm
We used LiDAR sensor to detect the walls. The algorithm used is as follows:
Inputs of the liDAR sensor are processed to determine the distance to the walls for each angle from -90 (LiDAR left) to 90 degrees(LiDAR right). LiDAR axis (0 degrees) is aligned with the robot axis. The error is calculated as the difference between LiDAR left and LiDAR right. Based on the error, the robot uses PID control to adjust its steering angle to correct its path. Kp, Ki, and Kd values are tuned to achieve the desired response. The same thing is used for turning left and right. 


![Flowchart](/schemes/addl/openroundalgorithm.jpeg)


#### Obstacle Round Algorithm

### Final Evaluation & Scores
