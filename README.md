# Unsupervised-WROFE2025-India
This is a repository for our WRO Future Engineers 2025 documentation. 

## Table of Contents
- [Unsupervised-WROFE2025-India](#unsupervised-wrofe2025-india)
  - [Table of Contents](#table-of-contents)
  - [Team](#team)
  - [About the challenge](#about-the-challenge)
  - [List of Components](#list-of-components)
  - [Design, Build, Code and Evaluate Process](#design-build-code-and-evaluate-process)
    - [Design](#design)
    - [Build](#build)
      - [Chassis](#chassis)
      - [Steering Mechanism](#steering-mechanism)
      - [Drive Mechanism](#drive-mechanism)
    - [Sensors and Perception](#sensors-and-perception)
    - [Power and Sense Management](#power-and-sense-management)
      - [LiPo Batteries](#lipo-batteries)
    - [Power Management](#power-management)
        - [DC Buck Converter:](#dc-buck-converter)
      - [Lithium Ion batteries:](#lithium-ion-batteries)
      - [Open Round Algorithm](#open-round-algorithm)
      - [Obstacle Round Algorithm](#obstacle-round-algorithm)
    - [Final Evaluation \& Scores](#final-evaluation--scores)
- [🚀 Robot Construction Guide](#-robot-construction-guide)
  - [🖨️ Step 0: Print the 3D Parts](#️-step-0-print-the-3d-parts)
    - [1. Prepare the microSD Card](#1-prepare-the-microsd-card)
    - [2. First Boot](#2-first-boot)
    - [3. Enable Required Interfaces](#3-enable-required-interfaces)
  - [Running the Code](#running-the-code)

## Team

<table style="width:100%; border: 1px solid black; border-collapse: collapse;">
  <thead>
    <tr>
      <th style="border: 1px solid black; padding: 8px;">Name</th>
      <th style="border: 1px solid black; padding: 8px;">Profile</th>
      <th style="border: 1px solid black; padding: 8px;">Role</th>
      <th style="border: 1px solid black; padding: 8px;">Contribution</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid black; padding: 8px;">1. Arnav Ramtake</td>
      <td style="border: 1px solid black; padding: 8px;">3rd year undergrad @ IIT Bombay</td>
      <td style="border: 1px solid black; padding: 8px;">Team Member</td>
      <td style="border: 1px solid black; padding: 8px;">Project development & Testing</td>
    </tr>
    <tr>
      <td style="border: 1px solid black; padding: 8px;">2. Kiara Bhandari</td>
      <td style="border: 1px solid black; padding: 8px;">Grade 9 @ Oberoi International School</td>
      <td style="border: 1px solid black; padding: 8px;">Team Member</td>
      <td style="border: 1px solid black; padding: 8px;">Project development & Testing</td>
    </tr>
    <tr>
      <td style="border: 1px solid black; padding: 8px;">3. Shubh Gupta</td>
      <td style="border: 1px solid black; padding: 8px;">Grade 9 @ Chatrabhuj Narsee School</td>
      <td style="border: 1px solid black; padding: 8px;">Team Member</td>
      <td style="border: 1px solid black; padding: 8px;">Project development & Testing</td>
    </tr>
    <tr>
      <td style="border: 1px solid black; padding: 8px;">4. Vinay Ummadi</td>
      <td style="border: 1px solid black; padding: 8px;">Mentor @ MakerWorks Lab</td>
      <td style="border: 1px solid black; padding: 8px;">Team Mentor</td>
      <td style="border: 1px solid black; padding: 8px;">Provided guidance and support</td>
    </tr>
  </tbody>
</table>

Team picture:

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/t-photos/team_photo1.jpeg" alt="Team Picture" width="500">

## About the challenge
The World Robot Olympiad (WRO) is an international robotics competition that encourages students to develop problem-solving, programming, and engineering skills.
The Future Engineers category is designed for students aged 14–22 years and focuses on autonomous driving. The challenge simulates real-world traffic conditions and requires teams to design and program a fully autonomous robot car.

The challenge requires students to construct an autonomous robot which will undergo 2 rounds. First, an open round challenge where the robot would need to complete 3 rounds around the arena within the time limit of 3 minutes (180 seconds). The second round, which is the obstacle round consists of navigating through red and green pillars, where the robot would move from the left of the green pillar and from right of the red pillar. The robot should once again, not exceed a time limit of 3 minutes.

## List of Components
<table border="1">
  <thead>
    <tr>
      <th>Component Name with Link</th>
      <th>Quantity</th>
      <th>Price</th>
    </tr>
  </thead>
  <tbody>
    <tr>
    <td><a href="https://robu.in/product/n20-6v-600-rpm-micro-metal-gear-motor/?gad_source=1&gad_campaignid=20381096599&gbraid=0AAAAADvLFWe2qiwILJ32C5JNwCPPtGWp3&gclid=Cj0KCQjw-4XFBhCBARIsAAdNOksdNfffwUSFCud8Fjba8RH2HJjGXhNb1jGbV-uPmwfpwsUavl32picaAkVOEALw_wcB">N20 DC Gear Motor</a></td>
      <td>1</td>
      <td>₹239</td>
    </tr>
    <tr>
    <td><a href="https://robu.in/product/raspberry-pi-5-model-8gb/?gad_source=1&gad_campaignid=19974686076&gbraid=0AAAAADvLFWccPNo1nAFbja4etEZbwSYsa&gclid=Cj0KCQjw-4XFBhCBARIsAAdNOksOP09gGjW_GFf7Vsckn2RSF4H_OcJGQZTPNndUeFb9ESBsUWQDfvsaAsGREALw_wcB">Raspberry Pi 5 (Cooling Fan + SD Card)</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      <tr>
    <td><a href="https://robu.in/product/raspberry-pi-camera-module-3-wide/"> Pi Camera 3 Wide Angle</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      <tr>
    <td><a href="https://share.google/GQIqzE7hObWl2JtNW">TB6612FNG Motor Driver</a></td>
      <td>1</td>
      <td>₹161</td>
    </tr>
      <tr>
    <td><a href="https://share.google/SYj7opQ4mhcO487lK">MG996R Servo Motor</a></td>
      <td>1</td>
      <td>₹347</td>
    </tr>
      <tr>
    <td><a href="https://robu.in/product/16-channel-12-bit-pwmservo-driver-i2c-interface-pca9685-arduino-raspberry-pi/">PCA9685 Servo Driver</a></td>
      <td>1</td>
      <td>₹223</td>
    </tr>
      <tr>
    <td><a href="https://share.google/BG01lgffrP5yxi8mK">N20 Wheels</a></td>
      <td>4</td>
      <td>₹140</td>
    </tr>
      <tr>
    <td><a href="https://share.google/xb8SxxgjGmudeReLK">LM2596S Voltage Regulator</a></td>
      <td>1</td>
      <td>₹41</td>
    </tr>
      <tr>
    <td><a href="https://robu.in/product/ydlidar-t-mini-plus-lidar-sensor/">YD LiDAR T-mini plus</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      <tr>
    <td><a href="https://www.amazon.in/Ambrane-Charging-Powerbank-Emergency-30/dp/B0D351GHY7?th=1">Power Bank</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      <tr>
    <td><a href="https://makerbazar.in/products/18650-3-7v-lithium-ion-rechargeable-cell-good-quality?variant=46013701423344&country=IN&currency=INR&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic">18650 Battery</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      <tr>
    <td><a href="https://share.google/fq9tZREMioah3AI9Q">Push Button Switch</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      <tr>
    <td><a href="https://robu.in/product/pct-spl-62-0-08-2-5mm-62-pole-wire-connector-terminal-block-with-spring-lock-lever-for-cable-connection/">Wire Connectors</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
      </tr>
      <tr>
    <td><a href="https://www.mathaelectronics.com/product/4-pole-2-way-dpdt-12pin-on-off-switch/">4 Pole 2 Way DPDT 12 pin ON/OFF Switch</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>

  </tbody>
</table>

## Design, Build, Code and Evaluate Process

### Design 
We started understading the given constraints and physical requirements of the robot and then designed the robot to meet these requirements. The design process included:
- **Understanding the Problem Statement**: We analyzed the requirements and constraints of the WRO Future Engineers 2025 competition.
- **Conceptualizing the Robot**: We brainstormed ideas for the robot's design, focusing on its chassis, steering mechanism, drive mechanism, sensors, and power management.
- **Creating Design Diagrams**: We created detailed design diagrams to visualize the robot's structure and components. These diagrams included:
  - Chassis design
  <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/chasis_design.jpeg" alt="Chasis Design Drawing" width="500">

  - Steering mechanism design
  
  - Drive mechanism design      

### Build


#### Chassis
The robot chasis is mainly designed from acrylic and 3d desgined parts. Robot consists of multiple layers to accommodate various components. Every structural component was fastened firmly with bolts and nuts to ensure durability and rigidity required for testing. For enhanced performance, the bottom acrylic sheet was carefully trimmed along the wheel areas, allowing for free wheel rotation and better maneuverability. Instead of expanding the robot's dimensyions in horizantally that would impact maneuverability, we opted for a three-layer vertical configuration. Not only did this choice in design improve space efficiency, but it also boosted the robot's performance by allowing better integration of its mechanical and electronic components in a compact footprint. The robot dimensions are 14cm(width) x 19cm(length) x 20cm(height). 

#### Steering Mechanism
We designed a distinct and novel steering mechanism that using a single servo motor which directly transfers motion to the steer both forward wheels by same angle. Both of the forward wheels are free to rotate independently, allowing for differential steering.  This is crucial when a robot or vehicle makes a turn, as the outer wheel has to cover a greater distance than the inner wheel. We have used the MG996R Servo Motor for the steering mechanism. 

MG996r Servo Motor:
<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Details</th>
    </tr>
  </thead>
  <tbody>
      <tr>
        <td><img src="https://www.jsumo.com/mg996r-servo-motor-digital-1701-65-B.jpg" alt="MG996r servo motor" height="200" width="200"></td>
        <td>
          1. MG996r Servo motor
          <br>
          <br>
          2. Digital servo: Pulse width modulation controlled(PWM)
        </td>
      </tr>
  </tbody>
</table>

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/PXL_20250817_102111155.jpg" alt="Steering Mechanism" width="500">


#### Drive Mechanism
An N20 DC gear motor powers the driving mechanism of the robot,transferring motion to the rear axle of the wheels via a gear system.The N20 motor is a small and light device (around 12 mm in diameter and weighing 10–12 g), frequently utilized in miniature robotics and DIY projects. It comes in 3 V, 6 V, and 12 V options, featuring gear ratios from 10:1 to 1000:1, allowing for a balance of speed and torque—lower ratios yield increased speed, while higher ratios offer enhanced torque. Featuring a 3 mm D-shaped output shaft and a robust metal gearbox, the N20 motor provides easy mounting affordability, and versatility, making it ideal for applications like line-following robots,remote-controlled vehicles, and small actuation systems.

N20 DC Gear motor:
<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Details</th>
    </tr>
  </thead>
  <tbody>
      <tr>
        <td><img src="https://robu.in/wp-content/uploads/2019/06/robu-7-11.jpg" alt="N20 dc Gear motor" height="200" width="200"></td>
        <td>
          1. N20 dc Gear Motor
          <br>
          <br>
          2. Speed range (depends on gear ratio): from ~30 RPM (high torque) to ~1000 RPM (low torque)
          <br>
          <br>
          3. High torque at low RPM due to gearbox
        </td>
      </tr>
  </tbody>
</table>

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/PXL_20250817_101933956.jpg" alt="Drive Mechanism" width="500">

### Sensors and Perception
LiDAR (YD LiDAR T-mini plus)  
The LiDAR sensor measures distances by emitting laser pulses and calculating their time-of-flight, providing precise range and angle data for mapping and navigation. With high accuracy and fast scan rates, it enables reliable obstacle detection, SLAM, and terrain mapping in real time.
Camera – Raspberry Pi Camera Module 3 (Wide)
The Raspberry Pi Camera Module 3 Wide features a 12 MP Sony IMX708 sensor with phase-detect autofocus, a 120° ultra-wide field of view, and HDR support. It delivers high-quality stills and 1080p video at up to 50 fps, making it well-suited for wide-angle imaging in robotics and vision-based applications.

<<<<<<< HEAD
### Power and Sense Management
#### LiPo Batteries 
| (images/18650battery) | *Specifications* | 
| -- | -------------- | 

=======
Lidar:
<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Details</th>
    </tr>
  </thead>
  <tbody>
      <tr>
        <td><img src="https://robu.in/wp-content/uploads/2024/11/YDLIDAR__T-mini_Plus-.jpg" alt="YD Lidar T-mini plus" width="200" height="200">
        <td>
          1. YD Lidar T-mini Plus
          <br>
          <br>
          2. 360° environment scanning which allows the robot to see in all directions, unlike ultrasonic/IR sensors which are narrow
          <br>
          <br>
          3. Range of up to 12 meters
        </td>
      </tr>
  </tbody>
</table>

Raspberry pi camera module 3 wide angle:
<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Details</th>
    </tr>
  </thead>
  <tbody>
<tr>
        <td><img width="200" height="200" alt="Raspberry Pi camera module 3 wide angle" src="https://github.com/user-attachments/assets/39f9bee2-c01f-469a-b27b-de2dcb5b5fcc" />
        </td>
        <td>
          1. Raspberry Pi camera module 3 wide angle
          <br>
          <br>
          2. 120° diagonal field of view (FoV), roughly corresponding to 102° horizontally and 67° vertically.
        </td>
      </tr>
  </tbody>
</table>

### Power Management


##### DC Buck Converter:
<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Details</th>
    </tr>
  </thead>
  <tbody>
      <tr>
        <td><img src="https://5.imimg.com/data5/SELLER/Default/2024/12/474327581/HV/WY/PL/147282047/lm2596s-dc-dc-buck-converter-power-supply.jpg" alt="LM2596S Voltage Regulator" width="200" height="200"></td>
        <td>
          1. DC buck converter  
          <br>
          <br>
          2. A buck converter or step-down converter is a DC-to-DC converter which decreases voltage, while increasing current, from its input (supply) to its output (load)
          <br>
          <br>
          3. DC-DC converter
        </td>
      </tr>
  </tbody>
</table>

#### Lithium Ion batteries:
 <table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Details</th>
    </tr>
  </thead>
  <tbody>
 <tr>
        <td><img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcSpzQU7jtEsYD5vpUwN10MAqyYzwncsd7vf-A&s" alt="18650 lithium ion batteries" hegiht=200 width=200>
        </td>
        <td>
          1. 18650 Lithium Ion batteries (11.1 V) 
          <br>
          <br>
          2. High energy density and capacity, leading to longer usage times and more power in a small package. 
        </td>
      </tr>
  </tbody>
</table>

#### Open Round Algorithm
We used LiDAR sensor to detect the walls. The algorithm used is as follows:
Inputs of the liDAR sensor are processed to determine the distance to the walls for each angle from -90 (LiDAR left) to 90 degrees(LiDAR right). LiDAR axis (0 degrees) is aligned with the robot axis. The error is calculated as the difference between LiDAR left and LiDAR right. Based on the error, the robot uses PID control to adjust its steering angle to correct its path. Kp, Ki, and Kd values are tuned to achieve the desired response. The same thing is used for turning left and right. 


<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/Open_Algorithm.png" alt="Open Round Algorithm" width="500">

#### Obstacle Round Algorithm

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/Obstacle_Algorithm.png" alt="Obstacle Round Algorithm" width="500">

### Final Evaluation & Scores
In our testing, we have achieved the following scores:
<table border="1">
  <thead>
    <tr>
      <th>Round</th>
      <th>Max Score Possible</th>
      <th>Score Achieved</th>
      <th>Time (seconds)</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Open Round</td>
      <td>30</td>
      <td>30</td>
      <td>60</td>
    </tr>
    <tr>
      <td>Obstacle Round</td>
      <td>40</td>
      <td>NA</td>
      <td>150</td>
    </tr>
  </tbody>
</table>

# 🚀 Robot Construction Guide

This document provides a step-by-step guide to building and setting up the robot.  

---

## 🖨️ Step 0: Print the 3D Parts
The 3D printable parts can be found in the `3d-models` folder.  
Recommended printer: **BambuLab X1-Carbon** (or equivalent).  

**Suggested print settings (update once finalized):**
- Material: PLA  
- Layer height: 0.2 mm  
- Infill: 20%  
- Supports: Yes  
- Raft: No  
- Brim: Yes  

---

⚙️ Step 1: Assemble the Chassis & Steering Servo

- Mount the **MG66R servo** onto the chassis using the provided slots or screws/zip ties.  
- Connect the servo horn to the steering linkage slot in the chassis.  
- Ensure the servo wire exits cleanly towards the electronics compartment.  
- The differential system relies on the **N20 motors**, while the servo provides steering correction.  



---

🔩 Step 2: Assemble the Powertrain

- Mount the **motor brackets** onto the chassis using screws.  
- Insert the **N20 gearmotors** into the brackets and secure them tightly.  
- Attach the **N20 wheels** directly to the motor shafts.  
- Ensure both wheels spin freely and remain aligned with the chassis.  
- If spacers are included in the 3D model, use them to level the wheels.  


---

🔌 Step 3: Attach the Electronics

- **Motor Driver (TB6612FNG):**  
  - Mount to the chassis (double-sided tape or 3D mount).  
  - Connect the N20 motors to the A and B outputs.  

- **PWM Controller (PCA9685):**  
  - Fix near the servo.  
  - Connect the MG66R servo to channel 0.  
  - Power the PCA9685 from the main battery (via Pi 5).  

- **Raspberry Pi 5:**  
  - Mount onto the electronics platform.  
  - Connect the TB6612FNG and PCA9685 via GPIO/I²C.  
  - Connect the **LiDAR sensor** via USB or UART (depending on module).  
  - Mount the **PiCamera3 Wide Angle** on the front of the chassis with a slight upward tilt.  

- **Battery Pack:**  
  - Place under or behind the electronics mount.  
  - Secure with Velcro or brackets.  
  - Connect battery output to a 5V regulator if required.  



---

📡 Step 4: Wiring Setup

- **Servo** → PCA9685 (channel 0)  
- **Motors** → TB6612FNG → Raspberry Pi GPIO  
- **PCA9685** → Raspberry Pi (I²C SDA + SCL)  
- **LiDAR** → Raspberry Pi (USB/UART)  
- **PiCamera3** → Raspberry Pi camera slot  

(Optional) Bundle excess wires with zip ties for a clean look.  


---

💻 Step 5: Software Setup

1. Insert a microSD card with **Raspberry Pi OS** into the Pi 5.  
2. Boot and complete first-time setup.  
3. Install dependencies:  
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install python3-pip i2c-tools
   pip3 install adafruit-circuitpython-servokit rplidar opencv-python
Enable I²C and camera support:
sudo raspi-config

>📥 Step 6: Upload the Code
Open Visual Studio Code on your laptop/desktop.
Connect to the Raspberry Pi via SSH or VS Code Remote.
Upload your control code (motor + servo + LiDAR + camera).
Run the program:
python3 main.py

🛠️ Step 7: Testing & Calibration
Power on the robot.
Test servo movement (ensure center position).
Run a basic motor test to confirm both N20 motors spin correctly.
Adjust LiDAR positioning and camera tilt.
Fine-tune the code for speed, steering, and stability.

📷 Media Placeholders
Add images or diagrams of your build process here:
docs/images/chassis.jpg
docs/images/electronics.jpg
docs/images/wiring.jpg




📝 Raspberry Pi 5 Initialization Steps

### 1. Prepare the microSD Card
- Download the latest **Raspberry Pi OS (64-bit)** from the official Raspberry Pi website.  
- Use **Raspberry Pi Imager** (or BalenaEtcher) to flash the OS to a microSD card (32GB recommended).  
- Insert the flashed microSD card into the Raspberry Pi 5.  

---

### 2. First Boot
- Connect the Pi to a monitor, keyboard, and mouse.  
- Power on the Pi using your battery pack or a 5V/5A USB-C supply.  
- Go through the first-time setup wizard:  
  - Select language, time zone, and keyboard layout.  
  - Connect to Wi-Fi.  
  - Update the system when prompted.  

---

### 3. Enable Required Interfaces
Open terminal and run:
``` bash
  sudo raspi-config

  Enable the following:
  I²C (for PCA9685 servo controller)
  Camera (for PiCamera3)
  SSH (for remote programming from VS Code)
  Serial/UART (if your LiDAR uses UART)
  Reboot the Pi after enabling:
  sudo reboot

  4. Install System Updates & Tools
  Run these commands:
  sudo apt update && sudo apt upgrade -y
  sudo apt install python3-pip git i2c-tools

  5. Install Python Libraries
  Install required libraries for motors, LiDAR, and camera:
  pip3 install adafruit-circuitpython-servokit rplidar opencv-python

  6. (Optional) VS Code Remote Setup
  To program directly from your laptop using Visual Studio Code:
  Install the Remote SSH extension in VS Code.
  Connect to the Pi using its IP address:
  ssh pi@<your_pi_ip>
  Default username: pi
  Default password: raspberry (change this after first login with passwd).
  
``` bash 
 






## Running the Code
To run the code, follow these steps:


[View LiDAR in 3D](3d/lidar.stl) 