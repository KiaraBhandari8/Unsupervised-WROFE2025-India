# Unsupervised-WROFE2025-India
This is a repository for our WRO Future Engineers 2025 documentation. 

## Table of Contents
- [Unsupervised-WROFE2025-India](#unsupervised-wrofe2025-india)
  - [Table of Contents](#table-of-contents)
  - [Team](#team)
  - [About the challenge](#about-the-challenge)
  - [List of Components](#list-of-components)
  - [Robot Pictures](#robot-pictures)
  - [Design, Build, Code and Evaluate Process](#design-build-code-and-evaluate-process)
    - [Design](#design)
    - [Mobility Management](#mobility-management)
      - [Powertrain](#powertrain)
        - [Drivetrain](#drivetrain)
        - [Motor](#motor)
        - [Motor Driver](#motor-driver)
    - [Build](#build)
      - [Chassis](#chassis)
      - [Steering Mechanism](#steering-mechanism)
      - [Drive Mechanism](#drive-mechanism)
    - [Sensors and Perception](#sensors-and-perception)
      - [LiDAR – YDLIDAR T-mini Plus](#lidar--ydlidar-t-mini-plus)
      - [Camera – Raspberry Pi Camera Module 3 (Wide)](#camera--raspberry-pi-camera-module-3-wide)
    - [Power and sense management](#power-and-sense-management)
        - [DC Buck Converter:](#dc-buck-converter)
      - [Lithium Ion batteries:](#lithium-ion-batteries)
      - [Open Round Algorithm](#open-round-algorithm)
      - [Obstacle Round Algorithm](#obstacle-round-algorithm)
    - [Final Evaluation \& Scores](#final-evaluation--scores)
  - [🚀 Robot Construction Guide](#-robot-construction-guide)
    - [🖨️ 0: Print the 3D Parts](#️-step-0-print-the-3d-parts)
    - [1. Prepare the microSD Card](#1-prepare-the-microsd-card)
    - [2. First Boot](#2-first-boot)
    - [3. Enable Required Interfaces](#3-enable-required-interfaces)

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
      <td>₹8,291</td>
    </tr>
      <tr>
    <td><a href="https://robu.in/product/raspberry-pi-camera-module-3-wide/"> Pi Camera Module 3 Wide Angle</a></td>
      <td>1</td>
      <td>₹3,500</td>
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
      <td>₹9,595</td>
    </tr>
      <tr>
    <td><a href="https://www.amazon.in/Ambrane-Charging-Powerbank-Emergency-30/dp/B0D351GHY7?th=1">Power Bank</a></td>
      <td>1</td>
      <td>₹1,699</td>
    </tr>
      <tr>
    <td><a href="https://makerbazar.in/products/18650-3-7v-lithium-ion-rechargeable-cell-good-quality?variant=46013701423344&country=IN&currency=INR&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic">18650 Battery</a></td>
      <td>2</td>
      <td>₹35</td>
    </tr>
      <tr>
    <td><a href="https://share.google/fq9tZREMioah3AI9Q">Push Button Switch</a></td>
      <td>1</td>
      <td>₹46</td>
    </tr>
      <tr>
    <td><a href="https://robu.in/product/pct-spl-62-0-08-2-5mm-62-pole-wire-connector-terminal-block-with-spring-lock-lever-for-cable-connection/">Wire Connectors</a></td>
      <td>1</td>
      <td>₹58</td>
    </tr>
      </tr>
      <tr>
    <td><a href="https://www.mathaelectronics.com/product/4-pole-2-way-dpdt-12pin-on-off-switch/">4 Pole 2 Way DPDT 12 pin ON/OFF Switch</a></td>
      <td>1</td>
      <td>₹500</td>
    </tr>
    <tr>
    <td><a href="[https://share.google/fq9tZREMioah3AI9Q](https://www.amazon.in/dp/B0D9VWCX66?ref=ppx_yo2ov_dt_b_fed_asin_title)">RC Car Rear Differential</a></td>
      <td>1</td>
      <td>₹1,275</td>
    </tr>

  </tbody>
</table>

## Robot Pictures 🤖
<table border="1">
  <thead>
    <tr>
      <th>Top</th>
      <th>Bottom</th>
    </tr>
  </thead>
  <tr>
    <td>
  <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/Top-removebg-preview.png" alt="Robot_Top" width="350" height="350"></td>
    <td><img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/Bottom1.png" alt="Robot_Bottom" width="225" height="225"></td>
  </tr>
  <thead>
    <tr>
      <th>Front</th>
      <th>Back</th>
    </tr>
  </thead>
  <tr>
    <td>
  <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/Front-removebg-preview.png" alt="Robot_Front" width="250" height="250"></td>
    <td><img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/Back-removebg-preview.png" alt="Robot_Back" width="250" height="250"></td>
  </tr>
   <thead>
    <tr>
      <th>Left</th>
      <th>Right</th>
    </tr>
  </thead>
  <tr>
    <td>
  <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/Left-removebg-preview.png" alt="Robot_Left" width="250" height="350"></td>
    <td><img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/Right-removebg-preview.png" alt="Robot_Right" width="250" height="325"></td>
  </tr>
</table>


### Design Process
We started understading the given constraints and physical requirements of the robot and then designed the robot to meet these requirements. The design process included:
- **Understanding the Problem Statement**: We analyzed the requirements and constraints of the WRO Future Engineers 2025 competition.
- **Conceptualizing the Robot**: We brainstormed ideas for the robot's design, focusing on its chassis, steering mechanism, drive mechanism, sensors, and power management.
- **Creating Design Diagrams**: We created detailed design diagrams to visualize the robot's structure and components. These diagrams included:
  - Chassis design

  - Steering mechanism design
  
  - Drive mechanism design      
### Circuit Schematics
![Alt text](https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/block.drawio.png?raw=true)
![Alt text](https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/images/circuit_schematics.drawio.png?raw=true)
### Mobility Management

The robot's mobility is managed by a combination of compoenents, icluding the powertrain, sterring system and chassis. These elements work together to ensure the robot's smooth and efficient movement. 

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/PXL_20250817_101933956.jpg" alt="Drive Mechanism" width="500">


#### Powertrain 

##### Drivetrain 
To minimize friction and therby reduce speed loss, driving axle was made from an steel rod. We connected the orot to the riving axle using a custom 3-D printed adapter. The driing axle is equipped with differential gears, which allow the robot to turn smoothly. For traction we used N20 wheels. 

Potential Improvements:

- Upgrade to higher-quality differential gear set for enhanced efficiency and smoother cornering. 
- Experiment with lighter alloys and hollow rods to balance strength with reduced weight. 
- Replace the 3D printed adapter with a kachined metal one for greater durability and precision.

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/differentialgear_stand.stl.gif" alt="Drive Mechanism" width="500"> <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/differentialgear_2.stl.gif" alt="Drive Mechanism" width="500"> <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/differentialgear_3.stl.gif" alt="Drive Mechanism" width="500"> 

##### Motor

N20 DC Gear motor:
<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Specifications:</th>
    </tr>
  </thead>
  <tbody>
      <tr>
        <td><img src="https://robu.in/wp-content/uploads/2019/06/robu-7-11.jpg" alt="N20 dc Gear motor" height="200" width="200"></td>
        <td>
          1. Name: N20 dc Gear Motor
          <br>
          <br>
          2. Speed Range:  ~ 30 RPm to ~ 2000 RPM
          <br>
          <br>
          3. High torque at low RPM due to gearbox
        </td>
      </tr>
  </tbody>
</table>

Following an evaluation of different motors, we settles on the **N20-6V-600 Rpm Micro Metal Gear Motor**. THis motor was selected for its lightweight and compact design, which stands out among others with comparable output. We secured the motor to the chassis using a custom 3-D printed holder. 

Where to buy the motor: [N20 Motor](https://robu.in/product/n20-6v-600-rpm-micro-metal-gear-motor/?gad_source=1&gad_campaignid=20381096599&gbraid=0AAAAADvLFWe2qiwILJ32C5JNwCPPtGWp3&gclid=Cj0KCQjw-4XFBhCBARIsAAdNOksdNfffwUSFCud8Fjba8RH2HJjGXhNb1jGbV-uPmwfpwsUavl32picaAkVOEALw_wcB) 

To connect the motor's axle to the chassis, we created a custom 3D - printed adapter. 

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/models/axle_holder.stl.gif" alt="Axle Holder" width="200"> <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/models/rearwheel_axle_bracket.stl.gif" alt="Axle Holder" width="200"> <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/models/rear_axle_holder.stl.gif" alt="Axle Holder" width="200"> 

Potential Improvements:
- Consider upgrading to a higher-torque or lower-RPM variant of the N20 motor for improved control and load handling.
- Replace the custom 3D-printed adapter with a machined metal connector to enhance strength and long-term reliability.
- Implement a sturdier motor mount or holder to minimize vibrations and ensure better alignment with the axle.




<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/PXL_20250817_101933956.jpg" alt="Drive Mechanism" width="500">

##### Motor Driver 

To control the speed of the drive motor, we utilized the SparkFun Dual TB6612FNG motor driver. 

<table border="1">
  <thead>
    <tr>
      <th style="width:300;">Component Image</th>
      <th style="width:200;">Specifications:</th>
    </tr>
  </thead>
  <tbody>
      <tr>
        <td><img src="https://encrypted-tbn1.gstatic.com/shopping?q=tbn:ANd9GcSKUTxRfuAtlUU0PhWXWyiSGpCcXlawcLx_tdUh4Kg0qDFXzZCxnrLLnl0BveQacK3BHN2lCsaOprJau9KqzQzv7GQrtM8G_z3Y95u-KYSwJJX7_vmguI6cyA" alt="TB6612FNG Motor driver" height="200" width="200"></td>
        <td>
          1. Name: TB6612FNG
          <br>
          <br>
          2. Motor Voltage (VM): 4.5–13.5 V
          <br>
          <br>
          3. Logic Voltage (VCC): 2.7–5.5 V
          <br>
          <br>
          4. Output Current: 1.2 A continuous, 3.2 A peak per channel
          <br>
          <br>
          5. Control Pins: Direction, PWM speed, Standby
        </td>
      </tr>
  </tbody>
</table>


Where to buy the motor drive: https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-with-headers.html

Potential Improvements:
- Replace the motor driver with a custom PCB to reduce weight and improve space utiliation 
- Add active cooling or heating sinks for enhanced thermal performance during extended use.
- Explore higher-current motor drivers to accomodate potential motor upgrades. 
- Implement a more robust power manangement system to ensure reliable operation. 



### Build ⚒ 


#### Chassis
The robot chasis is mainly designed from acrylic and 3d desgined parts. Robot consists of multiple layers to accommodate various components. Every structural component was fastened firmly with bolts and nuts to ensure durability and rigidity required for testing. For enhanced performance, the bottom acrylic sheet was carefully trimmed along the wheel areas, allowing for free wheel rotation and better maneuverability. Instead of expanding the robot's dimensyions in horizantally that would impact maneuverability, we opted for a three-layer vertical configuration. Not only did this choice in design improve space efficiency, but it also boosted the robot's performance by allowing better integration of its mechanical and electronic components in a compact footprint. The robot dimensions are 14cm(width) x 19cm(length) x 20cm(height). 

#### Steering Mechanism
The robot employs a distinct and novel steering mechanism designed to achieve efficient and reliable maneuvering using a single servo motor. An MG996R high-torque servo motor is directly coupled to the steering linkage, allowing both front wheels to turn at the same angle simultaneously. The forward wheels are free to rotate independently and are supported by a differential gear system, which ensures that each wheel can rotate at the appropriate speed when the robot takes a turn. This is essential because the outer wheel must travel a greater distance than the inner wheel during cornering, and the differential allows this motion without skidding or energy loss. By combining the servo-driven steering system with differential gears, the robot achieves precise steering control, mechanical efficiency, and smooth navigation across both straight paths and turns.
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


### Sensors and Perception

#### LiDAR – YDLIDAR T-mini Plus
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
          • Model: YDLIDAR T-mini Plus  
          <br><br>
          • 360° scanning capability for complete environmental awareness  
          <br><br>
          • Maximum range: up to 12 meters  
          <br><br>
          • Provides high-frequency distance and angle data for mapping and navigation
        </td>
      </tr>
  </tbody>
</table>

The LiDAR sensor operates on the principle of **time-of-flight measurement**, emitting rapid laser pulses and calculating the return time after striking objects. This provides precise range and angular data, enabling the robot to construct accurate real-time maps of its surroundings. Its wide scanning field, high accuracy, and fast update rate make it essential for **obstacle detection, Simultaneous Localization and Mapping (SLAM), and terrain mapping**. With LiDAR, the robot can not only avoid obstacles but also refine its spatial understanding continuously, ensuring smooth navigation even in dynamic or unfamiliar environments.

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/lidar.stl.gif" alt="Front Battery Holder" width="300" height="300"> 

Potential Improvements:
+ Use RANSAC line fitting or clustering to detect walls and obstacles more reliably
+ Apply filtering (median, moving average, outlier rejection, low-pass on derivative) for smoother control inputs
+ Add gain scheduling or adaptive PID to adjust steering dynamically with speed and environment
+ Try predictive controllers like Pure Pursuit, Stanley, or MPC for smoother navigation
+ Fuse LiDAR with IMU/encoders for better localization and drift correction
+ Use dynamic window or VFH (Vector Field Histogram) for local obstacle avoidance
+ Apply SLAM or map-based localization for improved path planning in complex environments
+ Add trajectory prediction to handle moving obstacles effectively

#### Camera – Raspberry Pi Camera Module 3 (Wide)
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
          • 12 MP Sony IMX708 sensor with HDR support  
          <br><br>
          • 120° ultra-wide diagonal field of view (≈102° horizontal, 67° vertical)  
          <br><br>
          • Phase-detect autofocus for sharp imaging  
          <br><br>
          • 1080p video at up to 50 fps for real-time processing
        </td>
      </tr>
  </tbody>
</table>

The Raspberry Pi Camera Module 3 (Wide) was chosen due to its ability to capture a broad field of view, critical for detecting obstacles, monitoring lanes, and ensuring spatial awareness with fewer blind spots. Its **high-resolution sensor and HDR capability** deliver clear images across varying light conditions, while the autofocus ensures reliable detection of both nearby and distant objects. The **high frame rate (50 fps)** ensures smooth real-time video streaming, which is vital for **vision-based navigation, SLAM, and obstacle recognition**. This makes it an optimal perception sensor for the robot’s autonomous operation.

Potential Improvements:

+ Upgrade to a camera with higher resolution for better object detection and tracking.
+ Test alternative lighting solutions to improve visibility in various conditions.
+ Implement advanced image processing algorithms to enhance detection accuracy.
+ Use color correction algorithms: Apply color correction algorithms to compensate for any variations in lighting conditions. These algorithms can adjust the color values of the captured image to match a reference color space, making the colors more consistent and easier to calibrate.
+ Provide user-adjustable parameters: Allow users to manually adjust color thresholds or ranges to fine-tune the color detection. This can be done through a user interface or by providing configuration files that can be modified.
+ Implement real-time feedback: Display the detected colors in real-time to the user, along with the calibrated values. This allows users to visually verify the accuracy of the color detection and make adjustments if necessary.

### Power and sense management

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

The robot is powered by a 3-cell 18650 battery pack (~11.1 V nominal), which acts as the main source of energy. Since the subsystems operate at different voltages, a two-rail power setup is used:

+ Direct 11.1 V rail – drives components that can operate directly from the pack
+ 3.3 V regulated rail – derived using an LM2596 buck DC-DC converter

The LM2596 was chosen because it provides 80–90% efficiency, avoiding the excessive heat losses of linear regulators. It also ensures:

+ Stable 3.3 V output across the pack’s 9–12.6 V range
+ Reliable operation of sensors, logic, and microcontrollers
+ Adequate current headroom for the control electronics
+ Simplified power management and protection of sensitive devices

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

The robot is powered by a 3-cell 18650 non-rechargeable battery pack (11.1 V nominal), selected to balance reliability, compactness, and sufficient energy supply for autonomous operation.

+ Stable voltage output – consistently provides ~11.1 V (9–12.6 V range) suitable for motors, sensors, and control boards
+ Non-rechargeable chemistry – eliminates risks of recharge-related degradation, swelling, or maintenance, ensuring dependable short- to medium-term use
+ Motor compatibility – higher voltage level supports motor drivers more effectively than AA or 9 V alternatives
+ Extended runtime – high energy density of 18650 cells delivers longer operation before replacement compared to conventional disposable batteries
+ Compact and lightweight – cylindrical form factor makes the pack easy to integrate without adding unnecessary bulk
+ Consistent discharge curve – maintains steady performance over usage cycle, preventing voltage drops that could affect control electronics

### 18650 Battery holder
<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/battery_holder.stl.gif" alt="Front Battery Holder" width="300" height="300"> <img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/rearbattery_stand.stl.gif" alt="Rear Battery Holder" width="300" height="300">

+ Compact Form Factor – The slim rectangular frame securely houses the 3-cell 18650 pack while minimizing wasted volume, making it well-suited for robots with limited internal space.
+ Structural Stability – The rigid walls and elongated side frame provide strong mechanical support, preventing cell movement or vibration during operation.
+ Mounting Flexibility – Pre-drilled holes allow easy attachment to the robot chassis, reducing the need for additional brackets or adhesives.
+ Weight Optimization – Open frame design reduces unnecessary material use, keeping the holder lightweight without compromising durability.
+ Thermal Management – The open sides promote airflow around the batteries, preventing heat buildup and improving safety during high-current discharge.
+ Maintenance-Friendly – The accessible frame allows straightforward battery replacement, inspection, or wiring adjustments.
+ Secure Alignment – Ensures the 18650 cells remain aligned in series/parallel configuration, reducing electrical contact resistance and improving power reliability.

#### Power Bank holder:
<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/models/powerbank_front_bracket.stl.gif" alt="Power Bank Holder" width="300" height="250">

+ Minimalistic frame reduces material use, keeping it lightweight yet strong
+ Raised side arms secure the power bank and prevent shifting during operation
+ Central mounting hole allows easy attachment to the chassis
+ Open design ensures quick access for wiring and charging without removal
+ Provides airflow around the power bank for passive cooling
+ Stable and practical mounting without adding bulk to the robot

#### Potential Improvements:
+ Use rubber padding or cushioning to reduce shock and protect the power bank from mechanical stress.
+ Design for tool-less removal so the power bank can be swapped quickly.
+ Include cable management features (clips or slots) to keep wires tidy and strain-free
+ Add universal sizing adjustability to fit different power bank models if needed.
+ Optimize the weight distribution so the holder doesn’t affect the robots performance


## Open Round Algorithm
### 1) Initialization
+ Robot initializes LiDAR and motion subsystems.
+ A 5-second pause allows hardware to stabilize.
+ PID controller parameters (Kp, Ki, Kd) are set depending on clockwise/anticlockwise wall-following mode.

### 2) LiDAR Data Acquisition
+ The LidarScanner continuously collects angle–distance pairs (-180° to +180°).
+ Distances are filtered to valid ranges, converted to millimeters, and stored in a dictionary.

### 3) Turn Detection
+ Front LiDAR readings (±10° sector) are averaged separately for left and right.
+ If both fall within a threshold range (1500–2000 mm), the system flags a turn condition.
+ Turn events are counted, but with a cooldown (≥2.5 s) to avoid multiple detections for the same turn.

### 4) Wall Distance Processing
+ Right wall sector: 30° to 90°.
+ Left wall sector: -90° to -30°.
+ Front sector: -5° to +5° (safety check).
+ Distances in each sector are averaged to estimate proximity to arena walls.

### 5) Error Calculation:
The four conditions include:
+ If both walls are visible: error = (right distance – left distance) → keeps robot centered.
+ If only one wall is visible: error = (wall distance – target distance) → maintains fixed offset.
+ If no walls detected: error = 0 → hold course.
+ If front obstacle is closer than safety threshold (e.g., <300 mm): error = 9999 (forces emergency stop).

### 6) PID Control
+ The error is fed into a PID controller to smooth steering response.
+ PID output adjusts servo angle around a center position (≈95°).
+ Output is clamped to ±20° for safe steering limits.

### 7) Motion Control
+ If no obstacle: robot moves forward at constant speed (0.6).
+ Servo adjusts steering continuously to minimize PID error.
+ If obstacle detected: robot stops, pauses, and waits before continuing.

### 8) Termination Condition
+ After max_turn_count = 12 turns, the robot stops permanently (end of round).

## Step by step breakdown of functions (Open Round):

### 1) read_lidar_data(lidar):
``` python
import serial
import struct
import math
import time
import ydlidar

# --- LiDAR Scanner Class (No Changes) ---
class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.laser = None
        self.scan_data = {}

        self.MIN_ANGLE = -180.0
        self.MAX_ANGLE = 180.0
        self.MIN_RANGE = 0.02
        self.MAX_RANGE = 16.0

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        try:
            ydlidar.os_init()
            self.laser = ydlidar.CYdLidar()

            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.MAX_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.MIN_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.MAX_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.MIN_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

            ret = self.laser.initialize()
            if not ret:
                raise IOError(f"LiDAR connection failed: {self.laser.DescribeError()}")

            ret = self.laser.turnOn()
            if not ret:
                raise IOError(f"Failed to turn on YDLIDAR: {self.laser.DescribeError()}")

            print(f"LiDAR: Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            print(f"LiDAR ERROR: Could not connect to LiDAR: {e}")
            raise IOError(f"LiDAR connection failed: {e}")

    def disconnect(self):
        if self.laser:
            print("LiDAR: Disconnecting...")
            self.laser.turnOff()
            self.laser.disconnecting()
            self.laser = None
            print("LiDAR: Disconnected.")

    def get_scan_data(self):
        if not self.laser:
            return None

        self.scan_data = {}
        scan = ydlidar.LaserScan()

        try:
            r = self.laser.doProcessSimple(scan)
            if r:
                for p in scan.points:
                    if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                        angle_degrees = round(math.degrees(p.angle))
                        distance_mm = p.range * 1000
                        self.scan_data[angle_degrees] = distance_mm
                return self.scan_data
            else:
                return None
        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None

```
### Explanation:
+ The LiDAR constantly scans in a 180° range around the front of the robot.
+ Data is divided into zones:
  + Left zone: -90° to -30°
  + Right zone: +30° to +90°
  + Forward zone: -15° to +15°
+ By averaging readings, sudden spikes (noise) are reduced.
+ The robot then knows how far it is from left wall, right wall, and any object in front.

### compute_wall_error:
``` python
def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=150):
    """
    Calculates the steering error based on LiDAR scan data to keep the robot
    at a target distance from the walls. Dynamically adjusts sensor angles
    based on front proximity.
    """
    # Define angular range for front detection
    front_angles_degrees = [angle for angle in range(-5, 6)] # -5 to 5 degrees inclusive

    # ## --- NEW LOGIC START --- ##
    # Check for close frontal obstacles to decide on the wall-sensing angle range
    front_distances = [
        scan_data[angle] for angle in front_angles_degrees
        if angle in scan_data and scan_data[angle] is not None and scan_data[angle] > 0
    ]
    
    is_front_obstacle_close = False
    if front_distances:
        avg_front_distance = sum(front_distances) / len(front_distances)
        if avg_front_distance < 350:
            is_front_obstacle_close = True

    # Dynamically set the left and right wall sensing angles
    if is_front_obstacle_close:
        # Wider angle ranges for close-quarters navigation
        print("LiDAR: Close-range mode activated (wider angles).")
        right_wall_angles_degrees = [angle for angle in range(30, 110)]  # 30 to 110
        left_wall_angles_degrees = [angle for angle in range(-110, -30)] # -110 to -29
    else:
        # Default (standard) angle ranges
        right_wall_angles_degrees = [angle for angle in range(30, 90)]   # 30 to 90
        left_wall_angles_degrees = [angle for angle in range(-90, -30)]  # -90 to -29
```
### Explanation:
+ This function ensures the robot stays centered in the arena.
+ If left > right → robot is drifting towards the right wall → needs to turn left.
+ If right > left → robot is drifting towards the left wall → needs to turn right.
+ The error value is passed into the PID controller for smooth correction.

### pid_controller:
``` python 
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_error):
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0:
            return self.prev_error

        P = self.Kp * current_error
        self.integral += current_error * dt
        I = self.Ki * self.integral
        derivative = (current_error - self.prev_error) / dt
        D = self.Kd * derivative
        output = P + I + D
        self.prev_error = current_error
        self.last_time = current_time
        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
```
### Explanation:
+ This is the core control function.
+ Works just like an autopilot system.
+ P (Proportional) – responds strongly to the current error.
+ I (Integral) – accumulates small errors over time (helps correct drift).
+ D (Derivative) – predicts future error change to avoid oscillations.
+ Outputs a steering correction angle for the servo.

### Explanation:
+ Converts PID output into servo rotation.
+ Prevents excessive turning (>45°) which could damage servo or robot frame.
+ Responsible for real-time navigation corrections.

### control_motors(driver, speed):
``` python
def control_motors(driver, speed):
    """
    Controls the TB6612FNG motor driver.
    Sends equal PWM speed signals to left & right motors for forward motion.
    Speed range is 0–255.
    """
    driver.set_motor_speed(speed, speed)  # equal speed for both wheels
```

### Explanation:
+ Keeps robot moving forward with a constant speed.
+ Uses PWM signals to the TB6612FNG driver.
+ If needed, can be extended to allow different left/right speeds for sharper turns.

### check_termination(forward_distance, lap_count, max_laps):
```python
def check_termination(forward_distance, lap_count, max_laps=3):
    """
    Checks if the run should terminate.
    Conditions:
    - If forward distance < threshold → obstacle or wall detected.
    - If lap_count >= max_laps → required laps completed.
    Returns True if termination condition is met.
    """
    if forward_distance < 20:  # cm threshold
        return True
    if lap_count >= max_laps:
        return True
    return False
```

### Explanation:
+ Ensures robot doesn’t run forever.
+ Two termination conditions:
  + Obstacle detected in front (arena end or unexpected block).
  + All laps completed (challenge success).
+ Returns True → robot stops.

<img src="https://github.com/KiaraBhandari8/Unsupervised-WROFE2025-India/blob/main/schemes/addl/Open_Algorithm.png" alt="Open Round Algorithm" width="500">


## Obstacle Round Algorithm
The obstacle round uses a **hybrid sensing and control system** where LiDAR and camera inputs are fused together to identify obstacles, calculate errors, and generate steering corrections. The system is structured into multiple layers: sensing, perception, control, and actuation.



### 1) LiDAR-Based Obstacle Detection
+ **Sensor**: YDLidar T-mini Plus provides a **270° scan**, but only **-90° to +90° relative to robot’s forward axis** is considered.  
+ **Preprocessing**:  
  - Filters raw data for outliers (values beyond sensor max range).  
  - Segments detected points into clusters representing potential obstacles.  
+ **Pillar Identification**:  
  - Uses expected pillar spacing to detect red/green pillar positions.  
  - Calculates **lateral offset error** = difference between current robot position and target trajectory (pass left of green, right of red).  
+ **Use Case**:  
  - Provides **fail-safe navigation** when camera detection fails (e.g., poor lighting).  
  - Supplies continuous lateral distance measurement to support stable alignment.

### 2) Camera-Based Color Detection Algorithm
The camera system adds semantic information (pillar color), which LiDAR alone cannot provide.

+ **Frame Acquisition & Preprocessing**  
  - Captures frames using **PiCamera2** at 2304×1296.  
  - Resized and blurred to reduce noise before color segmentation.  
  - Conversion **BGR → HSV** for robust color thresholding under variable lighting.  

+ **Color Segmentation**  
  - Predefined HSV thresholds for **red** and **green**.  
  - Binary masks created for each color channel.  
  - Morphological operations (erode, dilate) applied to reduce noise.  

+ **Region of Interest (ROI)**  
  - Inner ROI = normal detection zone.  
  - Outer ROI = activated when LiDAR detects an obstacle cluster ahead.  
  - Dual-ROI design reduces false positives from irrelevant background features.  

+ **Contour Detection & Validation**  
  - Extracts contours from binary masks.  
  - Rejects contours smaller than **MIN_CONTOUR_AREA = 1500**.  
  - Selects the largest contour per frame as the obstacle candidate.  
  - Assigns detection labels:  
    - `"red_obstacle"` → valid red pillar.  
    - `"green_obstacle"` → valid green pillar.  
    - `"obstacle"` → unidentified object.  
    - `"none"` → no valid obstacle.  

+ **Depth Sensitivity Factor**  
  - Implements **DEPTH_IMPORTANCE_FACTOR = 1.35**.  
  - Vertical pixel location of contour centroid (closer to bottom = nearer).  
  - Error signal scaled accordingly:  
    - Far object → low correction.  
    - Near object → stronger correction.  

+ **Steering Logic (Camera)**  
  - Red pillar → steer left with correction `steering = KP_STEERING × error × depth_factor`.  
  - Green pillar → steer right with same correction logic.  
  - Maintains flexibility for real-time corrections based on object position.  

+ **Fallback Mechanisms**  
  - If **no obstacle detected** → black region between floor lines is used for centering.  
  - If **frame underexposed** → assumes corner → performs sharp left turn (≈ -45°).  
  - Prevents robot from stalling during sensor failure.  

+ **Debugging Visual Overlays**  
  - Draws bounding boxes, centroids, ROI zones, and target lines.  
  - Displays numerical feedback (depth factor, correction angle).  
  - Useful for tuning HSV thresholds and ROI positions in real-time.  

### 3) Sensor Fusion & PID Steering Control
+ **Error Calculation**  
  - `LiDAR_error` = difference between distances to left/right wall.  
  - `Camera_error` = lateral offset of detected pillar.  
  - Fusion weights:  
    - **LiDAR dominant** if no valid pillar detected.  
    - **Camera dominant** if pillar is confirmed in ROI.  

+ **PID Controller**  
  - Input = combined error (LiDAR + Camera).  
  - **Kp (Proportional)** → quick correction response.  
  - **Ki (Integral)** → compensates for slow drift across laps.  
  - **Kd (Derivative)** → damps oscillations during sharp turns.  
  - Tuning values are selected through iterative testing for balance between stability and responsiveness.  

+ **Output**  
  - PID output mapped to **MG996R servo angle**.  
  - Motor PWM adjusted to maintain forward speed while avoiding jerks during corrections.  

### 4) Decision Flow & Execution
+ Final control decision is based on **sensor fusion state**:  
  - `"red_obstacle"` → steer left of pillar.  
  - `"green_obstacle"` → steer right of pillar.  
  - `"corner_avoid"` → perform sharp predefined turn.  
  - `"line_centering"` → maintain lane using LiDAR + ROI fallback.  

+ **Actuation**  
  - Steering angle sent to **MG996R high-torque servo**.  
  - Motor commands executed via **TB6612FNG motor driver**.  
  - Smooth velocity profiles prevent skidding or abrupt turns.  

+ **System Reliability**  
  - Fusion of LiDAR (geometric accuracy) and Camera (semantic recognition) ensures **redundancy and robustness**.  
  - Fail-safes prevent deadlock when either sensor produces invalid data.  

## Step by step breakdown of functions (obstacle round):

## Step by step breakdown of functions (obstacle round):

### robot_control_loop():
```python
def robot_control_loop():

    global output_frame, output_frame_lock, current_robot_state, latest_camera_frame, camera_frame_lock, camera_thread_stop_event
    global START_PAUSE_DURATION, previous_increment_time, turn_counter, max_turn_count, DELAY_BETWEEN_TURNS

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=False, hflip=False),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started with resolution {CAMERA_RESOLUTION} at {CAMERA_FRAMERATE} FPS.")
    
    time.sleep(1) 
    camera_acquisition_thread = threading.Thread(target=camera_acquisition_thread_func, args=(picam2, camera_thread_stop_event))
    camera_acquisition_thread.daemon = True
    camera_acquisition_thread.start()

    lidar_scanner, lidar_pid, lidar_acquisition_thread = None, None, None
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
        print("LiDAR system initialized successfully.")
    except IOError as e:
        print(f"WARNING: Failed to initialize LiDAR system: {e}.")
        lidar_scanner = None

    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    try:
        while True:
            loop_start_time = time.monotonic()
            
            with camera_frame_lock:
                if latest_camera_frame is None:
                    time.sleep(0.01) 
                    continue
                full_res_frame_rgb = latest_camera_frame.copy()

            processing_frame_rgb = cv2.resize(
                full_res_frame_rgb,
                (PROCESSING_WIDTH, PROCESSING_HEIGHT),
                interpolation=cv2.INTER_AREA
            )
            frame_bgr = cv2.cvtColor(processing_frame_rgb, cv2.COLOR_RGB2BGR)
            
            scan_data = None
            if lidar_scanner:
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()

            # Check if between walls
            between_walls = check_between_walls(scan_data, front_distance_min_threshold=1000, front_distance_max_threshold=2000,
                front_angle_range=7.5, side_angle_range=7.5, side_distance_threshold=1000, side_distance_tolerance=100)
    
            if between_walls:
                # print("----------------Between walls detected.----------------")
                if time.time() - loop_start_time > START_PAUSE_DURATION:  # Ensure initial pause
                    if time.time() - previous_increment_time > DELAY_BETWEEN_TURNS:  # Ensure cooldown between turns
                        # Only count a turn if 5 seconds have passed since the last one
                        turn_counter += 1
                        previous_increment_time = time.time()  # Reset the cooldown timer
                        print(f"Turn condition met! Executing turn {turn_counter}/{max_turn_count}.")

            print(f"--------Current turn count: {turn_counter}/{max_turn_count}--------")

            if turn_counter > max_turn_count:
                print(f"Max turn count ({max_turn_count}) reached, stopping robot.")
                robot_stop()
                time.sleep(60)
                break

            is_near_field_mode = check_front_obstacle_proximity(scan_data, distance_mm=1100)
            
            processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(
                frame_bgr,
                use_outer_roi_and_bottom_point=is_near_field_mode
            )
            vision_angle = -1*vision_angle

            if processed_frame is None:
                if STREAM_VIDEO:
                    with output_frame_lock:
                        output_frame = frame_bgr.copy()
                time.sleep(0.01)
                continue

            side_alert_status = check_lidar_side_alerts(scan_data)
            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_CRUISE_SPEED
            display_text = ""

            # BEHAVIOR ARBITRATION
            if logic_label == "red_obstacle" or logic_label == "obstacle":
                robot_speed_current = ROBOT_MANEUVER_SPEED
                if side_alert_status == "RIGHT":
                    current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                    target_servo_angle = SERVO_CENTER_ANGLE - LIDAR_SIDE_STEER_MAGNITUDE
                    display_text = "MODE: OVERRIDE | Right LiDAR!"
                elif side_alert_status == "LEFT":
                    current_robot_state = RobotState.LIDAR_SIDE_AVOIDANCE
                    target_servo_angle = SERVO_CENTER_ANGLE + LIDAR_SIDE_STEER_MAGNITUDE
                    display_text = "MODE: OVERRIDE | Left LiDAR!"
                else:
                    current_robot_state = RobotState.RED_AVOIDANCE if logic_label == "red_obstacle" else RobotState.GREEN_AVOIDANCE
                    servo_adjust = -vision_angle * STEERING_GAIN
                    print(f"Vision Angle:{round(vision_angle)} |Servo Adjust:{round(servo_adjust)}")
                    target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                    display_text = f"MODE: {'Red' if logic_label == 'red_obstacle' else 'Cam'}Avoid | Steer: {int(round(target_servo_angle))}°"
                
                target_servo_angle = int(round(np.clip(target_servo_angle, LIDAR_SERVO_MIN_ANGLE, LIDAR_SERVO_MAX_ANGLE)))

            ## --- MODIFIED ---
            elif lidar_scanner and lidar_pid:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                if scan_data:
                    # Check for the straight corridor override condition first.
                    if check_for_straight_corridor(scan_data, min_dist_mm=1750, max_dist_mm=2000):
                        target_servo_angle = SERVO_CENTER_ANGLE
                        display_text = "MODE: LiDARWF | Straight Override"
                        print("LiDAR Straight Corridor Override Activated!")
                    else:
                        # If not in a straight corridor, use the standard PID wall-following logic.
                        lidar_error = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM)
                        if lidar_error == 9999.0:
                            robot_stop()
                            current_robot_state = RobotState.STOP
                            display_text = "MODE: STOP (LiDAR Obstacle!)"
                            time.sleep(0.1)
                            continue
                        else:
                            pid_output = lidar_pid.update(lidar_error)
                            target_servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, CLOCKWISE_WALL_FOLLOWING)
                            display_text = f"MODE: LiDARWF | Steer: {round(target_servo_angle)}° | Err: {lidar_error:.0f}mm"
                else:
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE
                    display_text = "MODE: Fallback (No LiDAR Data)"
            else:
                robot_speed_current = ROBOT_CRUISE_SPEED
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                display_text = f"MODE: Fallback | Logic: {logic_label}"

            # APPLY ROBOT MOTION
            if current_robot_state != RobotState.STOP:
                adjust_servo_angle(target_servo_angle)
                if (target_servo_angle <= 70) or (target_servo_angle >= 110):
                    robot_forward_speed(ROBOT_SPEED_MAX)
                    print(f"Robot Speed : {ROBOT_SPEED_MAX}")
                else:
                    robot_forward_speed(robot_speed_current)
                    print(f"Robot Speed : {robot_speed_current}")
            else:
                robot_stop()

            # UI OVERLAYS
            if DEBUG_UI_OVERLAYS:
                loop_duration = time.monotonic() - loop_start_time
                fps = 1.0 / loop_duration if loop_duration > 0 else 0
                print(f"Frames Processed Per Second (FPS): {int(fps)}")
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(processed_frame, f"FPS: {int(fps)}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if STREAM_VIDEO:
                with output_frame_lock:
                    output_frame = processed_frame.copy()

    finally:
        print("Control loop ending...")
        camera_thread_stop_event.set()
        camera_acquisition_thread.join() 
        robot_stop()
        picam2.stop()
        print("Robot, camera, and LiDAR resources released.")

```

### Explanation:
1) Camera setup:
The function first starts the Pi camera, setting it to a resolution and framerate you’ve defined. It then launches a separate thread that continuously captures frames. This way, your robot always has the most recent camera image ready for processing.

2) LiDAR setup:
It tries to connect to the LiDAR sensor, which is used to detect distances and obstacles around the robot. If it works, another thread is started to keep grabbing LiDAR scan data. A PID controller is also created — this helps adjust steering smoothly when wall-following. If the LiDAR fails, the robot just defaults to a simpler straight-line mode.

3) Picking the starting mode:
If LiDAR is available, the robot begins in wall-following mode. If not, it starts in fallback mode where it just goes straight.

4) The infinite control loop:
Now the robot enters its main loop, which never stops until the function ends. In each cycle it:
+ Gets the latest camera frame and shrinks it to a smaller size for faster processing.
+ Gets the latest LiDAR scan data if the LiDAR is running
+ Checks if it’s between walls (like in a corridor). If it is, it counts how many turns it makes and eventually stops if too many turns are done.
+ Processes the camera frame to figure out if there are obstacles (red/green) or a line/path to follow. This also calculates a steering correction angle.
  
5) Behavior arbitration:
This is the “decision-making” part. The robot decides which mode to prioritize:
If a red or green obstacle is detected, it slows down and steers away using either vision or LiDAR side alerts.
If LiDAR is active and there are walls to follow, it uses either a straight corridor override (keep straight if path is clear) or the PID-based steering (hug a wall smoothly). If no LiDAR is available, it just drives straight based on vision fallback.

6) Motion control:
Once the decision is made, the robot adjusts its servo steering angle and motor speed. If it’s in stop mode, everything halts. Otherwise, it drives forward, sometimes faster if turning sharply.

7) Debugging overlays:
If debugging is turned on, the robot adds text overlays on the camera feed with things like the current mode, steering angle, and frame rate.

8) Shutdown:
If the loop ends (like from an error or stop condition), the robot shuts down cleanly: it stops the camera thread, halts the motors, and releases LiDAR and camera resources.

### check_for_straight_corridor(scan_data, min_dist_mm=1750, max_dist_mm=2000):
```python
def check_for_straight_corridor(scan_data, min_dist_mm=1750, max_dist_mm=2000):
    """
    Checks if the robot is facing a straight path by analyzing front LiDAR data.
    If the average distance in front-left and front-right sectors are both
    within a specific range, it signals to go straight.

    Args:
        scan_data (dict): The LiDAR scan data {angle: distance}.
        min_dist_mm (int): The minimum average distance for the condition.
        max_dist_mm (int): The maximum average distance for the condition.

    Returns:
        bool: True if the straight corridor condition is met, False otherwise.
    """
    if not scan_data:
        return False

    left_front_distances = []
    right_front_distances = []

    # Collect distances for left-front (-5 to <0 deg) and right-front (0 to 5 deg)
    angle_range_max = 10
    for angle, distance in scan_data.items():
        if -1*angle_range_max <= angle < 0 and distance > 0:
            left_front_distances.append(distance)
        elif 0 <= angle <= angle_range_max and distance > 0:
            right_front_distances.append(distance)

    # Ensure we have readings in both sectors to make a valid comparison
    if not left_front_distances or not right_front_distances:
        return False

    # Calculate the average distance for each sector
    avg_left_dist = sum(left_front_distances) / len(left_front_distances)
    avg_right_dist = sum(right_front_distances) / len(right_front_distances)
    
    # Check if BOTH averages fall within the specified range
    is_left_in_range = min_dist_mm < avg_left_dist < max_dist_mm
    is_right_in_range = min_dist_mm < avg_right_dist < max_dist_mm

    return is_left_in_range and is_right_in_range
```

### Explanation:
Purpose:
The function looks at LiDAR scan data in the front-left and front-right sectors.
If both sides report that the space in front is open and roughly the same distance away (within a certain range), then it assumes the robot is facing a straight corridor.

Step-by-step:
+ Input data
  + It takes a dictionary of LiDAR data, where each entry is {angle: distance}.
  + Example: {-5: 1800, 0: 1900, 5: 1850, ...}

+ Check if there’s scan data
  + If no LiDAR data is available, it immediately returns False.

+ Separate the angles
  + Splits the LiDAR readings into two groups:
    + Left front sector: angles between about –10° and 0°
    + Right front sector: angles between 0° and +10°
  + Only keeps distances that are valid (greater than zero).

+ Make sure both sides have data
  + If one side doesn’t have any valid distances, it can’t compare → returns False.

+ Calculate averages
  + Finds the average distance for the left side and the right side.

+ Check the range
  + If both averages fall between the min_dist_mm and max_dist_mm values, it means both sides agree the corridor is open and straight.
  + Otherwise, returns False.

+ What it returns
  + True → The robot is facing a straight corridor (good to go straight).
  + False → The path isn’t straight or the data isn’t good enough.


### process_frame_for_steering(frame, use_outer_roi_and_bottom_point=False):
```python
def process_frame_for_steering(frame, use_outer_roi_and_bottom_point=False):
    """
    Main vision pipeline:
    - Detects red/green obstacles in ROI
    - Applies depth-aware steering adjustment
    - Falls back to black line detection if no obstacle
    - Returns processed frame (for debugging), steering_angle, mask, label
    """

    # Handle empty input
    if frame is None or frame.size == 0:
        return frame, 0, None, "none"

    # Convert to HSV for color segmentation
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for green and red obstacles
    green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    red_mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    red_mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # ROI selection
    if use_outer_roi_and_bottom_point:
        roi = frame[OUTER_ROI_Y1:OUTER_ROI_Y2, OUTER_ROI_X1:OUTER_ROI_X2]
        roi_red = red_mask[OUTER_ROI_Y1:OUTER_ROI_Y2, OUTER_ROI_X1:OUTER_ROI_X2]
        roi_green = green_mask[OUTER_ROI_Y1:OUTER_ROI_Y2, OUTER_ROI_X1:OUTER_ROI_X2]
    else:
        roi = frame[INNER_ROI_Y1:INNER_ROI_Y2, INNER_ROI_X1:INNER_ROI_X2]
        roi_red = red_mask[INNER_ROI_Y1:INNER_ROI_Y2, INNER_ROI_X1:INNER_ROI_X2]
        roi_green = green_mask[INNER_ROI_Y1:INNER_ROI_Y2, INNER_ROI_X1:INNER_ROI_X2]

    # Find contours in red/green masks
    contours_red, _ = cv2.findContours(roi_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(roi_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    steering_angle = 0
    logic_label = "none"

    # Determine largest obstacle contour
    obstacle_contour = None
    color_detected = None
    for c in contours_red:
        if cv2.contourArea(c) > MIN_CONTOUR_AREA:
            obstacle_contour = c
            color_detected = "red"
            break
    for c in contours_green:
        if cv2.contourArea(c) > MIN_CONTOUR_AREA:
            obstacle_contour = c
            color_detected = "green"
            break

    if obstacle_contour is not None:
        # Compute bounding box
        x, y, w, h = cv2.boundingRect(obstacle_contour)
        obstacle_center = x + w // 2
        obstacle_bottom_y = y + h

        # Depth sensitivity: closer objects → stronger corrections
        normalized_depth = 1 - (obstacle_bottom_y / frame.shape[0])
        depth_factor = DEPTH_IMPORTANCE_FACTOR * (1 + normalized_depth)

        # Calculate steering correction
        error = (obstacle_center - (roi.shape[1] // 2))
        steering_angle = KP_STEERING * error * depth_factor

        # Assign logic label
        if color_detected == "red":
            logic_label = "red_obstacle"
        else:
            logic_label = "obstacle"

        # Debug drawing
        cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(frame, f"DepthFactor: {depth_factor:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        # Fallback: use black line detection
        correction = analyze_black_between_lines(frame, (INNER_ROI_X1, INNER_ROI_Y1), (INNER_ROI_X2, INNER_ROI_Y2))
        if correction is not None:
            steering_angle = correction
            logic_label = "line_centering"
        else:
            # Dark corner → assume sharp left turn
            steering_angle = -45
            logic_label = "corner_avoid"

    return frame, steering_angle, (red_mask, green_mask), logic_label
```

### Explanation:
+ Input checks
  + If frame is empty → return defaults.

+ Color filtering
  + HSV thresholds for green obstacles and red obstacles.
  + Masks created → isolate obstacles by color.

+ Region of Interest (ROI) selection
  + Two ROI modes:
    + Inner ROI (default) → normal detection zone.
    + Outer ROI + bottom detection point (when close to obstacles).

+ Contour detection
  + Finds contours in red/green masks.
  + Filters by MIN_CONTOUR_AREA to ignore noise.
  + Chooses the largest obstacle contour.

+ Obstacle classification
  + If red → "red_obstacle" label.
  + If green → "obstacle" label.
  + If none → "none" (fallback logic).

+ Depth-aware steering
  + Uses bottom Y-coordinate of obstacle (obstacle_bottom_y).
  + Normalizes relative to frame height → closer objects give larger values.
  + Multiplies steering correction by DEPTH_IMPORTANCE_FACTOR × normalized_depth.
  + Result: closer obstacles cause sharper steering corrections.

+ Fallback to black line centering
  + If no obstacle detected:
    + Calls analyze_black_between_lines().
    + If even that fails, assumes dark corner → hard left turn (-45°).

+ UI overlays
  + Draws rectangles, centerlines, contours.
  + Displays Depth Factor for debugging.

## analyze_black_between_lines
```python
def analyze_black_between_lines(frame, inner_start, inner_end):
    # This function remains unchanged
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x1, y1 = inner_start
    x2, y2 = inner_end
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
    if x1 >= x2 or y1 >= y2:
        return None
    roi = gray[y1:y2, x1:x2]
    _, black_mask = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)
    h, w = black_mask.shape
    left = black_mask[:, :w // 2]
    right = black_mask[:, w // 2:]
    black_left = np.sum(left) / 255
    black_right = np.sum(right) / 255
    total_black = black_left + black_right
    if total_black == 0:
        return None
    balance = (black_right - black_left) / total_black
    correction = KP_LINE_CENTERING * balance * 100
    return correction
```
## Explanation:
Purpose:
+ This function looks at a section of the camera frame (a region of interest, or ROI), checks how much black is on the left vs. the right, and then gives back a correction value.

Step-by-step:
1) Convert to grayscale:
The frame is turned into black-and-white shades so it’s easier to detect dark areas.

2) Define the region of interest (ROI):
The function takes two points (inner_start and inner_end) that form a rectangle. That rectangle is cut out of the image to focus only on the middle area where lines are expected. If the rectangle is invalid (like zero or negative width/height), the function just quits.

3) Thresholding to find black:
Inside that ROI, it applies a threshold so that black areas turn into white (on the mask) and everything else becomes black. In other words, it builds a binary mask that highlights black regions.

4) Split into left and right halves:
The ROI is divided down the middle: one half for the left side, one half for the right side.

5) Count black pixels
It sums up how many black pixels are detected on the left and the right. Then it adds them together for the total.

6) Handle empty case:
If no black is detected at all, the function returns None (meaning: no correction info available).

7) Balance calculation:
+ It calculates the difference between right-black and left-black, divided by the total.
  + If black is perfectly balanced left vs. right → result is 0 (no correction needed).
  + If more black is on the right → balance is positive.
  + If more black is on the left → balance is negative.

8) Correction scaling:
This balance value is multiplied by a gain constant (KP_LINE_CENTERING) and scaled up. The result is the steering correction value.

What it returns:
<br>
A numeric correction that tells the robot how much to adjust its steering to stay centered between lines. None if no black was detected.

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
      <td>40 + 22 (for parking)</td>
      <td>40 + 7</td>
      <td>150</td>
    </tr>
    <tr>
    </tr>
  </tbody>
</table>

## 🚀 Robot Construction Guide

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
   ```python
   sudo apt update && sudo apt upgrade -y
   sudo apt install python3-pip i2c-tools
   pip3 install adafruit-circuitpython-servokit rplidar opencv-python

Enable I²C and camera support:
sudo raspi-config


📥 Step 6: Upload the Code
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




### 📝 Raspberry Pi 5 Initialization Steps

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
``` python
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

  5. Install Libraries to Run Codes:

  pip3 install (install all of these in these versions to run the code smoothly)
    Package                            Version
  ---------------------------------- ----------
  asgiref                            3.6.0
  astroid                            2.14.2
  asttokens                          2.2.1
  attrs                              22.2.0
  av                                 12.3.0
  Babel                              2.10.3
  beautifulsoup4                     4.11.2
  blinker                            1.5
  certifi                            2022.9.24
  chardet                            5.1.0
  charset-normalizer                 3.0.1
  click                              8.1.3
  colorama                           0.4.6
  colorzero                          2.0
  cryptography                       38.0.4
  cupshelpers                        1.0
  dbus-python                        1.3.2
  dill                               0.3.6
  distro                             1.8.0
  docutils                           0.19
  Flask                              2.2.2
  gpiozero                           2.0.1
  html5lib                           1.1
  idna                               3.3
  importlib-metadata                 4.12.0
  isort                              5.6.4
  itsdangerous                       2.1.2
  jedi                               0.18.2
  Jinja2                             3.1.2
  jsonpointer                        2.3
  jsonschema                         4.10.3
  lazy-object-proxy                  1.9.0
  lgpio                              0.2.2.0
  libarchive-c                       2.9
  libevdev                           0.5
  logilab-common                     1.9.8
  lxml                               4.9.2
  MarkupSafe                         2.1.2
  mccabe                             0.7.0
  meson                              1.5.1
  more-itertools                     8.10.0
  mypy                               1.0.1
  mypy-extensions                    0.4.3
  numpy                              1.24.2
  oauthlib                           3.2.2
  olefile                            0.46
  parso                              0.8.3
  pexpect                            4.8.0
  pgzero                             1.2
  picamera2                          0.3.30
  pidng                              4.0.9
  piexif                             1.1.3
  pigpio                             1.78
  Pillow                             9.4.0
  pip                                23.0.1
  platformdirs                       2.6.0
  psutil                             5.9.4
  ptyprocess                         0.7.0
  pycairo                            1.20.1
  pycryptodomex                      3.11.0
  pycups                             2.0.1
  pygame                             2.1.2
  Pygments                           2.14.0
  PyGObject                          3.42.2
  pyinotify                          0.9.6
  PyJWT                              2.6.0
  pylint                             2.16.2
  PyOpenGL                           3.1.6
  pyOpenSSL                          23.0.0
  PyQt5                              5.15.9
  PyQt5-sip                          12.11.1
  pyrsistent                         0.18.1
  pyserial                           3.5
  pysmbc                             1.0.23
  python-apt                         2.6.0
  python-dotenv                      0.21.0
  python-prctl                       1.8.1
  pytz                               2022.7.1
  pyudev                             0.24.0
  reportlab                          3.6.12
  requests                           2.28.1
  requests-oauthlib                  1.3.0
  responses                          0.18.0
  rfc3987                            1.3.8
  roman                              3.3
  rpi-lgpio                          0.6
  RTIMULib                           7.2.1
  Send2Trash                         1.8.1b0
  sense-hat                          2.6.0
  setuptools                         66.1.1
  simplejpeg                         1.8.1
  simplejson                         3.18.3
  six                                1.16.0
  smbus2                             0.4.2
  soupsieve                          2.3.2
  spidev                             3.5
  ssh-import-id                      5.10
  thonny                             4.1.4
  toml                               0.10.2
  tomlkit                            0.11.7
  tqdm                               4.64.1
  twython                            3.8.2
  types-aiofiles                     22.1
  types-annoy                        1.17
  types-appdirs                      1.4
  types-aws-xray-sdk                 2.10
  types-babel                        2.11
  types-backports.ssl-match-hostname 3.7
  types-beautifulsoup4               4.11
  types-bleach                       5.0
  types-boto                         2.49
  types-braintree                    4.17
  types-cachetools                   5.2
  types-caldav                       0.10
  types-certifi                      2021.10.8
  types-cffi                         1.15
  types-chardet                      5.0
  types-chevron                      0.14
  types-click-spinner                0.1
  types-colorama                     0.4
  types-commonmark                   0.9
  types-console-menu                 0.7
  types-contextvars                  2.4
  types-croniter                     1.3
  types-cryptography                 3.3
  types-D3DShot                      0.1
  types-dateparser                   1.1
  types-DateTimeRange                1.2
  types-decorator                    5.1
  types-Deprecated                   1.2
  types-dj-database-url              1.0
  types-docopt                       0.6
  types-docutils                     0.19
  types-editdistance                 0.6
  types-emoji                        2.1
  types-entrypoints                  0.4
  types-first                        2.0
  types-flake8-2020                  1.7
  types-flake8-bugbear               22.10.27
  types-flake8-builtins              2.0
  types-flake8-docstrings            1.6
  types-flake8-plugin-utils          1.3
  types-flake8-rst-docstrings        0.2
  types-flake8-simplify              0.19
  types-flake8-typing-imports        1.14
  types-Flask-Cors                   3.0
  types-Flask-SQLAlchemy             2.5
  types-fpdf2                        2.5
  types-gdb                          12.1
  types-google-cloud-ndb             1.11
  types-hdbcli                       2.14
  types-html5lib                     1.1
  types-httplib2                     0.21
  types-humanfriendly                10.0
  types-invoke                       1.7
  types-JACK-Client                  0.5
  types-jmespath                     1.0
  types-jsonschema                   4.17
  types-keyboard                     0.13
  types-ldap3                        2.9
  types-Markdown                     3.4
  types-mock                         4.0
  types-mypy-extensions              0.4
  types-mysqlclient                  2.1
  types-oauthlib                     3.2
  types-openpyxl                     3.0
  types-opentracing                  2.4
  types-paho-mqtt                    1.6
  types-paramiko                     2.11
  types-parsimonious                 0.10
  types-passlib                      1.7
  types-passpy                       1.0
  types-peewee                       3.15
  types-pep8-naming                  0.13
  types-Pillow                       9.3
  types-playsound                    1.3
  types-polib                        1.1
  types-prettytable                  3.4
  types-protobuf                     3.20
  types-psutil                       5.9
  types-psycopg2                     2.9
  types-pyaudio                      0.2
  types-PyAutoGUI                    0.9
  types-pycurl                       7.45
  types-pyfarmhash                   0.3
  types-pyflakes                     2.5
  types-Pygments                     2.13
  types-pyinstaller                  5.6
  types-PyMySQL                      1.0
  types-pynput                       1.7
  types-pyOpenSSL                    22.1
  types-pyRFC3339                    1.1
  types-PyScreeze                    0.1
  types-pysftp                       0.2
  types-pytest-lazy-fixture          0.6
  types-python-crontab               2.6
  types-python-dateutil              2.8
  types-python-gflags                3.1
  types-python-jose                  3.3
  types-python-nmap                  0.7
  types-python-slugify               6.1
  types-pytz                         2022.6
  types-pyvmomi                      7.0
  types-pywin32                      304
  types-PyYAML                       6.0
  types-redis                        4.3
  types-regex                        2022.10.31
  types-requests                     2.28
  types-retry                        0.9
  types-Send2Trash                   1.8
  types-setuptools                   65.5
  types-simplejson                   3.17
  types-singledispatch               3.7
  types-six                          1.16
  types-slumber                      0.7
  types-SQLAlchemy                   1.4.43
  types-stdlib-list                  0.8
  types-stripe                       3.5
  types-tabulate                     0.9
  types-termcolor                    1.1
  types-toml                         0.10
  types-toposort                     1.7
  types-tqdm                         4.64
  types-tree-sitter                  0.20
  types-tree-sitter-languages        1.5
  types-ttkthemes                    3.2
  types-typed-ast                    1.5
  types-tzlocal                      4.2
  types-ujson                        5.5
  types-urllib3                      1.26
  types-vobject                      0.9
  types-waitress                     2.1
  types-whatthepatch                 1.0
  types-xmltodict                    0.13
  types-xxhash                       3.0
  types-zxcvbn                       4.4
  typing_extensions                  4.4.0
  uritemplate                        4.1.1
  urllib3                            1.26.12
  v4l2-python3                       0.3.5
  videodev2                          0.0.4
  webcolors                          1.11.1
  webencodings                       0.5.1
  Werkzeug                           2.2.2
  wheel                              0.38.4
  wrapt                              1.14.1
  zipp                               1.0.0
  
  ```

  6. (Optional) VS Code Remote Setup
  To program directly from your laptop using Visual Studio Code:
  Install the Remote SSH extension in VS Code.
  Connect to the Pi using its IP address:
  ssh pi@<your_pi_ip>
  Default username: pi
  Default password: raspberry (change this after first login with passwd).
  

 ### Conclusion
 
This project document captures the entire journey of designing, building, and testing the autonomous robot for the WRO Future Engineers 2025 challenge. The integration of mechanical innovation, sensor technology, and robust algorithms demonstrates the team’s commitment to problem-solving and hands-on learning in robotics. The solutions, improvements, and insights gained here not only enrich our experience but serve as a valuable guide for anyone interested in robotics or autonomous systems. We hope this repository inspires future participants and contributors to explore, experiment, and build upon our work for even greater achievements in the years ahead. 

 



































