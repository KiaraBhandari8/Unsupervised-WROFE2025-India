@ -1,711 +0,0 @@
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

## Robot Pictures
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

### Mobility Management

The robot's mobility is managed by a combination of compoenents, including the powertrain, steering system and chassis. These elements work together to ensure the robot's smooth and efficient movement. 

#### Powertrain 

##### Drivetrain 
To minimize friction and therby reduce speed loss, driving axle was made from an steel rod. We connected the orot to the riving axle using a custom 3-D printed adapter. The driing axle is equipped with differential gears, which allow the robot to turn smoothly. For traction we used N20 wheels. 

Potential Improvements:

- Upgrade to higher-quality differential gear set for enhanced efficiency and smoother cornering. 
- Experiment with lighter alloys and hollow rods to balance strength with reduced weight. 
- Replace the 3D printed adapter with a kachined metal one for greater durability and precision. 

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

Where to buy the motor: (insert link)

To connect the motor's axle to the chassis, we created a custom 3D - printed adapter. 


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
    Motor Voltage (VM): 4.5–13.5 V
Logic Voltage (VCC): 2.7–5.5 V
Output Current: 1.2 A continuous, 3.2 A peak per channel
Channels: Dual H-Bridge (2 DC motors / 1 stepper)
PWM Frequency: Up to 100 kHz
Control Pins: Direction, PWM speed, Standby
Protections: Thermal shutdown, overcurrent, undervoltage
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



### Build


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

#### Open Round Algorithm

+ A LiDAR sensor is used to detect arena walls and maintain a stable, centered path.
+ It provides distance readings from -90° (left) to +90° (right), with 0° aligned forward.
+ The robot calculates the distance to walls on both sides and determines an error value as the difference.
+ This error is fed into a PID controller, which adjusts the steering angle to minimize deviation.
+ The Kp, Ki, and Kd gains are tuned to balance responsiveness and stability, avoiding oscillation or delay.
+ The same control principle applies during turns, ensuring smooth and accurate maneuvering.
+ This approach enables consistent and precise navigation throughout the open round.

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








