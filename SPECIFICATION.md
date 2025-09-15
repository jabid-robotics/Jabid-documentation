**Jabid Robotic Arm - Project Specification (V0.1)**
*  A Modular, Open-Source, Precision Robotic Arm Platform

**Version: 0.1 (Pre-Release, Developmental)**
**Status: Active Development**

**1. Vision**

*  To create a modular, open-source, and industry-grade robotic arm platform that bridges the gap between hobbyist projects and industrial automation. The arm will feature a layered control architecture for high precision and culminate in an accessible AI-powered learning module.

**2. Mechanical Specification (Target for V1.0)**

*  Degrees of Freedom: 4 DOF + Gripper
*  Joint 1: Base Rotation (360°)
*  Joint 2: Shoulder Pitch (0-180°)
*  Joint 3: Elbow Pitch (0-180°)
*  Joint 4: Wrist Pitch (0-180°)
*  Gripper: Parallel Jaw
*  Target Payload: 1500 g
*  Target Repeatability: ± 1.5 mm
*  Primary Structure: 3D Printed (PLA/PETG) with metal reinforcement and bearings.
*  Actuation: Dynamixel XL330-series smart servos (or equivalent with position feedback).
*  Connectors: JST GH for all inter-joint connections.
*  Sensing: MPU-6050 (or MPU-9250) IMU on key links (forearm, upper arm) for gravity compensation and vibration damping.

**3. Electrical Specification (Target for V1.0)**

*  Main Brain: Raspberry Pi 4/5 (4GB+ recommended).
*  Real-Time Controller: STM32F4 series microcontroller (e.g., Nucleo STM32F446).
*  Power System: 12V DC input, distributed to servos and regulators for 5V (Pi) and 3.3V (STM32, IMU).
*  Communication:
*  Pi-to-STM32: UART (Serial).
*  STM32-to-IMU: I2C.
*  STM32-to-Servos: UART (Dynamixel protocol) or normal GPIO (if no position feedback).
*  Wiring: Daisy-chained cabling for power and data.

**V0.1 Approach: Use generic STM32 dev board and soldered perfboard/prototype shield.**

**4. Control Architecture (Target for V1.0)**

*  The system uses a layered software architecture to separate high-level planning from real-time control.
*  Layer	Hardware	Software	Function
*  High-Level Planning	Raspberry Pi	ROS 2 (Jazzy)
   *  jabid_ik
   *  jabid_control	Calculates Inverse Kinematics, trajectory planning, user API, and sends joint setpoints to the STM32.
*  Real-Time Control	STM32F4	Custom Firmware (C/C++)
   *  PID Control Loops
   *  Complementary Filter	Executes tight loops for each joint: reads encoders & IMU, calculates PID output, and sends commands to servos. Handles safety.
*  Hardware Interface	Smart Servos, IMU	Dynamixel SDK, I2C HAL	Provides low-level actuation and sensing.
*  Key Feature: IMU Integration. The IMU data is fused on the STM32 to provide:
*  Gravity Compensation: Adjusts torque output based on arm pose.
*  Vibration Damping: Actively reduces oscillation after movement.
*  Enhanced Precision: Corrects for positional drift and load-induced sag.

**5. Software Specification**

*  Framework: ROS 2 (Jazzy)
*  Core Packages:
*  jabid_description: Contains the URDF model of the arm.
*  jabid_control: ROS 2 node that manages the serial communication protocol with the STM32.
*  jabid_ik: Inverse Kinematics solver library.
*  jabid_moveit_config: Configuration for MoveIt 2 for motion planning (Future).
*  Simulation: Full compatibility with Gazebo Ignition.
*  Firmware: Custom STM32 code written in C/C++ using STM32CubeIDE and HAL libraries.

**6. Milestones & Roadmap**

*  V0.1 (Current): Functional 3DOF arm. STM32 firmware development and PID tuning with IMU on prototype board. Basic ROS 2 communication.
*  V0.5 (Alpha): 4DOF arm integrated with Raspberry Pi. All core software features implemented and tested.
*  V1.0 (Initial Release): Full system stable and documented. Release includes source, CAD, and instructions for prototype build.
*  V1.1 (PCB Release): Custom PCB design released, replacing prototype wiring.
*  V1.5 (AI Upgrade): AI module using RL in simulation (Gazebo) with Sim2Real transfer.

**7. Interface Definitions**

*  Mechanical: Standard 3-bolt pattern with 30mm hole spacing for module attachment.
*  Electrical: JST-GH-4 Pin Connectors for all inter-module connections (PWR, GND, Data+, Data-).
*  Software: ROS 2 sensor_msgs/JointState and std_msgs/Float64MultiArray as primary command interfaces.

**License**

*  This project is open-source and is currently under the MIT License. See the LICENSE file in the repository for details.
  
**Contributing**

*  Contributions are welcome! Please fork the repository and submit a Pull Request for any improvements. Please follow the coding and documentation standards outlined in the CONTRIBUTING.md file (to be created).
*  This document is the single source of truth for the Jabid Robotic Arm project. It will be updated as the project evolves.
