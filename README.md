# Hybrid-Quadruped-Robots

As part of our minor project during the Bachelor's degree in Mechatronics Engineering at NMIMS University, Mumbai, our team developed a quadruped robot simulation in 2020. 

### Team Members:
1. Miheer Diwan
2. Shubhankar Kulkarni
3. Khojasteh Mirza

### Summary:
- A hybrid adaptive(height-altering) quadruped robot with dual locomotion capabilities (wheeled and legged) was designed and simulated in CoppeliaSim. 
- This project aimed to enhance mobility and versatility by integrating both locomotion modes, allowing the robot to navigate diverse terrains effectively. 
- The robot was designed in SolidWorks, and a URDF (Unified Robot Description Format) model was created for it. This URDF model was imported into the CoppeliaSim environment, where various sensors and robot dynamics were integrated using the simulation environment’s API. This setup allowed for comprehensive simulation and testing of the robot’s performance.
- The controls for the robot are based on a simple PID controller and were tuned using MATLAB.

Please refer to the [Report](Report.pdf) for more details.

## Files:
- ```walker.py```: Main script for simulating the quadruped robot in CoppeliaSim (using ZMQ Remote API). It implements a diagonal trot gait for walking and runs the simulation file arnold_walk.ttt. Features include leg trajectory control, inverse kinematics for pose constraints, and wheel switching. Note: Some functionality needs implementation, such as increasing joint positions to elevate the robot and engaging the wheels to roll forward in wheel mode.
- ```elevationBot.py```: Older version of the code for an elevation quadruped robot (using legacy V-REP API). It allows the robot to increase/decrease its height based on obstacle height detected by proximity sensors. The mechanism uses motors and servos for elevation, with partial implementation of forward movement and gait for locomotion.

## Usage:
- For ```walker.py```: Requires CoppeliaSim installed. Load ```arnold_walk.ttt``` and run the script to simulate walking.
- ```arnold_wheels.ttt``` is a similar file, with the demonstration of morphing the wheels.
- For ```elevationBot.py```: Connect to V-REP server at 127.0.0.1:19997. Runs sensor-based height adjustment and basic movement.


## Tools Used:
1. Python
2. CoppeliaSim
3. OpenCV
4. MATLAB/Simulink
5. Solidworks
6. RaspberryPi
   

## CAD Design:
<p align="center">
  <img src="https://github.com/user-attachments/assets/a62aace6-de75-49bd-804e-b484a971ae47" width="300" />
  <img src="https://github.com/user-attachments/assets/41eb1224-e317-4c7e-b9fa-efcc877a7ab5" width="300" />
</p>


## Outcome:


https://github.com/user-attachments/assets/9b03ef7b-b2b3-48d1-999a-3ad01adec55b


https://github.com/user-attachments/assets/efa73c81-12f5-4534-9cf2-783d61e0e6ac



https://github.com/user-attachments/assets/d6bb64bf-e992-4820-a34a-0680039ad129




