# Proposed Architecture
The following document outlines one possible architecture for the autonomous UAV as presented in [Project Specifications](./spec.md)

Each header within this document specifies the functions that a particular device will be responsible for completing.
The headers are organized from top to bottom in terms of information flow within the stack.

## ROS/HLS
- Runs on the Jetson
- Localization/Mapping  
    - Intel RealSense + SLAM
- Exploration Algorithm
    - Graph Based exploration algorithm that outputs target coordinates or velocities
- Positional Controller
    - Works with/for exploration algorithm (Could be combined)
    - Turns coordinates to velocities for where to move


## Jetson
- Runs ROS software
- Connects to CIB (Control Interface Board) over USB
- Connects to Intel RealSense over USB

## CIB (Control Interface Board)
- STM32 PCB
- Connects to LiDARs over UART
- Connects to Jetson/ROS over USB
    - Passes LiDAR data back over USB CDC-ACM(If needed for visualization)
    - Exposes custom CDC-ACM interface for directional control
- Handles Gravitational Control Algorithm/Obstacle Avoidance
    - Receives velocity command from higher level software
    - Uses LiDAR Data to avoid collisions
    - Manages throttle compensation to maintain altitude
- Connects to FC over MSP or CRSF or both
- Additional PCB Functionality
    - Possibly performs some kind of battery sensing/management
    - Provides a 5V5A power source for the Jetson
    - Connection for GPS (Future Use, Extra Serial always good)

## FC (Flight Controller)
- Receives stick commands over MSP or CSRF from the CIB
- Handles low level flight kinematics/control
- Interfaces with the ESC (DSHOT or Bidirectional DSHOT)

