# Scout - Autonomous UAV Project
This repository contains all software related to the autonomous UAV project dubbed Scout.

Scout is a quad rotor UAV capable of autonomous navigation in small spaces without the use of GNSS signals.
This is accomplished through a combination of SLAM and accelerometer positioning.

Each subdirectory of this repository contains a README file explaining its contents.


## Software Stack
The software stack consists of ROS nodes in combination with some embedded software.

### ROS
ROS handles the bulk of the processing workload including SLAM and obstacle avoidance.

ROS Node descriptions are located under `src`.
Further documentation is present both within each package/node and under the `docs` folder.

### INAV
The INAV project was used as the firmware for the flight controller.
ROS communicates with INAV through the MSP protocol.

## Hardware Stack
The hardware stack is composed of the drone, Jetson, and various sensors.

### Drone
 - SpeedyBeeF405 V4 FC
 - SpeedyBee 65A 3-6S ESC
 - 10" Frame
 - 800KVA Motors

### Jetson
 - Jetson Nano
 - Quad Core Cortex-A57
 - Nvidia Maxwell GPU (256 Cuda Cores)
 - Ubuntu 22.04.5 LTS "Jammy Jellyfish"

