# To-Do

## Hardware
- [ ] Mount for rear vertical LiDAR
- [ ] Mount for top horizontal LiDAR
- [ ] Mount for Intel RealSense
- [ ] Mount for Nvidia Jetson
- [ ] Mount for Battery
- [ ] Mount for Bottom Sonar
- [ ] Mount for Top Sonar
- [ ] Mounting for Sensor PCB

## Electrical
- [ ] Jetson connection to Flight Controller
    - [ ] MSP Connection (USART)
    - [ ] CRSF Connection (UART)
- [ ] Bottom Sonar Connection to Flight Controller
- [ ] Sensor Board?? (Unknown if will be board or just wiring)
    - [ ] Jetson Power
    - [ ] RealSense Power
    - [ ] LiDAR UART Connections (2x, Top/Bottom)
    - [ ] Top Sonar Connection
    - [ ] Battery Cell Monitor
    - [ ] USB Connection to Jetson

## Software
### Navigation
- [ ] Exploration Algorithm
    - [ ] Research on robots exploring unknown environments

### Mapping + Localization
- [ ] Research Intel SLAM
- [ ] Implement SLAM
    - [ ] Integration with exploration algorithm
    - [ ] Saving map
    - [ ] Data formatting for presentation

### Obstacle Avoidance
- [x] Detecting surroundings with LiDAR
    - [x] Polling LiDAR data
    - [x] Publishing LiDAR data in an organized format
- [ ] Collision Avoidance Algorithm (Gravity based planning)
    - [x] Gathering collision data
    - [x] Summation and normalization of forces
    - [x] Resultant Force Calculations
    - [ ] Provide output to the Flight Controller
    - [ ] Testing

### Drone Connection
- [x] Configure Drone with FC Software (INAV)
    - [x] Initial Calibration
- [ ] Communication between drone FC and Jetson
    - [x] Test control over MSP
    - [ ] Test control over CRSF

