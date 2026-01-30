# Scout - Autonomous UAV Project

Scout is a quad rotor UAV capable of autonomous navigation in small spaces without the use of GNSS signals.
This is accomplished using a combination of SLAM and low level control algorithms for local and global positioning.


## Repo Organization
The base level of this repo is designed to follow the format of a ROS2 Project.
All code is encapsulated within ROS nodes within the [`src`](./src) folder.
Documentation can found under [`docs`](./docs).
Scripts, including build and launch scripts, can be found under [`scripts`](./scripts), while Python Launch files can be found under [`launch`](./launch).
Finally, the PCB design is available under [`PCB_Scout`](./PCB_Scout), and embedded code is placed under [`src/control_board`](./src/control_board/firmware)


## Building

### Dependencies
- ROS2 Humble
- arm-none-eabi-gcc
- CLANG/CLANG-15
- RViZ
- [Liftoff Steam Game](https://store.steampowered.com/app/410340/Liftoff_FPV_Drone_Racing/) (for running the simulation)

### Building ROS Nodes
The ROS build is handled by the [`build.sh`](./scripts/build.sh) script and the top level `Makefile`.
To build the ROS nodes, run:
```sh
make
```

To generate a clean build, run:
```sh
make clean all
```

#### Adding to the build
By default, the ROS build system is only configured to build the specified nodes for the current target system, set by the system `HOSTNAME`.
Package names listed under `PKGS_COMMON` will be built on all systems.
Some packages are platform specific, and thus are specified under `PKGS_{PLATFORM}`.
To add a platform, add a new hostname declaration, a new platform specific package list, and adjust the `if` condition at the bottom of [`build.sh`](./scripts/build.sh)

### Building Embedded Code
The embedded code is not built with the ROS nodes.
Do not attempt to add the embedded code to the ROS build system.
ROS will struggle to comprehend the cross compilation and linking used in the embedded codebase.

To build the embedded code, refer to the top level `Makefile` within the [Firmware](./src/control_board/firmware) folder.
After installing `arm-none-eabi-gcc`, the embedded code can typically be built and flashed with:
```sh
make build
# For Flashing using the Linux Open Source st-flash tool
make st-flash-lin
# For Flashing using the ST utility on Windows
make st-flash-win
# For Flashing using a black magic probe
make BlackMagic-flash
```

**Building on Windows:**
When building on Windows -w- VS Code, incorrectly configured environment variables can cause build environment to default to `MSVC` rather than the `arm-none-eabi` compiler.
This **will** cause the build to fail.

To resolve this error, either remove the `MSVC` compiler, or use the `CMakeLists` plugin within VS Code to correctly configure the compiler to `arm-none-eabi-gcc`.

## Simulation/HILT
Hardware In the Loop testing is possible through the Liftoff FPV Flight simulator.
First, download and install Liftoff from the [Steam Store](https://store.steampowered.com/app/410340/Liftoff_FPV_Drone_Racing/).
After installing, follow the directions within [Jchisholm/LiftoffSimulator](https://github.com/Jchisholm204/LiftoffSimulator) to build and install the required modifications.

Basic simulation is possible without installing the modifications, however, LiDAR data, along with the [`simulation/lidarstreams`](./src/simulation/lidarstreams) node will fail to function.

After Liftoff has been configured, launch the simulator, followed by the [`sim_rviz.py`](./launch/sim_rviz.py) Launch Script.
This script can be launched with:
```sh
ros2 launch ./launch/sim_rviz.py
```

After launching, RViZ should display the drone, along with simulated LiDAR data.
Note that with the current state of the simulation mod, the LiDAR data is directly correlated to the in-game camera angle.
Therefore, for accurate testing, set the in-game camera angle to zero (0).


## Flight Stack
### ROS
The highest level control algorithms.
Not designed to run in real-time (ROS cannot handle real-time scheduling).


### Control Board
Low Level Collision Avoidance and flight control algorithms.
The Control Board handles real-time flight kinematics based off of LiDAR data and feedback from the FC.

### INAV (FC)
The INAV project was used as the firmware for the flight controller.
The INAV stack handles the lowest level of flight kinematic control and motor control.

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

