# Project Specifications
This document describes the purpose of the project and sets some basic design goals.

## Purpose
The purpose of this project is to design an autonomous UAV capable of mapping tight spaces without colliding into obstacles or requiring a GNSS signal.

## Design Goals
### Autonomous Operation
The UAV must be able to operate fully autonomously, requiring only an initial and final input from an operator to start and stop the program.

### Mapping and Localization
The UAV must be able to traverse unknown environments without the use of GNSS signals for localization.
While traversing these environments, the UAV must be able to construct a map of its surroundings.
The constructed map must use scientific units and contain a virtualized 3D representation of the explored environment.
The UAV must make an attempt to explore as much of the environment as possible.
All explored areas of the environment should be added to the map.
The program must return the map to the operator on program completion.

### Collision Avoidance
While navigating, the UAV must be capable of avoiding collisions within the environment.
A collision is defined as any part of the UAV coming into direct physical contact with the environment during autonomous navigation.
Exceptions to the above exist when the UAV is performing an action that requires contact with the environment during situations such as landing and takeoff.

### Telemetry and Radio Communication
The UAV must be able to operate without the usage of any radio signals communicating with external hardware including but not limited to: GNSS signals, Base Station links, or wireless controller inputs.


