# Simulation - Liftoff Simulator Bridge
This package acts as a bridge between the Liftoff FPV flight simulator and ROS.

To read more about how to set up the simulator, see [this writeup](https://jchisholm204.github.io/posts/liftoffsim/).

## `simulation/control`
The simulation control node serves to inject control node into the Liftoff Simulator.
It does this by emulating a virtual controller device that Liftoff sees as a generic joystick.


To send commands to the Simulator, create a node that publishes a `geometry_msgs::Quaternion` message with the format:
```
x: forwards velocity
y: sideways velocity
z: vertical velocity
w: rotational velocity
```
The control node expects that all values within the Quaternion are capped between [-1, 1].

### Parameters
- `crtl_topic`: string - topic name to listen to control commands on

### Topics
- `ctrl_topic`: Quaternion - Control Command

## `simulation/lidarstreams`
The simulation `lidarstreams` node relays UDP LiDAR data outputs from the Liftoff simulator into the ROS network in the form of Laser Scan messages.

### Parameters
- `lidar_front_name`: string - topic name for the front LiDAR
- `lidar_vertical_name`: string - topic name for the vertical LiDAR
- `pub_rate`: milliseconds (int) - the rate at which LiDAR messages are published

### Topics
- `lidar_front_name`: `LaserScan` - Front LiDAR Laser Scan (pub)
- `lidar_vertical_name`: `LaserScan` - Vertical LiDAR Laser Scan (pub)

## `simulation/telemetry`
Publishes the Telemetry information from the Liftoff Simulator as ROS Telemetry messages
