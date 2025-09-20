# Source Code

See [this link](https://jchisholm204.github.io/posts/ros_project_templating/) for how to structure ROS packages/nodes.

## Perception Package
Handles all perception including video processing, sensor data, ...

## Interfaces
The interfaces package handles all sensor/device interfacing.

### `inav_bridge`
The `inav_bridge` node handles interfacing ROS with INAV on the FC.
Receives generic `Vector3` commands on target angles to set the drone to.
Publishes IMU, Gyro, RSSI, and other data.
#### Deps
- libmsp (included in the project directory as a git submodule)

#### Parameters
- `fc_path`:
    - Desc: UNIX file path to the FC file descriptor
    - Type: `std::string`
    - Default: `/dev/ttyACM0`
- `fc_baud`:
    - Desc: Baud rate to use when interacting with the FC
    - Type: `uint64_t`
    - Default: `921600`
- `wd_timeout`:
    - Desc: Watch-Dog timeout value
    - Type: `uint64_t` (ms)
    - Default: `500`
- `pub_prefix`:
    - Desc: Prefix applied to all ROS/INAV messages
    - Type: `std::string`
    - Default: `inav`
- `movement_topic`:
    - Desc: Topic name to receive movement messages (`Vector3`)
    - Type: `std::string`
    - Default: `movement`
- `arm_topic`:
    - Desc: Topic name to receive arming messages over (`Bool`)
    - Type: `std::string`
    - Default: `armed`
- `fall_time`:
    - Desc: Time (in s) to allow the drone to fall on low throttle before shutting down during failure.
    - Type: `uint64_t` (seconds)
    - Default: `5`
- `fall_speed`:
    - Desc: Throttle speed to use when falling during failure
    - Type: `uint64_t`
    - Default: `1500`
    - Max: `2000`
    - Min: `0`

#### Topics
All topics are published with the naming convention `/prefix/topic`.
- `imu`: Gravity Readings in x, y, z format
- `gyro`: Rotational Readings in yaw, pitch, roll format
- `quat`: Orientation of the drone in quat format
- `batt`: Battery State (Voltage, Current, mAh used)
- `motor_speed`: Motor Speed (in RPM) reported by the ESC
- `altitude`: Barometric Altitude Reading
- `rssi`: RSSI Signal Strength (0-100%)

### `rplidar_bridge`
Handles interfacing between the RPLiDAR and ROS.

#### Parameters
- `lidar_path`:
    - Desc: UNIX File path to the LiDAR file descriptor
    - Type: `std::string`
    - Default: `/dev/ttyUSB0`
- `lidar_baud`:
    - Desc: Baud rate to use with the LiDAR (Change with Caution)
    - Type: `uint64_t`
    - Default: `460800`
- `pub_prefix`:
    - Desc: Publish Prefix (lidar topic name)
    - Type: `std::string`
    - Default: `lidar`
- `frame_id`:
    - Desc: Base Frame ID for RVIZ
    - Type: `std::string`
    - Default: `map`
- `pub_rate`:
    - Desc: Lidar Message Publish rate (in ms)
    - Type: `uint64_t`
    - Default: `50`
    - Max: `1000`
    - Min: `10`
- `lidar_mode`:
    - Desc: Lidar Mode to be used
    - Type: `uint64_t`
    - Default: `0`
    - Max: Depends on model of LiDAR
    - Min: `0`

#### Topics
- `$(pub_prefix)`: Laser Scan message with LiDAR Data
