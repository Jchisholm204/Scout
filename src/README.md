# Source Code

See [this link](https://jchisholm204.github.io/posts/ros_project_templating/) for how to structure ROS packages/nodes.

## Controllers
The Controllers package handles all the control algorithms applied to the drone.

### `gravitational`
Gravitational based motion controller.
Accepts in a directional vector on where to move.
Produces stick outputs that can be applied to the drone.
Uses Lidar data avoid objects.

#### Parameters
- `imu_topic`:
    - Desc: Topic name of the IMU topic
    - Type: `std::string`
    - Default: `/inav/imu`
- `gyro_topic`:
    - Desc: Topic name for the drone gyro
    - Type: `std::string`
    - Default: `/inav/gyro`
- `battery_topic`:
    - Desc: Topic name for the drone battery
    - Type: `std::string`
    - Default: `/inav/batt`
- `motor_topic`:
    - Desc: Topic name for the drone motor average rpm
    - Type: `std::string`
    - Default: `/inav/mtr`
- `altitude_topic`:
    - Desc: Topic name for the drone barometric altitude
    - Type: `std::string`
    - Default: `/inav/altitude`
- `hor_lidar_topic`:
    - Desc: Topic name the horizontal LiDAR data
    - Type: `std::string`
    - Default: `/lidar`
- `ver_lidar_topic`:
    - Desc: Topic name the vertical LiDAR data
    - Type: `std::string`
    - Default: `/verdar`
- `target_topic`:
    - Desc: Topic name to listen for target positions on
    - Type: `std::string`
    - Default: `/gtarget`
- `movement_topic`:
    - Desc: Topic name top send out drone movement commands on
    - Type: `std::string`
    - Default: `/inav/movement`
- `arming_topic`:
    - Desc: Topic name top send out drone arming commands on
    - Type: `std::string`
    - Default: `/inav/armed`
- `collide_max`:
    - Desc: Maximum range value to consider points for collision detection
    - Type: `float` (meters)
    - Default: `0.8f`
- `hover_alt`:
    - Desc: Minimum distance from the ground to overcome ground effects
    - Type: `float` (meters)
    - Default: `0.5f`
- `log_level`:
    - Desc: Logging Level to use
    - Type: `std::string` [`none`, `info`, `warning`, `error`]
    - Default: `none`
- `horizontal_weight`:
    - Desc: Weighting applied to horizontal LiDAR readings
    - Type: `float` (meters)
    - Default: `1.0f`
- `vertical_weight`:
    - Desc: Weighting applied to vertical LiDAR readings
    - Type: `float` (meters)
    - Default: `1.0f`
- `target_weight`:
    - Desc: Weighting applied to the target position 
    - Type: `float` (meters)
    - Default: `1.0f`
- `hover_weight`:
    - Desc: Weighting applied to the hover distance
    - Type: `float` (meters)
    - Default: `1.0f`

#### Topics
- `[movement_topic]`: Publishes stick commands
- `[arming_topic]`: Topic to send the arming messages over
- `[target_topic]_mk`: Publishes a `visualization_msgs::msg::Marker` for Rviz compatibility

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
