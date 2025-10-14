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


