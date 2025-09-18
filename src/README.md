# Source Code

See [this link](https://jchisholm204.github.io/posts/ros_project_templating/) for how to structure ROS packages/nodes.

## Perception Package
Handles all perception including video processing, sensor data, ...

## Interfaces
The interfaces package handles all sensor/device interfacing.
### `inav_bridge`
The `inav_bridge` node handles interfacing ROS with INAV on the FC.

Deps:
    - libmsp (included in the project directory as a git submodule)
