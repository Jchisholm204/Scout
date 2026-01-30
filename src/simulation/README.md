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
The topic name can be configured through setting the `ctrl_topic` parameter at launch time.

## `simulation/lidarstreams`
