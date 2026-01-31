# Packages
Each package contains relevant documentation in its directory.
See [this link](https://jchisholm204.github.io/posts/ros_project_templating/) for how to structure ROS packages/nodes.

### ([`control_board`](./control_board))
Contains the [embedded code](./control_board/firmware) and [driver](./control_board/driver) for the control PCB.
This board handles the low-level control and obstacle avoidance.

### [`simulation`](./simulation)
Contains the bridge code that allows ROS to interact with the Liftoff simulator.

### [`controllers`](./controllers)
High level controllers and control algorithms.
Includes planning algorithms.

### Interfaces (deprecated)
Interfaces used for interacting directly with sensors, removing the control board from the critical path.
