Gravitational control is a method for path execution that assigns a "gravity" to all points in the workspace.
The corresponding "gravity" values can then be turned into vectors acting on the drone.
After normalising, the vectors can be summed to find the movement vector the drone should take.

The gravitational controller on the drone is responsible for:
- Fetching LiDAR Data
- Changing LiDAR data into gravitational fields/vectors
- Inclusion of the destination/target point or velocity
- Outputting the movement vector the drone should take.

The gravitational controller must be able to work with fixed point values (applied as masses), and velocity values (applied during the vector summation step).