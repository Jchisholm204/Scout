# Localization Controller

Operates on LiDAR points from either the drone or simulation to generate "walls."
Walls are either "valid," representing free space, or "invalid," representing actual physical walls seen by the LiDAR sensors.

**Requires:**
The `laser_segmentation` node to be online.


The `laser_segmentation` node generates a base set of segments from the LiDAR data.
This node uses that data to generate "walls" by merging and separating segments.
This node also applies a hysteresis filter to smooth out jumpy walls and prevent false positives for unexplored areas.
