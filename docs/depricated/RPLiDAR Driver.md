The RPLiDAR driver is a software abstraction between the LiDAR UART interface and the rest of the software.
It should:
- Communicate with the LiDAR through the STM32 UART driver (may be called Serial)
- Provide point cloud data to the rest of the software through an interface with realised SI units
- Be able to set up the LiDAR for scanning
- Change basic settings and perform basic configuration options