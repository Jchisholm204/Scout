# Device Specifications
- Vendor ID: `0xFFFE`
- Product ID: `0xD40E`
# EP Specifications
The following lists the endpoint specifications for the USB interface.
RX/TX pairs are listed from the perspective of the host as per USB IF specifications.
## Control/Planning (CDC ACM)
- NTF Interface Number: `0x00`
- Data Interface Number: `0x01`
- Data RX EP: `0x01`
- Data TX EP: `0x81`
- Maximum Packet Size: `0x40`
- NTF EP: `0x82`
- Maximum NTF Packet Size: `0x08`
### TX Interface
- Target Type - `(u8)enum`
	- Velocity - Sets the target velocity of the drone
	- Positional - Sets the target position of the drone
	- Hold - Drone will ignore position and attempt to hold the current position
- Target X - `float32`
- Target Y - `float32`
- Target Z - `float32`
- Target W (yaw) - `float32`
- Flags
	- Arm Drone
	- Disarm Drone
	- Reset 1
	- Reset 0
	- Execute Fail Safe
	- Valid 0
	- Valid 1
### RX Interface
- $A_x$ (Acceleration in X) - `float32`
- $A_y$ (Acceleration in Y) - `float32`
- $A_z$ (Acceleration in Z) - `float32`
- $A_w$ (Acceleration in W) - `float32`
- Battery Voltage - `float32`
- Flags - `u8`
	- Stopped before Collision
	- Drone Armed
	- FC Error
	- CIB Error
	- Valid 0
	- Valid 1

## RPLiDAR Hor
Forwards the RPLiDAR data from the horizontal lidar.
- NTF Interface Number: `0x02`
- Data Interface Number: `0x03`
- Data RX EP: `0x03`
- Data TX EP: `0x83`
- Maximum Packet Size: `0x40`
- NTF EP: `0x84`
- Maximum NTF Packet Size: `0x08`
## RPLiDAR Ver
Forwards the RPLiDAR data from the vertical lidar.
- NTF Interface Number: `0x04`
- Data Interface Number: `0x05`
- Data RX EP: `0x05`
- Data TX EP: `0x85`
- Maximum Packet Size: `0x40`
- NTF EP: `0x86`
- Maximum NTF Packet Size: `0x08`