The Central Interface Board (CIB) handles low level drone operation including collision avoidance, flight execution, and system monitoring.
*Note: Flight kinematics are out of the scope for this board.*

# Interfaces
- 1x USB FS (over USBC) [[CIB USB Spec]]
	- PG Excluded
- 6x UART/USART (with 5V PG)
	- 2x RPLidar Connections [[RPLiDAR]]
	- 1x MSP connection
	- 1x CRSF OUT connection
	- 2x Debug/EPS
- 8x analog
	- Scaling voltage dividers for up to 8S battery monitoring
- 5V5A generic power connection (for Jetson Nano)

## Battery Monitoring
Multi cell batteries typically contain balance leads.
For a 4S battery, the lead pattern would be as follows:
- GND
- 4.2V
- 8.4V
- 12.6V
- 16.8V
For any battery with a higher number of cells, the pattern continues increasing 4.2V (Max cell voltage) for every connection.
The battery monitor must use the correct voltage dividers to accurately sample battery cell voltages.

# Chips
- An `STM32F446` should be used as the MCU.
- LLS's are not required for CSRF, RPLiDARS, or FC interactions.
- An IMU with internal filtering should be selected

# Power
The voltage rails and connected devices are listed below with amperage requirements
- VBatt
	- Drone (~A)
- 5V0
	- Jetson (5A)
	- 2x RPLidar (4A)
	- CSRF Module (500mA)
- 3V3
	- STM32F446 (1A)
