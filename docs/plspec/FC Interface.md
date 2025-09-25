The Flight Controller (FC) interface serves as a middle ware for basic events:
- Arming/Disarming
- Setting target velocities
*Note:*
- The Flight controller relies on the MSP and CSRF interfaces to communicate with the drone.
- The FC interface task is not required to perform flight kinematics.
- The FC interface task is not required to run PID loop calculations
	- This is handled automatically by the gravitational controller
- The FC interface task is required to monitor control timeouts and prevent the FC from timing out
- On the event of a failure, the FC task should:
	1. Notify higher level control
	2. Center all control values (make the drone hover in place)
	3. Perform rapid controlled decent
