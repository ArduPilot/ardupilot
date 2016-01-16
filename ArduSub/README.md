ArduSub
=======

# Initial Setup

1. Upload firmware to PX4
2. Open QGroundControl. Set virtual joystick to unchecked. 
3. Calibrate joystick with throttle and yaw on right stick, pitch and roll on left stick. Very carefully! Check RC out to verify all 1500 when centered.
4. Calibrate radio using the joystick with the same inputs.
5. Change parameters:
	- DISARM_DELAY to 0
	- ARMING_CHECK to Disabled
	- BRD_SAFETYENABLE to Disabled
6. Setup Power
	- Analog Voltage and Current
	- 10000 mAh
	- Power Sensor: Other
	- Current pin: Pixhawk
	- Voltage pin: Pixhawk
	- Voltage multiplier: 9.174
	- Amps per volt: 14.70
