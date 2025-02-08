# Torqeedo TroqLink Driver

Torqeedo TorqLink driver lua script

# How To Use

Connect the Torqeedo motor(s) to the autopilot's CAN ports.  If only one motor is used it should be connected to CAN1.
If two motors are used, connect the left motor to CAN1 and the right motor to CAN2

Enable CAN1 by setting these parameters:

- CAN_P1_DRIVER = 1 (First driver)
- CAN_D1_PROTOCOL = 10 (Scripting)

If CAN2 is being used set these parameters:

- CAN_P2_DRIVER = 2 (Second driver)
- CAN_D2_PROTOCOL = 12 (Scripting2)

Copy this script to the autopilot's SD card in the APM/scripts directory and restart the autopilot
