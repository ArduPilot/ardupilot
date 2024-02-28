# DJI RS2 and RS3-Pro Mount Driver

DJI RS2 and RS3-Pro gimbal mount driver lua script

## How to use

- Connect gimbal to autopilot's CAN1 port or CAN2 port
- If connected to CAN1, set CAN_D1_PROTOCOL = 10 (Scripting), CAN_P1_DRIVER = 1 (First driver)
- If connected to CAN2, set CAN_D2_PROTOCOL = 10 (Scripting), CAN_P2_DRIVER = 2 (Second driver)
- Set SCR_ENABLE = 1 to enable scripting
- Set SCR_HEAP_SIZE = 120000 (or higher)
- Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
- Reboot the autopilot
- Copy the mount-djirs2-driver.lua script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot

## Issues

If the ground station reports "Pre-arm: Mount not healthy", update the 
gimbal firmware using the DJI Ronin phone app to version 01.04.00.20 or 
later to correct a mismatch in the way data is received from the gimbal. 
Completing this update may take more than an hour.
