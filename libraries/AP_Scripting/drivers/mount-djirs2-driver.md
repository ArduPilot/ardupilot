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
