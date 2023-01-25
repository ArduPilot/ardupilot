# DJIR2 Mount Driver

DJIR2 gimbal mount driver lua script

How to use
  Connect gimbal to autopilot's CAN1 port
  Set CAN_D1_PROTOCOL = 10 (Scripting)
  Set CAN_P1_DRIVER = 1 (First driver)
  Set SCR_ENABLE = 1 to enable scripting
  Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
  Reboot the autopilot
  Copy the mount-djir2-driver.lua script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
