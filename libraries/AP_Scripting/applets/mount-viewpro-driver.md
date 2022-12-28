# Mount Viewpro Driver

Viewpro gimbal driver lua script

How to use
  Connect gimbal UART to one of the autopilot's serial ports
  Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the gimbal
  Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
  Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
  Set CAM_TRIGG_TYPE = 3 (Mount) to enable camera control using the mount driver
  Copy the mount-viewpro-driver.lua script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
