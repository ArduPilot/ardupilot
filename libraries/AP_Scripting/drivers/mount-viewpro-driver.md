# Mount Viewpro Driver

Viewpro gimbal driver lua script

# How To Use

  Connect gimbal UART to one of the autopilot's serial ports
  Set SERIALx_PROTOCOL = 28 (Scripting) where "x" corresponds to the serial port connected to the gimbal
  Set SCR_ENABLE = 1 to enable scripting and reboot the autopilot
  Set MNT1_TYPE = 9 (Scripting) to enable the mount/gimbal scripting driver
  Set CAM1_TYPE = 7 (Scripting) to enable camera control using the scripting driver
  Set RCx_OPTION = 300 (Scripting1) to allow real-time selection of the video feed and camera control
  Copy this script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot
  Set VIEP_CAM_SWLOW, VIEP_CAM_SWMID, VIEP_CAM_SWHIGH to which cameras are controlled by the auxiliary switch
      0: No change in camera selection
      1: EO1
      2: IR thermal
      3: EO1 + IR Picture-in-picture
      4: IR + EO1 Picture-in-picture
      5: Fusion
      6: IR1 13mm
      7: IR2 52mm
  Set VIEP_ZOOM_SPEED to control speed of zoom (value between 0 and 7)
  Set VIEP_ZOOM_MAX to the maximum zoom.  E.g. for a camera with 20x zoom, set to 20
  Optionally set VIEP_DEBUG = 1 or 2 to increase level of debug output to the GCS
