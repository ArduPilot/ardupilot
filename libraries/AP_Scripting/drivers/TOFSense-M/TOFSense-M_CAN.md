# TOFSense-M CAN Driver

TOFSense-M CAN Driver lua script. Only Copter/Rover/Plane 4.5 and above

## How to use - Pre-Configuration

- Connect sensors to autopilot's CAN1 port or CAN2 port
- If connected to CAN1, set CAN_D1_PROTOCOL = 10 (Scripting), CAN_P1_DRIVER = 1 (First driver)
- If connected to CAN2, set CAN_D2_PROTOCOL = 10 (Scripting), CAN_P2_DRIVER = 2 (Second driver)
- Set SCR_ENABLE = 1 to enable scripting
- Set SCR_HEAP_SIZE = 150000 (or higher)
- If using the sensor as a 1-D Rangefinder (typically for terrain following); set RNGFND1_TYPE = 36 (Scripting) to enable the rangefinder scripting driver
- If using the sensor as a 3-D Proximity sensor (typically for obstacle); set PRX1_TYPE = 15 (Scripting) to enable the proximity scripting driver
- Reboot the autopilot
- Copy the TOFSense-M_CAN.lua script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot


## How to use - Script Parameter Configuration

If everything above is done correctly, new "TOFSENSE_" parameters should be visible (only after script loads. Please refresh parameters if not visible). Script parameters to adjust are listed below

### TOFSENSE_PRX
If you have set RNGFND1_TYPE = 36, then set this as 0
If you have set PRX1_TYPE = 15, then set this as 1
Any change in this parameter will require a reboot (or scripting restart), ignore any errors on change before reboot. Make sure RNGFND/PRX is configured before setting this.


### TOFSENSE_NO
Total number of  TOFSense-M CAN sensors connected on the bus. Change will require a reboot


### TOFSENSE_MODE
TOFSENSE-M mode to be used.
- 0 for 8x8 mode.
- 1 for 4x4 mode.
All sensors must be in same mode. You can change the mode of sensor from NAssistant Software

### TOFSENSE_ID1
First TOFSENSE-M sensor ID. Leave this at 0 to accept all IDs and if only one sensor is present. You can change ID of sensor from NAssistant Software.


### TOFSENSE_INST1
First TOFSENSE-M sensors RNGFND_/PRX_ Instance
Setting this to 1 will pick the first backend from PRX_ or RNG_ Parameters.
So for example if RNGFND1_TYPE = 36, RNGFND2_TYPE = 36, then you can set this parameter to 2, to pick RNGFND2_ parameters to configure this sensor


### Configuring more than one sensor on same CAN bus

As described above, TOFSENSE_INST2, TOFSENSE_ID2 and TOFSENSE_INST3, TOFSENSE_ID3 can be adjusted so to support multiple sensors