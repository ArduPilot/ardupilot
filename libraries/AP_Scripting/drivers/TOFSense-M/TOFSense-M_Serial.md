# TOFSense-M Serial Driver

TOFSense-M Serial Driver lua script. Only Copter/Rover/Plane 4.5 and above

## How to use - Pre-Configuration

- Connect sensors to one of autopilot's Serial port
- Set SERIALx_PROTOCOL 28
- Set SCR_ENABLE = 1 to enable scripting
- Set SCR_HEAP_SIZE = 150000 (or higher)
- If using the sensor as a 1-D Rangefinder (typically for terrain following); set RNGFND1_TYPE = 36 (Scripting) to enable the rangefinder scripting driver
- If using the sensor as a 3-D Proximity sensor (typically for obstacle); set PRX1_TYPE = 15 (Scripting) to enable the proximity scripting driver
- Reboot the autopilot
- Copy the TOFSense-M_Serial.lua script to the autopilot's SD card in the APM/scripts directory and reboot the autopilot

## How to use - Script Parameter Configuration

If everything above is done correctly, new "TOFSENSE_S_" parameters should be visible (only after script loads. Please refresh parameters if not visible). Script parameters to adjust are listed below. Any changes would require reboot

### TOFSENSE_S1_PRX
If you have set RNGFND1_TYPE = 36, then set this as 0
If you have set PRX1_TYPE = 15, then set this as 1
Any change in this parameter will require a reboot (or scripting restart), ignore any errors on change before reboot. Make sure RNGFND/PRX is configured before setting this.

### TOFSENSE_S1_SP
UART instance sensor is connected to. Set 1 if sensor is connected to the port with fist SERIALx_PROTOCOL = 28. For example, if SERIAL1_PROTOCOL = 28, and SERIAL2_PROTOCOL = 28, then setting this parameter 2 would assume the SERIAL2 port is attached to the sensor

### TOFSENSE_S1_BR
Serial Port baud rate. Sensor baud rate of the sensor can be changed from Nassistant software


## Advanced: Configuring more than one sensor on different Serial ports

It is not recommended to use this driver for more than one sensor. However, it is possible if you wish to do it.
Simply make a copy of the lua script, open it and change the first line "local sensor_no = 1" to "local sensor_no = 2" and upload that script along side the first script.
Once a reboot is done, new TOFSENSE_S2_ parameters should be visible which can be configured as described above. 

