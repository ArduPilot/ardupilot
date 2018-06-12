board connection:

just see board's documentation.

default connection:
USB                (Serial0 in MP)
Telemetry to UART1 (Serial1) 
GPS  to      UART6 (Serial3) 

Built-in OSD is     Serial2

this board REQUIRES external Baro and Compass via I2C bus: UART4 TX is SCL and UART4 RX is SDA

Built-in OSD can be configured via files in root directory of SD card:

* eeprom.osd for configuration,  and
* font.mcm for fonts (this file will be deleted after flashing)

also supported connection to built-in OSD with CT from my MinimOSD (https://github.com/night-ghost/minimosd-extra)
* set BRD_CONNECT_COM parameter to OSD's Serial(usually 2), then reboot / power cycle
* USB will be connected to OSD after reboot, supported load/store/fonts in MAVLink mode

