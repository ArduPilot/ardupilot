this is for Matek F405-wing board

# Board connection

Just see board's documentation.


## Default connection
| Function     | Serial |
| ------       | ------ |
| USB          | Serial0 in MP |
| Telemetry    | UART1 (Serial1) |
| GPS          | UART6 (Serial3) | 
| Built-in OSD | Serial2 |
| UART2        | Serial4 |
| UART5        | Serial5 |


## OSD
Built-in OSD can be configured via files in root directory of SD card:
- eeprom.osd is configuration, exported from Configuration Tool.
- font.mcm is font (select one of https://github.com/night-ghost/minimosd-extra/tree/master/Released/FW_%2B_Char). This file will be deleted after flashing.

Firmware supports connection to built-in OSD with CT from my MinimOSD (https://github.com/night-ghost/minimosd-extra). To do this:
- set BRD_CONNECT_COM parameter to OSD's Serial (usually 2), then reboot / power cycle
- USB will be connected to OSD after reboot, supported load/store/fonts in MAVLink mode

## Voltage and current reading

How to get voltage/current reading
- BAT_MONITOR 4
- BAT_VOLT_PIN 10
- BAT_CURR_PIN 11
- BAT_VOLT_MULT 11.0
- BAT_AMP_PERVOLT 38.0

Don't try to configure Curr/Vol reading from Initial setup page of MP, because VOL/CURR variables will be reset.
