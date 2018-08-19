this is for AirbotV2, AirbotV3, Also Known As Omnibus boards

# Board connection

Just see board's documentation.


## Default connection
| Function     | Serial |
| ------       | ------ |
| USB          | Serial0 in MP |
| Telemetry    | UART1 (Serial1) |
| GPS          | UART6 (Serial3) | 
| Built-in OSD | Serial2 |

This board REQUIRES external Compass via I2C bus. 

## OSD
Built-in OSD can be configured via files in root directory of SD card:
- eeprom.osd is configuration, exported from Configuration Tool.
- font.mcm is font (select one of https://github.com/night-ghost/minimosd-extra/tree/master/Released/FW_%2B_Char). This file will be deleted after flashing.

Firmware supports connection to built-in OSD with CT from my MinimOSD (https://github.com/night-ghost/minimosd-extra). To do this:
- set BRD_CONNECT_COM parameter to OSD's Serial (usually 2), then reboot / power cycle
- USB will be connected to OSD after reboot, supported load/store/fonts in MAVLink mode

OSD will work better when VSYNC out from MAX connected to PC3 (R8 to Vcc).

## Voltage and current reading

How to get voltage/current reading(tested on omnibus, should work on other targets to):
- BAT_MONITOR 4
- BAT_VOLT_PIN 8
- BAT_CURR_PIN 7
- BAT_VOLT_MULT 11.0 (or 10.1 for apm power module)
- BAT_AMP_PERVOLT 38.0 (or 17 for apm power module)

Don't try to configure Curr/Vol reading from Initial setup page of MP, because VOL/CURR variables will be reset.

Attention!

If you select PPM (both via jumper or removing 0 ohm resistor) UART1 is no more used for RC IN and can be 
used for telemetry (Serial1 on MP settings).

Once PPM is selected you can use this pin for RC IN with PPM/SBUS/DSM, the parser in the HAL is able to understand 
which protocol are you using and to decode it properly. 
