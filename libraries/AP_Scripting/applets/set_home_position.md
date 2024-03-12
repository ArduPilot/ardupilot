# Set Home Position

This script is a useful tool to be able to set a home point other than arming with a transmitter that has a free switch
available (2-pos or 3-pos).

## How it works

Activating via 2-pos switch sets the current gps coordinates and the original height of the arming home as new home point, with
3-pos switch it is also possible to set a home with coordinates and height declared in the script (option disabled by default).

## Setup and Use

- Dynamic home only (the default)

1) Copy the LUA script via MavFTP to the APM/SCRIPTS directory, if it does not exist create it.

2) Activate the LUA script functionality in the parameters by setting SCR_ENABLE to 1 and reboot the FC.

3) Choose and set a 2-pos or 3-pos switch on the RC and check that it works in the Radio Calibration screen
of the Mission Planner.

4) For the corresponding channel chosen set RCx_OPTION to 300.

5) With the GPS fix and the system armed flip the switch to the high state position and check on the Mission Planner hud
if the "Dynamic home set" message appears, the new home will be set.
In "Data->Messages" the coordinates and height (that of arming) of the dynamic home set will be shown.
Without the GPS fix the message "Home position not set, unable to set HOME to current position" will be shown.
   
6) To set a new home dynamic point you will need to flip the switch again, the intermediate position of the switch will have no effect.


- Preset and dynamic home

1) Edit the LUA script by uncommenting the three lines (location object) with the coordinates and height and set them with the syntax:
the coordinates with the first 9 numbers and the height in centimeters, the example is in the script.
Uncomment them only when you are sure you have edited them correctly, do not use the preset ones in the script!

2) Copy the LUA script via MavFTP to the APM/SCRIPTS directory, if it does not exist create it.

3) Activate the LUA script functionality in the parameters by setting SCR_ENABLE to 1 and reboot the FC.

4) Choose and set a 3-pos switch on the RC and check that it works in the Radio Calibration screen of the Mission Planner.

5) For the corresponding channel chosen set RCx_OPTION to 300.

6) Dynamic home position: with the GPS fix and the system armed flip the switch to the high state position and check on the
Mission Planner hud if the "Dynamic home set" message appears, the new home will be set.
In "Data->Messages" the coordinates and height (that of arming) of the dynamic home set will be shown.
Without the GPS fix the message "Home position not set, unable to set HOME to current position" will be shown.
To set a new home dynamic point you will need to flip the switch again, the intermediate position of the switch will have no effect.
   
Preset home position: with the GPS fix and the system armed flip the switch to the middle state position and check on the
Mission Planner hud if the "Preset home activated" message appears, the new home will be set.
In "Data->Messages" the coordinates and height set in the script will be shown.

7) To set a new dynamic or preset home point you will need to switching to another position and returning to the desired one.
Always return the switch to the low position after making the desired choice.   

## Disclamer

It is recommended to use this script only if you are aware of what you are doing.
Forcibly setting a home point different from the arming one could cause problems in case of RTL, so only act if you are sure
of what you are doing.
