# Set Home Position applet

This script is a useful tool to be able to set a home point other than arming with a transmitter that has a free switch
available (2-pos or 3-pos).
It is possible to set a dynamic home, i.e. at the coordinates where the drone is at that moment, or a preset home set by parameters.
If terrain is enabled the script will use that as the height for the dynamic home.
It provides interactive messages on each feature in both Messages and hud of Mission Planner.

## How it works

Activating via 2-pos switch sets the current gps coordinates and the original height of the arming home as new home point
or reactivate the arming home.
3-pos switch it is also possible to use a preset home with coordinates and height declared in the script (option disabled by default).
In both cases to use the desired home (arming, preset or dynamic) the switch must be left in the desired position.
The chosen home will then be the one relative to the switch position, and will be maintained in that state until disarm
if that is your desired home.
Therefore a different status always cancels the previous one, only if the home preset has not been set via parameters
the intermediate position of the switch will have no effect and the previously chosen home will be retained.

## Parameters Descriptions

The script adds 4 parameters to control it's behaviour. The parameters
are:

- HSW_ENABLE : Setting it to 1 enables the script, 0 to disable it (the default)
- HSW_PRESET_LAT : Desired home preset latitude, example value: 44.123456
- HSW_PRESET_LNG : Desired home preset longitude, example value: 11.123456
- HSW_PRESET_ALT : Desired home preset altitude, value in centimetres

## Setup and Use

- Dynamic home only

1. copy the LUA script via MavFTP to the APM/SCRIPTS directory, if it does not exist create it

2. activate the LUA script functionality in the parameters by setting SCR_ENABLE, reboot the FC and
activate the script by setting HSW_ENABLE to 1, followed by a parameter refresh.
If the SCR_ENABLE parameter is not present, it means that your flight control does not support LUA scripts

3. choose and set a 2-pos or 3-pos switch on the RC and check that it works in the Radio Calibration screen
of the Mission Planner

4. For the corresponding channel chosen set RCx_OPTION to 300

5. with the GPS fix and the system armed flip the switch to the high state position and check on the Mission Planner hud
if the "HSW: dynamic home activated" message appears, the new home will be set.
In "Data->Messages" the coordinates and height (that of arming) of the dynamic home set will be shown.
Without the GPS fix the message "HSW: home position not set, unable to set home to current position" will be shown.
If terrain is enabled the script will set the dynamic home height to that detected by its database, showing a message
"HSW: using terrain data, altitude X", where X is the altitude in the terrain database at those coordinates

6. to set a new home dynamic point you will need to flip the switch again, i.e. in the high-low-high sequence, if you use 3-pos
switches the middle one will have no effect and the previously chosen home will be retained (arming or dynamic)

7. the switch should be kept on the high position if you want to keep that home dynamic until disarming.
Returning to the low position will activate the arming home and display the message 'HSW: arming home activated' in the hud


- Preset and dynamic home

1. follow the same steps for the dynamic home but set latitude, longitude and altitude in the respective parameters.
If these values are left at the default (0), the preset home will not work

2. choose and set a 3-pos switch on the RC and check that it works in the Radio Calibration screen of the Mission Planner

3. dynamic home position: with the GPS fix and the system armed flip the switch to the high state position and check on the
Mission Planner hud if the "HSW: dynamic home activated" message appears, the new home will be set.
In "Data->Messages" the coordinates and height of the dynamic home set will be shown.
Without the GPS fix the message "HSW: home position not set, unable to set home to current position" will be shown.
If terrain is enabled the script will set the dynamic home height to that detected by its database, showing a message
"HSW: using terrain data, altitude X", where X is the altitude in the terrain database at those coordinates
   
4. preset home position: with the GPS fix and the system armed flip the switch to the middle state position and check on the
Mission Planner hud if the "HSW: preset home activated" message appears, the new home will be set.
In "Data->Messages" the coordinates and height set in the script will be shown

5. to set a new home dynamic point you will need to flip the switch again, i.e. in the high-low-high or high-middle-high sequence

6. a different home status of the switch always cancels the previous one, then at any time the user can set the switch to
'arming, preset or dynamic' and keep it in case until disarming

## Disclamer

It is recommended to use this script only if you are aware of what you are doing.
Forcibly setting a home point different from the arming one could cause problems in case of RTL, so only act if you are sure
of what you are doing.
If terrain is enabled always consider the possible gaps between the actual data and the data in its database.
