# Gimbal_Camera_Mode Lua Script

This script is a useful tool when using gimbal and camera. You can use it to switch between manual
and auto control for mission mode. It is meant for use with a transmitter that has a free switch
available (3-pos).

## How it works

The camera is setup for picture trigger during auto but can do other things e.g. video during
manual mode also. The script changes parameters but does not store them.
It changes the `SERVO<X>_FUNCTION`s and switches between `RCIN<X>` (pass-through) and `mount_tilt`,
`mount_roll` and `camera_trigger`.
In order to see the stored parameter values, disable the script (SCR_USER1) and reboot the FC.

## Modes

The control modes set by the 3-pos switch are as follows:

- Gimbal-Cam mode manual: Gimbal and Camera in manual control mode (pass-through)

- Gimbal-Cam mode auto/manual: Gimbal auto and Camera in manual control mode

- Gimbal-Cam mode auto: Gimbal and Camera in auto control mode (controlled by ArduPilot and mission)

## Setup and Use

- If you have not done so already, follow the instructions on ArduPilot's wiki page to enable auto camera and gimbal control.

- Also configure channels for manual control.
 
- For example I then have (to convert the values use https://ardupilot.org/copter/docs/parameters.html):

  - SERVO8_FUNCTION = 7 (mount_tilt)   leads to   MNT_RC_IN_TILT = 9 (RC9)     and to   RCIN9  (59)

  - SERVO9_FUNCTION = 8 (mount_roll)   leads to   MNT_RC_IN_ROLL = 10 (RC10)   and to   RCIN10 (60)

  - SERVOn_FUNCTION = ? (mount_pan)   is not used in this example

  - SERVO10_FUNCTION = 10 (camera_trigger)   leads to   RC12_OPTION = 9 (Camera Trigger)   and to   RCIN12 (62)

- Now fill these into the script Gimbal_Camera_Mode.lua:

- - -
     -- low: set gimbal and cam to manual mode (RCIN9, RCIN10, RCIN12)
    set_save_param('SERVO8_FUNCTION',59,false)
    set_save_param('SERVO9_FUNCTION',60,false)
    set_save_param('SERVO10_FUNCTION',62,false)
- - -
     -- middle: set gimbal to auto and cam to manual mode (mount_tilt, mount_roll, RCIN12)
    set_save_param('SERVO8_FUNCTION',7,false)
    set_save_param('SERVO9_FUNCTION',8,false)
    set_save_param('SERVO10_FUNCTION',62,false)
- - -
     -- high: set gimbal and cam to auto mode (mount_tilt, mount_roll, camera_trigger)
    set_save_param('SERVO8_FUNCTION',7,false)
    set_save_param('SERVO9_FUNCTION',8,false)
    set_save_param('SERVO10_FUNCTION',10,false)
- - -
   (these settings in 'update()' could be automatically derived if Gimbal and Camera are fully setup, but
   that would cause a bigger memory footprint of the script and increase the chance of faulty behaviour also)
- - -

- To get appropriate feedback from the script it is advised to set `SCR_DEBUG_LVL` to 3 during setup.

- Load the modified Gimbal_Camera_Mode.lua to the 'scripts' folder on your flight controller.

- Reboot your flight controller.

- Set `RC<X>_OPTION` to 300 for the RC input channel that corresponds to the switch that you would like to control the mode.

- Set `SCR_USER1` to 1 in order to enable the script and allow it to run.

- Test the script by moving the switch and checking the log messages output.

## Output Messages

### "LUA: Gimbal-Cam mode started"

The script has been started and runs now.

This message is usually immediately followed by "LUA: Gimbal-Cam mode < X >". If NOT, it might be disabled, check `SCR_USER1`. The script should run as soon as setting `SCR_USER1` to 1 (if not, there is a major error with lua scripting in general as the parameter `SCR_USER1` cannot be read).

### "LUA: RC channels not set"

The RC channel is not set to the scripting channel (300). Set it!

### "LUA: Gimbal-Cam mode < X >"

Mode chosen, confer 'Modes' above.

### "LUA: param set failed"

Failed to set parameter.  Script will persist to try and set parameters without exiting.  If this message continues to show, consider landing and investigating the underlying issue.

## Known Issues

Manual and auto modes do not have same travel as SERVOn_ settings are ignored for "pass-through" RCINn channels - a scaled and bounded variant must be introduced and used here.
