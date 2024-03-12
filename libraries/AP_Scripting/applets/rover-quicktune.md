# Rover QuickTune

Rover QuickTune tunes the steering (aka turn rate) and speed controller gains for rovers and boats

The script is designed to be used in Circle mode and updates the following parameters

ATC_STR_RAT_P
ATC_STR_RAT_I
ATC_STR_RAT_D
ATC_STR_RAT_FF
ATC_STR_RAT_FLTD
ATC_STR_RAT_FLTT
ATC_SPEED_P
ATC_SPEED_I
ATC_SPEED_D
CRUISE_SPEED
CRUISE_THROTTLE

# How To Use
Install this script in the autopilot's SD card's APM/scripts directory
Set SCR_ENABLE to 1 and reboot the autopilot
Set RTUN_ENABLE to 1 (default)

Set RCx_OPTION = 300 where "x" refers to the transmitter's 2 or 3 position switch
use to start/stop/save tuning.  E.g. if channel 6 is used set RC6_OPTION = 300

If necessary, the RTUN_RC_FUNC parameter can be set to another number (e.g. 302 for scripting3)
to avoid RCx_OPTION conflicts with other scripts.

By default the tune is saved a few seconds after completion.  To control saving of the tune manually set RTUN_AUTO_SAVE to 0

Arm the vehicle and switch to Circle mode
Optionally set CIRC_SPEED (or WP_SPEED) to about half the vehicle's max speed
Optionally set CIRC_RADIUS to a value at least twice the vehicle's turning radius
Note the above parmaters only take effect when the vehicle is switched into Circle mode

Move the RC switch to the middle position to begin the tuning process.
Text messages should appear on the ground station's messages area showing progress

During tuning the steering or throttle output are compared to the response for at least 10 seconds.
A message may appear stating the steering, turn rate, throttle or speed are too low in which case
the vehicle speed should be increased (Mission Planner's Action tab "Change Speed" button may be used)
or the radius should be reduced.

By default the gains will be tuned in this order:

- ATC_STR_RAT_FF, then ATC_STR_RAT_P and I are set to ratios of the FF
- CRUISE_SPEED and CRUISE_THROTTLE, then ATC_SPEED_P and I are set to ratios of the FF

The script will also adjust filter settings:

 - ATC_STR_RAT_FLTD and FLTT will be set to half of the INS_GYRO_FILTER value

Once tuning is complete "RTUN: tuning done" will be displayed
Save the tune by raising the RC switch to the high position

If the RC switch is moved high (ie. Save Tune) before the tune is completed the tune will pause, and any parameters completed will be saved and the current value of the one being actively tuned will remain active. You can resume tuning by returning the switch again to the middle position.  If the RC switch is moved to the low position, the parameter currently being tuned will be reverted but any previously saved parameters will remain.

If you move the switch to the low position at any time in the tune before gains are saved, then all parameters will be reverted to their original values. Parameters will also be reverted if you disarm before saving.

If the pilot gives steering or throttle input during tuning then tuning is paused for 4 seconds.  Tuning restarts once the pilot returns to the input to the neutral position.

# Parameters

The script has the following parameters to configure its behaviour

## RTUN_ENABLE

Set to 1 to enable the script

## RTUN_RC_FUNC

The RCx_OPTIONS function number to be used to start/stop tuning
By default RCx_OPTIONS of 300 (scripting1) is used

## RTUN_AXES

The axes that will be tuned. The default is 3 meaning steering and speed
This parameter is a bitmask, so set 1 to tune just steering.  2 for just speed

## RTUN_STR_FFRATIO

Ratio between measured response and FF gain. Raise this to get a higher FF gain
The default of 0.9 is good for most users.

## RTUN_STR_P_RATIO

Ratio between steering FF and P gains. Raise this to get a higher P gain, 0 to leave P unchanged
The default of 0.2 is good for most users.

## RTUN_STR_I_RATIO

Ratio between steering FF and I gains. Raise this to get a higher I gain, 0 to leave I unchanged
The default of 0.2 is good for most users.

## RTUN_SPD_FFRATIO

Ratio between measured response and CRUISE_THROTTLE value. Raise this to get a higher CRUISE_THROTTLE value
The default of 1.0 is good for most users.

## RTUN_SPD_P_RATIO

Ratio between speed FF and P gain. Raise this to get a higher P gain, 0 to leave P unchanged
The default of 1.0 is good for most users.

## RTUN_SPD_I_RATIO

Ratio between speed FF and I gain. Raise this to get a higher I gain, 0 to leave I unchanged
The default of 1.0 is good for most users.

## RTUN_AUTO_FILTER

This enables automatic setting of the PID filters based on the
INS_GYRO_FILTER value. Set to zero to disable this feature.

## RTUN_AUTO_SAVE

Enables automatic saving of the gains this many seconds after the tuning
completes unless the pilot move the RC switch low to revert the tune.
Setting this to a non-zero value allows you to use quicktune with a 2-position
switch, with the switch settings as low and mid positions. A zero
value disables auto-save and you need to have a 3 position switch.
