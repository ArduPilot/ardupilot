# Rover QuickTune

Rover QuickTune tunes the steering (aka turn rate), speed and position controller velocity gains for rovers and boats

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
PSC_VEL_P
PSC_VEL_I
PSC_VEL_D

# How To Use
Install this script in the autopilot's SD card's APM/scripts directory
Set SCR_ENABLE to 1 and reboot the autopilot
Set RTUN_ENABLE to 1.

Set RCx_OPTION = 300 where "x" refers to the transmitter's 2 or 3 position switch
use to start/stop/save tuning.  E.g. if channel 6 is used set RC6_OPTION = 300

If necessary, the RTUN_RC_FUNC parameter can be set to another number (e.g. 302 for scripting3)
to avoid RCx_OPTION conflicts with other scripts.

If only a 2-position switch is available set RTUN_AUTO_SAVE to 1

Arm the vehicle and switch to Circle mode
Optionally set CIRC_SPEED (or WP_SPEED) to about half the vehicle's max speed
Optionally set CIRC_RADIUS to a value at least twice the vehicle's turning radius
Note the above parmaters only take effect when the vehicle is switched into Circle mode

Move the RC switch to the middle position to begin the tuning process.
Text messages should appear on the ground station's messages area showing progress

For P and D gain tuning, the relevant gain is slowly raised until the vehicle begins to oscillate after which the gain is reduced and the script moves onto the next gain.

For FF tuning, the steering or throttle output are compared to the response for at least 10 seconds.
A message may appear stating the steering, turn rate, throttle or speed are too low in which case
the vehicle speed should be increased (Mission Planner's Action tab "Change Speed" button may be used)
or the radius should be reduced.  The velocity FF is not tuned (nor does it need to be).

By default the gains will be tuned in this order:

 - ATC_STR_RAT_D
 - ATC_STR_RAT_P and I
 - ATC_STR_RAT_FF
 - ATC_SPEED_D
 - ATC_SPEED_P
 - CRUISE_SPEED and CRUISE_THROTTLE
 - PSC_VEL_D
 - PSC_VEL_P and I

The script will also adjust filter settings:

 - ATC_STR_RAT_FLTD and FLTT will be set to half of the INS_GYRO_FILTER value

Once tuning is complete "RTUN: tuning done" will be displayed
Save the tune by raising the RC switch to the high position

If the RC switch is moved high (ie. Save Tune) before the tune is completed the tune will pause, and any parameters completed will be saved and the current value of the one being actively tuned will remain active. You can resume tuning by returning the switch again to the middle position.  If the RC switch is moved to the low position, the parameter currently being tuned will be reverted but any previously saved parameters will remain.

If you move the switch to the low position at any time in the tune before using the Tune Save switch position, then all parameters will be reverted to their original values. Parameters will also be reverted if you disarm before saving.

If the pilot gives steering or throttle input during tuning then tuning is paused for 4 seconds.  Tuning restarts once the pilot returns to the input to the neutral position.

# Parameters

The script has the following parameters to configure its behaviour

## RTUN_ENABLE

Set to 1 to enable the script

## RTUN_RC_FUNC

The RCx_OPTIONS function number to be used to start/stop tuning
By default RCx_OPTIONS of 300 (scripting1) is used

## RTUN_AXES

The axes that will be tuned. The default is 7 meaning steering, speed and velocity
This parameter is a bitmask, so set 1 to tune just steering.  2 for speed. 4 for velocity

## RTUN_DOUBLE_TIME

How quickly a gain is raised while tuning. This is the number of seconds
for the gain to double. Most users will want to leave this at the default of 10 seconds.

## RTUN_PD_GAINMARG

The percentage P and D gain margin to use. Once the oscillation point is found
the gain is reduced by this percentage. The default of 80% is good for most users.

## RTUN_FF_GAINMARG

The percentage FF gain margin to use. Once the output and response are used to calculate
the ideal feed-forward, the value is reduced by this percentage. The default of 20% is good for most users.

## RTUN_OSC_SMAX

The Oscillation threshold in Hertz for detecting oscillation
The default of 5Hz is good for most vehicles  but on very large vehicles
you may wish to lower this. For a vehicle of 50kg a value of 3 is likely to be good.
For a vehicle of 100kg a value of 1.5 is likely to be good.

You can tell you have this set too high if you still have visible
oscillations after a parameter has completed tuning. In that case
halve this parameter and try again.

## RTUN_SPD_P_MAX

The speed controller P gain max (aka ATC_SPEED_P)

## RTUN_SPD_D_MAX

The speed controller D gain max (aka ATC_SPEED_D)

## RTUN_PI_RATIO

The ratio for P to I. This should normally be 1, but on some large vehicles a value of up to 3 can be
used if the I term in the PID is causing too much phase lag.

If RTUN_RP_PI_RATIO is less than 1 then the I value will not be
changed at all when P is changed.

## RTUN_AUTO_FILTER

This enables automatic setting of the PID filters based on the
INS_GYRO_FILTER value. Set to zero to disable this feature.

## RTUN_AUTO_SAVE

Enables automatic saving of the gains this many seconds after the tuning
completes unless the pilot move the RC switch low to revert the tune.
Setting this to a non-zero value allows you to use quicktune with a 2-position
switch, with the switch settings as low and mid positions. A zero
value disables auto-save and you need to have a 3 position switch.
