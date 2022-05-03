# VTOL QuickTune

This script implements a fast VTOL tuning system for multicopters and
quadplanes. This script can be used to automate the process of
producing a good "manual tune" for the VTOL rate control parameters.

The script is designed to be used in QLOITER mode for quadplanes or
LOITER mode in multicopters, although it can also be used in other
VTOL modes.

# Parameters

The script adds 7 parameters to control it's behaviour. The parameters
are:

## QUIK_ENABLE

this must be set to 1 to enable the script

## QUIK_AXES

This is the set of axes that the tune will run on. The default is 7,
which means roll, pitch and yaw. It is a bitmask, so if you want just
roll and pitch then set this to 3. For just yaw you would set it to 4.

## QUIK_DOUBLE_TIME

This controls how quickly a gain is raised while tuning. It is a time
in seconds for the gain to double. Most users will want to leave this
at the default of 10 seconds.

## QUIK_GAIN_MARGIN

This is the percentage gain margin to use. Once the oscillation point
for a gain is found the gain is reduced by this percentage. The
default of 70% is good for most users.

## QUIK_OSC_SMAX

This is the oscillation threshold in Hertz for detecting oscillation
when a gain is raised. The default of 5Hz is good for most vehicles,
but on very large vehicles you may wish to lower this. For a vehicle
of 50kg a value of 3 is likely to be good. For a vehicle of 100kg a
value of 1.5 is likely to be good.

You can tell you have this set too high if you still have visible
oscillations after a parameter has completed tuning. In that case
halve this parameter and try again.

## QUIK_YAW_P_MAX

This sets a limit on the YAW_P rate gain. The yaw axis on most
multirotor style vehicles needs to have a much lower limit on the P
gain than the oscillation limit to ensure that enough control remains
for roll, pitch and thrust. A maximum of 0.5 is good for most VTOL
vehicles.

## QUIK_YAW_D_MAX

This sets a limit on the YAW_D rate gain. The yaw axis on most
multirotor style vehicles needs to have a much lower limit on the D
gain than the oscillation limit to ensure that enough control remains
for roll, pitch and thrust. A maximum of 0.01 is good for most VTOL
vehicles.

# Operation

First you should setup harmonic notch filtering using the guide in the
ArduPilot wiki. This tuning system relies on you already having
reduced gyro noise using the harmonic notch filter. It will fail if
your noise is too high.

Install the lua script in the APM/SCRIPTS directory on the flight
controllers microSD card, then set SCR_ENABLE to 1. Reboot, and
refresh parameters. Then set QUIK_ENABLE to 1.

You will then need to setup a 3 position switch on an available RC
input channel for controlling the tune. If for example channel 6 is
available with a 3 position switch then you should set RC6_OPTION=300
to association the tuning control with that switch.

You should then takeoff and put the vehicle into QLOITER mode (for
quadplanes) or LOITER mode (for multicopters) and have it in a steady
hover in low wind.

Then move the control switch you setup with option 300 to the middle
position. This will start the tuning process. You will see text
messages on the ground station showing the progress of the tune. As
the aircraft reaches the oscillation limit of each parameter it will
start a small oscillation, then it will reduce that gain by the
configured QUIK_GAIN_MARGIN percentage and then move onto the next
parameter.

With default settings the parameters to be tuned will be:

 - RLL_D
 - RLL_P
 - PIT_D
 - PIT_P
 - YAW_D
 - YAW_P

The script will also adjust filter settings using the following rules:

 - the FLTD and FLTT settings will be set to half of the INS_GYRO_FILTER value
 - the YAW_FLTE filter will be set to a maximum of 2Hz
 - if no SMAX is set for a rate controller than the SMAX will be set to 50Hz

Once the tuning is finished you will see a "Tuning: done" message. You
can save the tune by moving the switch to the high position. You
should do this to save before you land and disarm.

If you move the switch to the low position at any time in the tune
then all parameters will be reverted to their original
values. Parameters will also be reverted if you disarm.

