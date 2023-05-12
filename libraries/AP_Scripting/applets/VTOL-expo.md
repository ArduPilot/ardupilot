# VTOL Expo Tune

This script implements a system for estimating the VTOL thrust expo
(MOT_THST_EXPO on copter, Q_M_THST_EXPO on quadplanes).

The script is designed to be used in GUIDED mode.

# Parameters

The following parameters are used:

## EXPO_ENABLE

this must be set to 1 to enable the script

## EXPO_RC_FUNC

The RCz_OPTIONS scripting function binding to be used for this script.
Default RCz_OPTIONS binding is 300 (scripting1).

## EXPO_PULSE_TIME

This sets the length of the throttle pulses. The default of 1.5
seconds should be good for most vehicles, but some larger vehicles may
need a longer value.

## EXPO_PULSE_SLEW

This sets the slew time of the throttle pulses. The default of 0.05
seconds should be good for most vehicles, but some larger vehicles may
need a longer value.

## EXPO_PULSE_THST

This sets the amplitude of the throttle pulses as a proportion of
hover throttle. The default of 0.25 should be good for most vehicles.

## EXPO_CONV_THR

This sets the convergence threshold. When the acceleration error is
below this level for 5 consecutive pulse pairs the tuning is complete.

## EXPO_LIMIT_MIN

This sets the minimum expo value that the script will use.

## EXPO_LIMIT_MAX

This sets the maximum expo value that the script will use.

# Operation

You need to be in GUIDED mode with the vehicle at least 10 meters off
the ground. Have the VTOL-expo.lua script loaded and have a 3 position
switch setup with RCn_OPTION for that switch set to EXPO_RC_FUNC
(default to 300).

When you are ready to tune move the switch to the middle position. The
vehicle will pulse down the throttle for half of EXPO_PULSE_TIME
seconds followed up a pulse up of the throttle for the same time. The
vehicle will then settle for a few seconds before repeating. After
each pulse the expo will be adjusted based on the measured
acceleration.

When the tune is finished it will stop pulsing. You can then save the
tune by moving the switch to the top position.

At any point before you have saved you can move the switch to the
bottom position and the original expo will be restored and the tune
will stop. You can also stop the tune by changing to any other flight
mode. A flight mode change before saving will also revert any expo
change.

When the acceleration error has been less than EXPO_CONV_THR for 5
pulse pairs the tune is finished and the final value will be reported
and the tuning will stop.

You should tune in low wind conditions, and ensure the vehicle is not
oscillating at all before running the tune. If the vehicle oscillates
in roll, pitch or yaw while running the expo tune then the resulting
expo may be incorrect (it will get a value that is too high). You
should re-tune to prevent oscillation before re-running the expo tune.

# Note on using in quadplanes

In quadplanes the parameter Q_GUIDED_MODE will be overridden to a
value of 2 while running this script with EXPO_ENABLE=1. This ensures
that if you enter GUIDED mode from a VTOL mode it will not transition
to forward flight. A value of 2 for Q_GUIDED_MODE is new for the 4.5.x
plane release.

