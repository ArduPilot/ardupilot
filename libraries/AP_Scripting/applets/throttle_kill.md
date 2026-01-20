# Throttle Kill Applet

This applet is for fixed wing aircraft to allow a throttle value below
SERVO3_MIN to be used to kill an engine that uses PWM for engine kill,
while allowing values below SERVO3_MIN in any mode without the kill
switch being activated.

It is particularly suited to turbine engines which kill the motor
using a low throttle input.

# Parameters

The script creates a set of parameters with names starting withg THR_KILL

## THR_KILL_FUNC

This is the auxillary function to use for activating the throttle
kill. This should be set to a scripting auxillary function such as 300

## THR_KILL_CHAN

This is the output channel to apply the throttle kill to. This is
normally 3 for SERVO3

## THR_KILL_VAL

This is the auxiliary function value to activate the kill
switch. This is normally 2 meaning to activate on a high value of the
auxillary function. If you want it to activate on low values then set
this to 0.

## THR_KILL_DEF

This is the default auxiliary function value at startup. If you want
the engine to be killed by default on startup then set this to the
same value as THR_KILL_VAL

# Usage

After this script is installed and you have setup the above parameters
then the THR_KILL_CHAN servo output channel will be forced to
THR_KILL_PWM when the auxillary function is activated.
