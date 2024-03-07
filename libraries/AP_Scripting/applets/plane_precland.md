# Plane Precision Landing

This script implements a precision landing system for VTOL fixed wing
aircraft (quadplanes).

Precision positioning over a landing sensor is supported in QLOITER,
QLAND, QRTL and AUTO landing.

# Parameters

Beyond the normal PLND parameters the script adds 2 additional parameters to
control it's behaviour. The parameters are:

## PLND_ALT_CUTOFF

This is an optional altitude in meters below which the precision
landing system will stop correcting the landing position. Many
precision landing sensors have poor performance at low altitudes, so
setting this to around 5 meters is advisable. A value of zero disables
this cutoff.

## PLND_DIST_CUTOFF

This is a maximum horizontal distance in meters that will be accepted
for a landing corrections. If this parameter is greater than zero and
the precision landing subsystem gives a distance beyond this distance
then precision landing correction will stop and the last landing
position will be used.

# Operation

You should first install and configure a precision landing sensor as described here:

  https://ardupilot.org/copter/docs/precision-landing-with-irlock.html

then you should enable the precision subsystem and install the lua
script in the APM/scripts folder of your flight controller.

The script will start adjusting the landing position only when in the
descent phase of an automatic VTOL landing. The PPLD log message can
be used to analyse the performance of the precision landing.

It is advisable to have a manual pilot able to take over control in a
mode such as QLOITER for instances where the precision landing system
may malfunction.

## Moving Target

If the PLND_OPTIONS bit for a moving target is enabled then the
vehicle will be set to track the estimated target velocity during
descent

## Precision QLoiter

To enable precision position hold in QLOITER you will need to use
auxiliary function 39 (PRECISION_LOITER) on an R/C switch or via GCS
auxiliary switch buttons. When enabled the vehicle will position
itself above the landing target. Height control is under user control.

