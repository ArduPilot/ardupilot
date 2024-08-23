# Plane Follow

This script implements follow functionality for Plane. The plane must be
flying in Guided mode and will trigger on a scripting switch.


# Parameters

The script adds the following parameters to control it's behaviour. It uses 
the existing FOLL parameters that are used for the Copter FOLLOW mode.

## TERR_BRK_ENABLE

This must be set to 1 to enable the script.

## TERR_BRK_ALT

This is the terrain altitude threshold for engaging BRAKE mode. The
onboard terrain system must be enabled with TERRAIN_ENABLE=1 and
terrain must have either been preloaded to the vehicle (see
https://terrain.ardupilot.org ) or be available from the ground
station over MAVLink.

Make sure you set sufficient margin to cope with obstacles such as
trees or any local towers or other obstacles.

## FOLL_ALT_TYPE

The existing FOLLOW mode parameter that specifies the target frame for 
altitudes to be used when following.

## FOLL_FAIL_MODE

This is the mode the plane will change to if following fails. Failure happens
if the following plane loses telemetry from the target, or the distance exceeds
FOLL_???_DIST

# Operation

Install the lua script in the APM/SCRIPTS directory on the flight
controllers microSD card. Review the above parameter descriptions and
decide on the right parameter values for your vehicle and operations.

Most of the follow logic is in AP_Follow which is part of the ArduPilot c++
code, so this script just calls the existing methods to do things like
lookup the SYSID of the vehicle to follow and calculate the direction and distance
to the target, which should ideally be another fixed wing plane, or VTOL in
fixed wing mode.
