# Plane Follow

This script implements follow functionality for Plane. The plane must be
flying in Guided mode and will trigger on a scripting switch. The target plane
must be connected via MAVLink and sending mavlink updates to the chase plane
running this script. The MAVLINK_SYSID of the target must be set in FOLL_SYSID,
and must be different from the MAVLINK_SYSID of the following plane.


# Parameters

The script adds the following parameters to control it's behaviour. It uses 
the existing FOLL parameters that are used for the Copter FOLLOW mode.

## FOLL_FAIL_MODE

This is the mode the plane will change to if following fails. Failure happens
if the following plane loses telemetry from the target, or the distance exceeds
FOLL_DIST_MAX.

## FOLL_EXIT_MODE

The flight mode the plane will switch to if it exits following. 

## FOLL_ALT_TYPE

The existing FOLLOW mode parameter that specifies the target frame for 
altitudes to be used when following.

## FOLL_ACT_FN

The scripting action that will trigger the plane to start following. When this
happens the plane will switch to GUIDED mode and the script will use guided mode
commands to steer the plane towards the target.

# Operation

Install the lua script in the APM/SCRIPTS directory on the flight
controllers microSD card. Review the above parameter descriptions and
decide on the right parameter values for your vehicle and operations.

Most of the follow logic is in AP_Follow which is part of the ArduPilot c++
code, so this script just calls the existing methods to do things like
lookup the SYSID of the vehicle to follow and calculate the direction and distance
to the target, which should ideally be another fixed wing plane, or VTOL in
fixed wing mode.

The target location the plane will attempt to acheive will be offset from the target
vehicle location by FOLL_OFS_X and FOLL_OFS_Y. FOLL_OFS_Z will be offset against the 
target vehicle, but also FOLL_ALT_TYPE will determine the altitude frame that the vehicle
will use when calculating the target altitude. See the definitions of these
parameters to understand how they work. 
