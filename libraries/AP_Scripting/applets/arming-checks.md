# arming_checks

This script implements user defined pre-arm and arming checks. Checks
can be warnings or errors. Warnings don't prevent arming but will display
on the ground station, whereas errors will prevent arming.

Additional arming checks can be added by adding a method that returns true/false (true = arming ok), and defining the check, including the parameter it uses in the method arming_checks().

# Parameters

There is a single parameter defined for each arming check. The error or warning level can be changed for each arming check by setting the MAV_SEVERITY of the arming check. An arming check can be disabled by setting the parameter value to MAV_SEVERITY.NONE or -1.

All arming check parameters are prefixed with ZAR_
Z = Scripting Parameter
AR = Arming Checks

# ZAR_SYSID

If the SYSID_THISMAV parameter has not been changed from it's default value of 1, this arming check will prevent arming. This is intended to be used in large fleets, were SYSID_THISMAV should always be set. The check can be disabled by setting this parameter to -1.

# ZAR_FOLL_SYSID

If the FOLL_SYSID parameter has not been changed from it's default value of 0, this arming check will prevent arming. This is intended to be used in large fleets flying in swarms, were FOLL_SYSID should almost always be set. The check can be disabled by setting this parameter to -1.

# ZAR_MNTX_SYSID

If the MNTX_SYSID parameter does not match the FOLL_SYSID, this arming check will prevent arming. This is intended to be used in large fleets flying in swarms, were the camera mount should always be pointing to the vehicle being followed. The check can be disabled by setting this parameter to -1.

# ZAR_RTL_ALT

This check is to by default warn if the vehicle RTL_ALT has not been set. This is intended to be an example of how specific configuration checks could be added for specific requirements without having to change the firmware.

# ZAR_RTL_CLIMB

This check is to by default warn if the vehicle RTL_CLIMB has not been set. This is intended to be an example of how specific configuration checks could be added for specific requirements without having to change the firmware.

# ZAR_Q_FS_LAND and ZAR_Q_FS_RTL

These two checks are to display important configuration information for a vehicle at boot time. One of these checks should be displayed. Neither is an error. This is to demonstrate how to notify the pilot of important configuration information at boot time without preventing flying.

# ZAR_AIRSPEED

The values of AIRSPEED_STALL (optional) should be less than AIRSPEED_MIN which should be less than AIRSPEED_CRUISE
which should be less than AIRSPEED_MAX. This validates that this rule has not been broken.

# ZAR_STALL

AIRSPEED_STALL is a newly added and optional value, but if set, AIRSPEED_MIN should not be more than 25% higher 
than AIRSPEED_STALL. When looked at the other way round, AIRSPEED_STALL should not be more than 20% lower than
AIRSPEED_MIN. 

# ZAR_SCALING

The SCALING_SPEED should be close to AIRSPEED_CRUISE. Its quite easy to remember to set AIRSPEED_CRUISE correctly
when setting up an aircraft, but not to reset (and possibly retune) SCALING_SPEED. This gives a warning if the values are very different.

# Operation

Copy this file to the APM/scripts folder on your SD card and make sure 

SCR_ENABLE = 1
SCR_HEAPSIZE = 250000 (minimum)

Edit the arming_checks table if you want to change any of the checks from warnings to errors or errors to warnings or delete the row if you want to remove the check completely.
