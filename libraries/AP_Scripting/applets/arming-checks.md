# arming-checks

This script implements user defined pre-arm and arming checks. Checks
can be warnings or errors. Warnings don't prevent arming but will display
on the ground station, whereas errors will prevent arming.

Additional arming checks can be added by adding a method that returns true/false (true = arming ok), and defining the check, including the parameter it uses in the method arming_checks().

# Parameters

There is a single parameter defined for each arming check. The error or warning level can be changed for each arming check by setting the MAV_SEVERITY of the arming check. An arming check can be disabled by setting the parameter value to MAV_SEVERITY.NONE or -1.

All arming check parameters are prefixed with ARM_. Copter specific checks are prefexed with ARM_C_, Plane specific checks are prefixed with ARM_P_, Rover specific checks are prefixed with ARM_R_, Blimp specific checks are prefixed with ARM_B_, Antenna Tracker specific checks are prefixe with ARM_A_, TradHeli specific checks are prefixed with ARM_H_.

# Generic checks

These checks apply to all firmware types

# ARM_SYSID

If the SYSID_THISMAV parameter has not been changed from it's default value of 1, this arming check will prevent arming. This is intended to be used in large fleets, were SYSID_THISMAV should always be set. 

# ARM_FOLL_SYSID_X

If the FOLL_SYSID parameter has not been changed from it's default value of 0, this arming check will prevent arming. This is intended to be used in large fleets flying in swarms, were FOLL_SYSID should almost always be set. 
# ARM_FOLL_OFS_DEF

Follow offsets should not be left as the default (0). If using the Follow library, leaving the follow offsets at zero will 
mean the follow vehicle will attempt to fly to where the target is, which would likely cause a crash. 

# ARM_MNTX_SYSID

If the MNTX_SYSID parameter does not match the FOLL_SYSID, this arming check will prevent arming. This is intended to be used in large fleets flying in swarms, were the camera mount should always be pointing to the vehicle being followed. 


# ARM_RTL_CLIMB

This check is to by default warn if the vehicle RTL_CLIMB has not been set. This is intended to be an example of how specific configuration checks could be added for specific requirements without having to change the firmware. 

# ARM_ESTOP

This check will fail if the motors are emergency stopped. The standard pre-arm will not prevent arming if the emergency stop
has been set by a switch (set using RCx_OPTION = 165). This check will prevent arming regardless of the reason for the EStop. 

# ARM_FENCE

If fences are loaded but no fence is enabled this will prevent arming. 

# ARM_RALLY

If there is a rally point more than RALLY_LIMIT_KM kilometers from the home location, this will prevent arming..T

# Copter specific checks

The following checks only apply if the firmware is ArduCopter

# ARM_C_RTL_ALT

This check is to by default warn if the vehicle RTL_ALT has been set to a value > ARM_V_ALT_LEGAL. 

# Plane specific checks

The following checks only apply if the firmware is ArduPlane

# ARM_P_RTL_ALT

This check is to by default warn if RTL_ALTITUDE > ARM_V_ALT_LEGAL which defaults to 120m (400ft) which is the legal limit in many jurisdictions. This is intended to be an example of how specific configuration checks could be added for specific requirements without having to change the firmware.

# ARM_P_QRTL_ALT

This check is to by default warn if Q_RTL_ALT > ARM_V_ALT_LEGAL which defaults to 120m (400ft) which is the legal limit in many jurisdictions. This is intended to be an example of how specific configuration checks could be added for specific requirements without having to change the firmware.

# ARM_P_Q_FS_LAND and ARM_P_Q_FS_RTL

These two checks are to display important configuration information for a vehicle at boot time. One of these checks should be displayed. Neither is an error. This is to demonstrate how to notify the pilot of important configuration information at boot time without preventing flying.

# ARM_P_AIRSPEED

The values of AIRSPEED_STALL (optional) should be less than AIRSPEED_MIN which should be less than AIRSPEED_CRUISE
which should be less than AIRSPEED_MAX. This validates that this rule has not been broken.

# ARM_P_STALL

AIRSPEED_STALL is a newly added and optional value, but if set, AIRSPEED_MIN should not be more than 25% higher 
than AIRSPEED_STALL. When looked at the other way round, AIRSPEED_STALL should not be more than 20% lower than
AIRSPEED_MIN. 

# ARM_P_SCALING

The SCALING_SPEED should be close to AIRSPEED_CRUISE. Its quite easy to remember to set AIRSPEED_CRUISE correctly
when setting up an aircraft, but not to reset SCALING_SPEED. The aircraft should be retuned if SCALING_SPEED is changed. The airThis gives a warning if the values are very different.

# Operation

Copy this file to the APM/scripts folder on your SD card and make sure 

SCR_ENABLE = 1

Edit the arming_checks table if you want to change any of the checks from warnings to errors or errors to warnings or delete the row if you want to remove the check completely.
