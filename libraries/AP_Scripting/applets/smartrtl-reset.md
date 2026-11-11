# SmartRTL Return Point Reset

This script resets the SmartRTL return point in flight from an RC
switch. When the switch is moved to its high position the SmartRTL
breadcrumb path recorded so far is discarded and the vehicle's current
position becomes the point SmartRTL will return to and land at.

The script is mainly intended for vehicles that navigate without GPS,
for example with optical flow and a rangefinder. The position estimate
of such a vehicle drifts over time, so the oldest part of the SmartRTL
path, including the original return point, is the least accurate part
of it. Resetting the return point at a known safe location keeps the
return journey short and accurate. The reset also re-activates SmartRTL
if it had been deactivated earlier in the flight because of a bad
position estimate or a full path buffer.

The script works on Copter, Heli and Rover.

## Parameters

The script adds the following parameters to control its behaviour.

### SRTL_RST_ENABLE

This must be set to 1 to enable the script.

### SRTL_RST_FN

The RCx_OPTION function number of the switch used to reset the return
point, one of the scripting functions 300 to 315. This defaults to 300
(Scripting1). Set one of the RCx_OPTION parameters to this value to
select the switch. The same function can be triggered from a ground
station with MAV_CMD_DO_AUX_FUNCTION.

## Operation

Install the lua script in the APM/SCRIPTS directory on the flight
controller's microSD card, enable scripting with SCR_ENABLE=1 and
reboot. Set RCx_OPTION to 300 (or the value of SRTL_RST_FN) for the
switch you want to use.

SmartRTL must be enabled (SRTL_POINTS greater than 0) and the vehicle
must have a good position estimate. With optical flow this requires the
EKF to be configured for optical flow navigation (for example
EK3_SRC1_POSXY=0 and EK3_SRC1_VELXY=5), a rangefinder and the optical
flow sensor to be healthy.

While armed and flying, move the switch to its high position. The
vehicle reports "SmartRTL: return point reset" when the reset is done.
The reset is refused, with a message explaining why, while the vehicle
is disarmed (arming resets the path anyway), while it is in SmartRTL
mode, or when it has no good position estimate. The switch is read
through the firmware's debounced auxiliary switch handling, which
ignores RC input during a radio failsafe.

After the reset, switching to SmartRTL mode retraces the path recorded
since the reset and lands at the reset point.

Only SmartRTL uses the new return point. RTL, and the RTL fallback when
SmartRTL is not available, still go to the home location, and the
ground station map does not show the reset point.
