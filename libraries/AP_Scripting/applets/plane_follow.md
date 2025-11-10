# Plane Follow

This script implements follow functionality for Plane. The plane must be
flying in fixed wing mode and will trigger on a scripting switch or AUX function.

The target plane must be connected via MAVLink and sending mavlink updates to the chase/follow plane running this script.

The MAVLINK_SYSID of the target must be set in FOLL_SYSID on the follow plane,
and ==must be different== from the MAVLINK_SYSID of the following plane.

| Parameter | Target Plane | Follow Plane |
| --------- | ------------ | ------------ |
| SYSID_THIS_MAV | 1 | 2 |
| FOLL_SYSID | n/a | 1 |


# Parameters

The script adds the following parameters to control it's behaviour. It uses 
the existing FOLL parameters that are used for the Copter FOLLOW mode. In addition
the following "FOLLP" parameters are added.

## FOLLP_FAIL_MODE

This is the mode the plane will change to if following fails. Failure happens
if the following plane loses telemetry from the target, or the distance exceeds
FOLL_DIST_MAX.

## FOLLP_EXIT_MODE

The flight mode the plane will switch to if it exits following. 

## FOLLP_ACT_FN

The scripting action that will trigger the plane to start following. When this
happens the plane will switch to GUIDED mode and the script will use guided mode
commands to steer the plane towards the target.

## FOLLP_TIMEOUT

If the target is lost, this is the timeout to wait to re-aquire the target before 
triggering FOLLP_FAIL_MODE

## FOLLP_OVRSHT_DEG

This is for the heuristic that uses the difference between the target vehicle heading
and the follow vehicle heading to determine if the vehicle has overshot and should slow
down and turn around. 75 degrees is a good start but tune for your circumstances.

## FOLLP_TURN_DEG

This is for the heuristic that uses the difference between the target vehicle heading
and the follow vehicle heading to determine if the target vehicle is executing a turn.
15 degrees is a good start but tune for your circumstances.

## FOLLP_DIST_CLOSE

One of the most important heuristics the follow logic uses to match the heading and speed
of the target plane is to trigger different behavior when the target location is "close".
How close is determined by this value, likely a larger number makes more sense for larger 
and faster vehicles and lower values for smaller and slower vehicles. Tune for your circumstances.

## FOLLP_ALT_OVR

The follow logic can have the follow vehicle track the altitude of the target, but setting a value
in FOLLP_ALT_OVR allows the follow vehicle to follow at a fixed altitude regardless of the altitude
of the target. The FOLLP_ALT_OVR is in meters in FOLL_ALT_TYPE frame. 

## FOLLP_D_P

The follow logic uses two PID controllers for controlling speed, the first uses distance (D) 
as the error. This is the P gain for the "D" PID controller.

## FOLLP_D_I

The follow logic uses two PID controllers for controlling speed, the first uses distance (D) 
as the error. This is the I gain for the "D" PID controller.

## FOLLP_D_D

The follow logic uses two PID controllers for controlling speed, the first uses distance (D) 
as the error. This is the D gain for the "D" PID controller.

## FOLLP_V_P

The follow logic uses two PID controllers for controlling speed, the first uses velocity (V) 
as the error. This is the P gain for the "V" PID controller.

## FOLLP_V_I

The follow logic uses two PID controllers for controlling speed, the first uses distance (V) 
as the error. This is the I gain for the "V" PID controller.

## FOLLP_V_D

The follow logic uses two PID controllers for controlling speed, the first uses distance (V) 
as the error. This is the D gain for the "V" PID controller.

## FOLLP_LKAHD

Time to "lookahead" when calculating distance errors.


# Operation
Enable Lua scripting by setting `SCR_ENABLE = 1` on the FOLLOW plane.

Install the plane_follow.lua script in the `APM/scripts` directory on the flight
controller's microSD card on the FOLLOW plane. s

Install the pid.lua, mavlink_attitude.lua files
in the `APM/scripts/modules` directory on the SD card on the FOLLOW plane.

Install the mavlink_msgs.lua files
in the `APM/scripts/modules/mavlink` directory on the SD card on the FOLLOW plane.

No scripts are required on the target plane.

Review the above parameter descriptions and decide on the right parameter values for your vehicle and operations.

Most of the follow logic is in AP_Follow library, which is part of the ArduPilot c++
code, so this script just calls the existing methods to do things like
lookup the SYSID of the vehicle to follow and calculate the direction and distance
to the target, which should ideally be another fixed wing plane, or VTOL in
fixed wing mode.

The target location the plane will attempt to achieve will be offset from the target
vehicle location by FOLL_OFS_X and FOLL_OFS_Y. FOLL_OFS_Z will be offset against the 
target vehicle, but also FOLL_ALT_TYPE will determine the altitude frame that the vehicle
will use when calculating the target altitude. See the definitions of these
parameters to understand how they work. FOLLP_ALT_OVR will override the operation of FOLL_OFS_Z
setting a fixed altitude for the following plane in FOLL_ALT_TYPE frame.

The existing FOLL_YAW_BEHAVE and FOLL_POS_P parameters are ignored by Plane Follow.

To ensure the follow plane gets timely updates from the target, the SRx_POSITION and SRx_EXTRA1
telemetry stream rate parameters should be increased to increase the rate that the POSITION_TARGET_GLOBAL_INT
and ATTITUDE mavlink messages are sent. The default value is 4Hz, a good value is probably 8Hz or 10Hz but 
some testing should be done to confirm the best rate for your telemetry radios and vehicles.

To prevent Mission Planner, QGC, MAVproxy or any other ground control station connected to your plane
from requesting a different stream rate, also set MAVx_OPTIONS bit 2 for the matching mavlink port. 

For example if your telemetry radio is connected to Telem1 = SERIAL1 then set
MAV1_OPTIONS = 4
MAV1_POSITION = 10
MAV1_EXTRA1 = 10

Ideally the connection is direct plane-to-plane and not routed via a Ground Control Station. This has been tested with 2x HolyBro SiK telemetry radios, one in each plane. RFD900 radios might work and LTE or other IP radio based connections will probably work well, but haven't been tested. Some users have reported using ESP32 WiFi modules configured with one of the radios set to be in station mode. Fast telemetry updates from the target to the following plane will give the best results.
