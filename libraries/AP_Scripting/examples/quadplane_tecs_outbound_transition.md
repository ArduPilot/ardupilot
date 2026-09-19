# Quadplane TECs Outbound Transition

This script implements a method to do a TECs controlled outbound transition in a quadplane. Based on
setting a flight plan with an intermediate waypoint to be used as the profile shaper, an outbound
transition can be flown at a hold speed for a hold time (similar to accelerating in ground effect).
Once a desired time has elapsed the waypoint will cycle to the next waypoint essentially setting
the climb profile to cruise speed. Transition will still occur at the previous transition speed.

A VTOL Takeoff altitude must be selected as higher than `TOT_NAVALT_MIN` or a normal VTOL Takeoff
and transition will be flown.

This script assumes the waypoint after takeoff acts as an intermediate waypoint to shape the
profile. VTOL Takeoff (1), WP(2), WP(3) should be in a straight line with the intention to not pass
WP(2).

The script will be reset at VTOL landing allowing for multiple takeoff and landings in a flight
plan. TOT_NAVALT_MIN is relative to the current altitude when VTOL_Takeoff is initiated.

Early transitions are protected against, as could be the case with a moving takeoff and wind shear
where the vehicle may accelerate through the hold speed setpoint. In this case, the vehicle
transitions and the waypoint is revved to the next waypoint in the flight plan.

# Parameters

This script adds 4 parameters to control its behavior. The parameters are:

## TOT_ENABLE

This must be set to 1 to enable the script.

## TOT_HOLD_TIMER_S

The time in seconds to hold a reduced airspeed. This is useful for accelerating in ground effect to
a speed less than cruise speed or setting a specific climb speed that could be desired to be
assisted by VTOL motors.

## TOT_HOLD_SPD_M_S

The desired airspeed during the hold timer in meters/second.

## TOT_NAVALT_MIN

The minimum altitude to begin the maneuver in AGL, meters (above ground level, meters). Under this
altitude, normal VTOL Takeoff will be performed. This will always be at least `Q_NAVALT_MIN` checked
at script boot.

# Operation

Install the script in the APM/SCRIPTS folder on your microSD (you can
use mavFTP for that). Then reboot and re-fetch parameters. You will
find you now have several TOT parameters.

Set TOT_ENABLE to 1 and the other parameters as desired to shape the outbound transition.

Now reboot to start the script.

An example flight plan could be:

* VTOL takeoff to 20 meters
* Cruise waypoint set at 10 meters
* Cruise waypoint set at 20 meters
* ...

The aircraft profile would look like:

* VTOL takeoff to `min(TOT_NAVALT_MIN, Q_NAVALT_MIN)`
* Forward translation begins using TECs to `TOT_HOLD_SPD_M_S`
* Start a hold timer
* Set `TRIM_ARSPD_CM` and `ARSPD_FWB_MIN` to the hold speed
* Cycle waypoint to WP2 (this sets the altitude desired and thus profile via TECs, VTOL is
protected by `Q_ASSIST_SPEED` which is unmodified here)
* Hold timer expires
* Reset `TRIM_ARSPD_CM` and `ARSPD_FWB_MIN` to values at script boot
* Airspeed is now trying to reach cruise setpoint
* Cruise airspeed is reached, cycle to WP3 which sets cruise pattern (this is to allow a
different altitude during the hold timer)

Note: the waypoints will cycle as usual if passed. The waypoint will always cycle once transition
airspeed is reached.
