# QuadPlane Terrain Avoidance

 This script will detect if a quadplane following an Auto mission is likely to hit elevated terrain, such as
 a small hill, cliff edge, high trees or other obstacles that might not show up in the OOTB STRM terrain model.
 The code will attempt to avoid the impact by:-
 - "Pitching" up if the plane can safely fly over the obstacle 
 - "Quading" up by switching to QuadPlane loiter mode (Quading) and gaining altitude using VTOL motors 
 - "CMTC" - can't make that climb. If the approaching terrain is higher than the plane can climb based on the 
    configured PTCH_LIM_MAX_DEG (divided by 2.0), then loiter to altitude before continuing.

This code works best long range rangefinders such as the LightWare long range lidars that can measure 
   distances up to 90-95 meters away. The CMT function only uses terrain data.

 The terrain avoidance will be on by default but will not function at "home" or within TA_HOME_DIST meters
 of home. The scripting function TA_ACT_FN can be used to disable terrain folling at any time
 Terrain following will operate in modes Auto, Gukded, RTL and QRTL.

The "Can't make that climb" (CMTC) feature will prevent ArduPlane from flying into terrain it does know about
by calculating the required pitch to avoid terrain between the current location and the next waypoint including
all points in between. If the pitch required is > PTCH_LIM_MAX_DEG / 2 then the code will perform a loiter to
altitude, using fixed wing loiter, to acheive a safe altitude to avoid the terrain before continuing the mission. 

For CMTC remember to turn it on (defaults to off), also set TA_CMTC_HGT (height above terrain for CMTC to try to acheive) and TA_ALT_MAX (max altitude to fly).
Aso set TA_CMTC_RAD to a smaller radius than WP_LOITER_RAD. Make sure that the plane can acheive TA_CMTC_RAD based on the ROLL_LIMIT_MAX.

Note: 

- Q_ASSIST should also be configured for best results TA_PTCH_DWN_MIN > Q_ASSIST_ALT > TA_QUAD_DWN_MIN
- This script uses RC overrides. If you have used RC_OPTIONS (bit 1) to disable RC overrides, then this script will not work correctly.

# Parameters

Beyond the normal Q_ASSIST parameters the script adds several additional parameters to
control it's behaviour. The parameters are prefixed with ZTA and ZTB. The parameters are:

## TA_ACT_FN 

An RC scripting function to disable terrain avoidance. It defaults to on.

## TA_HOME_DIST

A circle around home (in meters), where terrain avoidance will not run.
Allows for safe takeoff and landing at the home location.

## TA_ALT_MAX

Maximum altitude above terrain that the plane can go to if avoiding terrain.
Set this to avoid "flyways" usually caused by malfunctioning rangefinders.

## TA_PTCH_DWN_MIN

The minimum distance to the ground directly when pitching will start.

# TA_PTCH_FWD_MIN

The minimum distance forward where pitching will start. This requires
a forward facing range finder best installed pointing at a 45 degree
angle.

# TA_PTCH_GSP_MIN

The minimum groundspeed to use pitching. If groundspeed is below this then
pitching will not be attempted.

## TA_QUAD_DWN_MIN

The minimum distance to the ground directly when quading will start.
Should be lower than TA_PTCH_DWN_MIN by at least 5m.

# TA_QUAD_FWD_MIN

The minimum distance forward where quadinging will start. This requires
a forward facing range finder best installed pointing at a 45 degree
angle. (the same one used by TA_PTCH_FWD_MIN)

# TA_GSP_MAX

The maximum groundspeed to attempt to fly. For best results when doing
magnetometry surveys ideally a steady groundspeed is required even 
in windy conditions. This attempts to acheive that.

# TA_GSP_AIRBRAKE

If the vehicle exceeds ZTB_GSP_MAX and slowing the motors (desired airspeed)
isn't working then if this is set to 1, the script will attempt to use QHOVER
to reduce airspeed. This doesn't work very well, so test it for your use case.
It defaults to off.

# TA_CMTC_ENABLE

Enable the Can't Make That Climb (CMTC) feature, which will circle to gain altitude if 
the required pitch up to the next waypoint exceeds PTCH_LIM_MAX_DEG / 2.

# TA_CMTC_HGT

If CMTC is enabled, uses this height as the clearance required above terrain altitude to 
use for CMTC calculation. If the plane can't make this number of meters clearance above the
terrain between the current location and the next waypoint then CMTC will be engaged.

# TA_CMTC_RAD

When loitering to gain altitude if CMTC is triggered, use this as the loiter radius. If not set 
or is <= 0 then use WP_LOITER_RAD. Should normally be set lower than WP_LOITER_RAD.


# Operation

Good TECS tuning of your aircraft is essential. The script relies
on TECS to do all the work. Some parameters it refers to directly.

This script operates by default and can be turned off in flight with
a switch, or will disable it self if close to home (TA_HOME_DIST meters).

Install the quadplane_terrain_avoid.lua script in the APM/scripts folder on the SD card on your autopilot. Install the mavlink_wrappers.lua module in the APM/scripts/modules folder on the same SD card. Configure
the parameters and set your transmitter with switch for TA_ACT_FN to allow disabling the function in the air.


