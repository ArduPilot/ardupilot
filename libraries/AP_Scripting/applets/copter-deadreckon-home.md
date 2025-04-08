# Copter Deadreckon Home applet

Copter attempts to fly home using dead reckoning if the GPS quality deteriorates or an EKF failsafe triggers

CAUTION: This script only works for Copter 4.3 (and higher)

Deadreckoning will only be activated while the vehicle is in autonomous modes (e.g. Auto, Guided, RTL, Circle, Land, Brake, SmartRTL or AutoRTL modes).  Deadreckoning is not activated in pilot controlled modes (e.g. Loiter, PosHold, etc) because we assume the pilot could simply switch to AltHold and fly the vehicle home manually.

## Parmeter Descriptions

  - DR_ENABLE : 1 = enabled, 0 = disabled
  - DR_ENABLE_DIST : distance from home (in meters) beyond which the dead reckoning will be enabled
  - DR_GPS_SACC_MAX : GPS speed accuracy maximum, above which deadreckoning home will begin (default is 0.8).  Lower values trigger with good GPS quality, higher values will allow poorer GPS before triggering. Set to 0 to disable use of GPS speed accuracy.
  - DR_GPS_SAT_MIN : GPS satellite count threshold below which deadreckoning home will begin (default is 6).  Higher values trigger with good GPS quality, Lower values trigger with worse GPS quality. Set to 0 to disable use of GPS satellite count.
  - DR_GPS_TRIGG_SEC : GPS checks must fail for this many seconds before dead reckoning will be triggered.
  - DR_FLY_ANGLE : lean angle (in degrees) during deadreckoning.  Most vehicles reach maximum speed at 22deg
  - DR_FLY_ALT_MIN : min alt (above home in meters) during deadreckoning. zero to return at current alt
  - DR_FLY_TIMEOUT : timeout (in seconds).  Vehicle will attempt to switch to NEXT_MODE after this many seconds of deadreckoning.  If it cannot switch modes it will continue in Guided_NoGPS.  Set to 0 to disable timeout
  - DR_NEXT_MODE : flight mode vehicle will change to when GPS / EKF recovers or DR_FLY_TIMEOUT expires.  Default is 6=RTL, see FLTMODE1 parameter description for list of flight mode number.  Set to -1 to return to mode used before deadreckoning was triggered

## How to use

  1. set SCR_ENABLE = 1 to enable scripting (and reboot the autopilot)
  2. set SCR_HEAP_SIZE to 80000 or higher to allocate enough memory for this script
  3. set DR_ENABLE = 1 to enable dead reckoning
  4. optionally set DR_GPS_SACC_MAX and/or DR_GPS_SAT_MIN parameters to adjust how bad the GPS quality must be before triggering
  5. confirm "DR: waiting for dist (Xm < 50m)" message is displayed on ground station (so you know script is working)
  6. arm and takeoff to a safe altitude
  7. fly at least DR_ENABLE_DIST meters from home and confirm "DR: activated!" is displayed on ground station

  If this script senses low GPS quality or an EKF failsafe triggers

  - vehicle will change to Guided_NoGPS mode
  - vehicle will lean in the last known direction of home (see DR_FLY_ANGLE)
  - if GPS recovers or EKF failsafe is cleared the vehicle will switch to DR_NEXT_MODE (if -1 then it will switch back to the mode in use before the GPS/EKF failure)
  - if the timeout is surpassed (see DR_FLY_TIMEOUT) the vehicle will try to switch to DR_NEXT_MODE.  If it fails to change it will continue in Guided_NoGPS but keep trying to change mode
  - the pilot can retake control by switching to an "unprotected" mode like AltHold, Loiter (see "protected_mode_array" below)

## Testing in SITL

  - set map setshowsimpos 1 (to allow seeing where vehicle really is in simulator even with GPS disabled)
  - set SIM_GPS1_ENABLE = 0 to disable GPS (confirm dead reckoning begins)
  - set SIM_GPS1_ENABLE = 1 to re-enable GPS
  - set SIM_GPS_NUMSAT = 3 to lower simulated satellite count to confirm script triggers
  - set DR_GPS_SACC_MAX = 0.01 to lower the threshold and trigger below the simulator value which is 0.04 (remember to set this back after testing!)

## Test on a real vehicle

  1. set DR_FLY_TIMEOUT to a low value (e.g. 5 seconds)
  2. fly the vehicle at least DR_DIST_MIN meters from home and confirm the "DR: activated!" message is displayed
  3. set GPS1_TYPE = 0 to disable GPS and confirm the vehicle begins deadreckoning after a few seconds
  4. restore GPS1_TYPE to its original value (normally 1) and confirm the vehicle switches to DR_NEXT_MODE
  5. restore DR_FLY_TIMEOUT to a higher value for real-world use

  Note: Instaed of setting GPS1_TYPE, an auxiliary function switch can be setup to disable the GPS (e.g. RC9_OPTION = 65/"Disable GPS")

## Testing that it does not require RC (in SITL):
  - set FS_OPTIONS's "Continue if in Guided on RC failsafe" bit
  - set FS_GCS_ENABLE = 1 (to enable GCS failsafe otherwise RC failsafe will trigger anyway)
  - optionally set MAV_GCS_SYSID = 77 (or almost any other unused system id) to trick the above check so that GCS failsafe can really be disabled
  - set SIM_RC_FAIL = 1 (to simulate RC failure, note vehicle keeps flying)
  - set SIM_RC_FAIL = 0 (to simulate RC recovery)

## Test with wind (in SITL)
  - SIM_WIND_DIR <-- sets direction wind is coming from
  - SIM_WIND_SPD <-- sets wind speed in m/s
