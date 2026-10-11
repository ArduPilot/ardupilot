# GNSS Loss ExternalNav Fallback

Switches the EKF to an ExternalNav source set (for example a visual odometry camera or a motion capture system) when the GPS is lost or degraded while armed. After the switch it watches the ExternalNav source and changes mode if its quality drops, a time limit is reached, or it stops.

The existing source switching scripts choose a source automatically only while the pilot holds an RCx_OPTION = 300 (Scripting1) switch high. This script needs no RC input, and it adds the mode changes that decide what the vehicle does once it no longer has GPS.

This script is for Copter (including Heli). On other vehicles it displays "GNSSFB: Copter only, script stopped" and does nothing.

CAUTION: Test this script in SITL, then on a real vehicle with GNSSFB_ENABLE set while flying in an open area, before relying on it.

## Parameter Descriptions

- GNSSFB_ENABLE : 1 = enabled, 0 = disabled (default)
- GNSSFB_SRC_SET : the EKF source set to switch to, 2 = `EK3_SRC2_` (default), 3 = `EK3_SRC3_`
- GNSSFB_LOSS_TIME : seconds the primary GPS must report less than a 3D fix, or send nothing, before switching (default 0.3). A GPS that has sent nothing for 1 second counts as having no fix
- GNSSFB_SAT_MIN : the GPS is degraded while it reports fewer satellites than this (default 6). 0 disables this check
- GNSSFB_HACC_MAX : the GPS is degraded while its reported horizontal accuracy is worse than this many meters (default 5). 0 disables this check
- GNSSFB_DEGR_TIME : seconds the GPS must be degraded, or without a fix, before switching (default 2)
- GNSSFB_XN_QUAL : after the switch, change to GNSSFB_RET_MODE if the ExternalNav quality is from 1 to below this value (default 0, disabled). A quality of 0 means unknown and is ignored
- GNSSFB_XN_TOUT : after the switch, change to GNSSFB_LND_MODE if the ExternalNav source is unusable for this many seconds (default 0.5). It is unusable when it has sent nothing recently, when it reports a quality of -1 (failed), or when its quality is below VISO_QUAL_MIN so the EKF ignores it
- GNSSFB_XN_TMAX : after the switch, change to GNSSFB_RET_MODE after this many seconds on the ExternalNav source set (default 0, disabled)
- GNSSFB_RET_MODE : mode number used when the ExternalNav quality is low or the time limit is reached (default 6, RTL). -1 announces without changing mode
- GNSSFB_LND_MODE : mode number used when the ExternalNav source is lost (default 9, LAND). -1 announces without changing mode

## How to use

1. Set up the ExternalNav source and check that it works on its own, see [ArduPilot's non-GPS navigation pages](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html)
2. Set SCR_ENABLE = 1 and reboot the autopilot
3. Copy this script to the autopilot's SD card in the APM/scripts directory and reboot
4. Set the EKF source sets so that GPS is the primary and ExternalNav the secondary, for example:
    - EK3_SRC1_POSXY = 3 (GPS), EK3_SRC1_VELXY = 3 (GPS), EK3_SRC1_POSZ = 1 (Baro), EK3_SRC1_VELZ = 3 (GPS), EK3_SRC1_YAW = 1 (Compass)
    - EK3_SRC2_POSXY = 6 (ExternalNav), EK3_SRC2_VELXY = 6 (ExternalNav), EK3_SRC2_POSZ = 1 (Baro), EK3_SRC2_VELZ = 6 (ExternalNav), EK3_SRC2_YAW = 1 (Compass)
    - EK3_SRC_OPTIONS = 0
5. Set GNSSFB_ENABLE = 1 and reboot. "GNSSFB: loaded" is displayed on the ground station
6. Optionally set GNSSFB_XN_QUAL and GNSSFB_XN_TMAX. Good values depend on the ExternalNav source; for example 50 and 120

While enabled, the script blocks arming unless AHRS_EKF_TYPE is 3, VISO_TYPE is set and the chosen source set's EK3_SRCn_POSXY is 6 (ExternalNav). The pre-arm message says which one is wrong, for example "PreArm: GNSSFB: EK3_SRC2_POSXY not ExternalNav".

## What the script does

While armed, and only after the primary GPS has had a 3D fix since arming:

- if the GPS has no 3D fix (or has sent nothing for 1 second) for GNSSFB_LOSS_TIME, it switches to GNSSFB_SRC_SET and displays "GNSSFB: GNSS lost, source set 2"
- if the GPS is degraded, or keeps dropping in and out of a 3D fix, for GNSSFB_DEGR_TIME, it switches and displays "GNSSFB: GNSS degraded, source set 2"
- if the ExternalNav quality is low, or the time limit is reached, it changes to GNSSFB_RET_MODE once and displays "GNSSFB: ExtNav quality low, mode 6" (or "ExtNav time limit")
- if the ExternalNav source is unusable it changes to GNSSFB_LND_MODE once and displays "GNSSFB: ExtNav lost, mode 9". This can follow the return mode change, but not the other way round

If a mode change is refused the script displays "mode 6 refused" (for example) once and keeps trying.

The script makes mode changes only while the vehicle is still in the mode it was in when the script switched sources (or in the script's own return mode). If the pilot, the ground station or a failsafe changes mode after the switch, the script displays "GNSSFB: mode changed, no more mode changes" and leaves the modes alone until disarmed. The vehicle's own EKF failsafe still applies.

The script does not switch back to GPS in flight. The pilot can do that with an RCx_OPTION = 90 (EKF Source Set) switch; that switch acts when its position changes, so if it is already low it has to be moved out of low and back. If anything else changes the source set after the switch, the script displays "GNSSFB: source set changed, standing down" and does nothing more until disarmed.

The script does nothing while the source set is not the primary one, so a pilot who has already switched sources with an RC switch is not overridden.

On disarm the script returns to the primary source set if it was the one that switched (even if GNSSFB_ENABLE was set to 0 in flight), displays "GNSSFB: disarmed, back to source set 1" and starts again from the beginning.

## Limitations

- The script watches the ExternalNav messages, not the EKF. A source that keeps sending velocity but stops sending position still looks usable to the script. The EKF failsafe remains the backstop for that case
- The script watches only the primary GPS (see GPS_PRIMARY and GPS_AUTO_SWITCH)

## Testing in SITL

The Copter autotest GNSSLossExtNavFallback runs these steps using the simulated Vicon system:

1. start SITL with the simulated Vicon on serial 5 (`sim_vehicle.py -v ArduCopter -A "--serial5=sim:vicon:"`) and set VISO_TYPE = 2, SERIAL5_PROTOCOL = 2 and SIM_VICON_TMASK = 16 (ODOMETRY messages, which carry a quality)
2. set the source sets as above, then take off in Loiter
3. set SIM_GPS1_ENABLE = 0 to lose the GPS fix, SIM_GPS1_TYPE = 0 to make the GPS go silent, or SIM_GPS1_NUMSATS = 4 to degrade it, and check the vehicle holds position on ExternalNav
4. set SIM_VICON_QUAL to a value below GNSSFB_XN_QUAL to trigger the return mode, or set GNSSFB_XN_TMAX to test the time limit
5. set SIM_VICON_FAIL = 1, or SIM_VICON_QUAL = -1, to lose the ExternalNav source and trigger the land mode
6. change mode after the switch and check the script no longer changes mode
