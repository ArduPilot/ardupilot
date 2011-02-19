/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://pixhawk.ethz.ch/software/mavlink
 *	 Generated on Friday, February 11 2011, 07:42 UTC
 */
#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif


#include "../protocol.h"

#define MAVLINK_ENABLED_COMMON

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// ENUM DEFINITIONS

/** @brief Commands to be executed by the MAV. They can be executed on user request,       or as part of a mission script. If the action is used in a mission, the parameter mapping       to the waypoint/mission message is as follows:       Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7.        */
enum MAV_CMD
{
	MAV_CMD_NAV_WAYPOINT=16, 		/* Navigate to waypoint.Hold time (ignored by fixed wing, time to stay at waypoint for rotary wing)Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)0 to pass through the WP, if > 0 radius to pass by WP. Allows trajectory control.Desired yaw angleLatitudeLongitudeAltitude*/
	MAV_CMD_NAV_LOITER_UNLIM=17, 	/* Loiter around this waypoint an unlimited amount of timeEmptyEmptyRadius around waypoint, in meters. If positive loiter clockwise, else counter-clockwiseDesired yaw angle.LatitudeLongitudeAltitude*/
	MAV_CMD_NAV_LOITER_TURNS=18, 	/* Loiter around this waypoint for X turnsTurnsEmptyRadius around waypoint, in meters. If positive loiter clockwise, else counter-clockwiseDesired yaw angle.LatitudeLongitudeAltitude*/
	MAV_CMD_NAV_LOITER_TIME=19, 	/* Loiter around this waypoint for X secondsSeconds (decimal)EmptyRadius around waypoint, in meters. If positive loiter clockwise, else counter-clockwiseDesired yaw angle.LatitudeLongitudeAltitude*/
	MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch locationEmptyEmptyEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_NAV_LAND=21, 			/* Land at locationEmptyEmptyEmptyDesired yaw angle.LatitudeLongitudeAltitude*/
	MAV_CMD_NAV_TAKEOFF=22, 		/* Takeoff from ground / handMinimum pitch (if airspeed sensor present), desired pitch without sensorEmptyEmptyYaw angle (if magnetometer present), ignored without magnetometerLatitudeLongitudeAltitude*/
	MAV_CMD_NAV_R_WAYPOINT=23, 		/* Navigate to waypoint.Hold time (ignored by fixed wing, time to stay at waypoint for rotary wing)Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)0 to pass through the WP, if > 0 radius to pass by WP. Allows trajectory control.Desired yaw angleLatitudeLongitudeAltitude*/
	MAV_CMD_NAV_LAST=95, 			/* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumerationEmptyEmptyEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_CONDITION_DELAY=112, 	/* Delay mission state machine.Delay in seconds (decimal)EmptyEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached.Descent / Ascend rate (m/s)EmptyEmptyEmptyEmptyEmptyFinish Altitude*/
	MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point.Distance (meters)EmptyEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_CONDITION_LAST=159, 	/* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumerationEmptyEmptyEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_CONDITION_ANGLE=160, 	/* */
	MAV_CMD_DO_SET_MODE=176, 		/* Set system mode.Mode, as defined by ENUM MAV_MODEEmptyEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_JUMP=177, 			/* Jump to the desired command in the mission list.  Repeat this action only the specified number of timesSequence numberRepeat countEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_CHANGE_SPEED=178, 	/* Change speed and/or throttle set points.Speed type (0=Airspeed, 1=Ground Speed)Speed  (m/s, -1 indicates no change)Throttle  ( Percent, -1 indicates no change)EmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_SET_HOME=179, 		/* Changes the home location either to the current location or a specified location.Use current (1=use current location, 0=use specified location)EmptyEmptyEmptyLatitudeLongitudeAltitude*/
	MAV_CMD_DO_SET_PARAMETER=180, 	/* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.Parameter numberParameter valueEmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_SET_RELAY=181, 		/* Set a relay to a condition.Relay numberSetting (1=on, 0=off, others possible depending on system hardware)EmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_REPEAT_RELAY=182, 	/* Cycle a relay on and off for a desired number of cyles with a desired period.Relay numberCycle countCycle time (seconds, decimal)EmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_SET_SERVO=183, 		/* Set a servo to a desired PWM value.Servo numberPWM (microseconds, 1000 to 2000 typical)EmptyEmptyEmptyEmptyEmpty*/
	MAV_CMD_DO_REPEAT_SERVO=184, 	/* Cycle a between its nominal setting and a desired PWM for a desired number of cyles with a desired period.Servo numberPWM (microseconds, 1000 to 2000 typical)Cycle countCycle time (seconds)EmptyEmptyEmpty*/
	MAV_CMD_ENUM_END
};

/** @brief Data stream IDs. A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages.       */
enum MAV_DATA_STREAM
{
	MAV_DATA_STREAM_ALL=0, /* Enable all data streams*/
	MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.*/
	MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS*/
	MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW*/
	MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.*/
	MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.*/
	MAV_DATA_STREAM_EXTRA1=250, /* Dependent on the autopilot*/
	MAV_DATA_STREAM_EXTRA2=251, /* Dependent on the autopilot*/
	MAV_DATA_STREAM_EXTRA3=252, /* Dependent on the autopilot*/
	MAV_DATA_STREAM_ENUM_END
};


// MESSAGE DEFINITIONS

#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_boot.h"
#include "./mavlink_msg_system_time.h"
#include "./mavlink_msg_ping.h"
#include "./mavlink_msg_system_time_utc.h"
#include "./mavlink_msg_action_ack.h"
#include "./mavlink_msg_action.h"
#include "./mavlink_msg_set_mode.h"
#include "./mavlink_msg_set_nav_mode.h"
#include "./mavlink_msg_param_request_read.h"
#include "./mavlink_msg_param_request_list.h"
#include "./mavlink_msg_param_value.h"
#include "./mavlink_msg_param_set.h"
#include "./mavlink_msg_gps_raw_int.h"
#include "./mavlink_msg_scaled_imu.h"
#include "./mavlink_msg_gps_status.h"
#include "./mavlink_msg_raw_imu.h"
#include "./mavlink_msg_raw_pressure.h"
#include "./mavlink_msg_attitude.h"
#include "./mavlink_msg_local_position.h"
#include "./mavlink_msg_global_position.h"
#include "./mavlink_msg_gps_raw.h"
#include "./mavlink_msg_sys_status.h"
#include "./mavlink_msg_rc_channels_raw.h"
#include "./mavlink_msg_rc_channels_scaled.h"
#include "./mavlink_msg_servo_output_raw.h"
#include "./mavlink_msg_waypoint.h"
#include "./mavlink_msg_waypoint_request.h"
#include "./mavlink_msg_waypoint_set_current.h"
#include "./mavlink_msg_waypoint_current.h"
#include "./mavlink_msg_waypoint_request_list.h"
#include "./mavlink_msg_waypoint_count.h"
#include "./mavlink_msg_waypoint_clear_all.h"
#include "./mavlink_msg_waypoint_reached.h"
#include "./mavlink_msg_waypoint_ack.h"
#include "./mavlink_msg_gps_set_global_origin.h"
#include "./mavlink_msg_gps_local_origin_set.h"
#include "./mavlink_msg_local_position_setpoint_set.h"
#include "./mavlink_msg_local_position_setpoint.h"
#include "./mavlink_msg_control_status.h"
#include "./mavlink_msg_safety_set_allowed_area.h"
#include "./mavlink_msg_safety_allowed_area.h"
#include "./mavlink_msg_attitude_controller_output.h"
#include "./mavlink_msg_position_controller_output.h"
#include "./mavlink_msg_nav_controller_output.h"
#include "./mavlink_msg_position_target.h"
#include "./mavlink_msg_state_correction.h"
#include "./mavlink_msg_set_altitude.h"
#include "./mavlink_msg_request_data_stream.h"
#include "./mavlink_msg_manual_control.h"
#include "./mavlink_msg_global_position_int.h"
#include "./mavlink_msg_vfr_hud.h"
#include "./mavlink_msg_debug_vect.h"
#include "./mavlink_msg_named_value_float.h"
#include "./mavlink_msg_named_value_int.h"
#include "./mavlink_msg_statustext.h"
#include "./mavlink_msg_debug.h"
#ifdef __cplusplus
}
#endif
#endif
