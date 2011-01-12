/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://pixhawk.ethz.ch/software/mavlink
 *	 Generated on Friday, January 7 2011, 10:04 UTC
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
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// ENUM DEFINITIONS


// MESSAGE DEFINITIONS

#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_boot.h"
#include "./mavlink_msg_system_time.h"
#include "./mavlink_msg_ping.h"
#include "./mavlink_msg_action.h"
#include "./mavlink_msg_action_ack.h"
#include "./mavlink_msg_set_mode.h"
#include "./mavlink_msg_set_nav_mode.h"
#include "./mavlink_msg_param_request_read.h"
#include "./mavlink_msg_param_request_list.h"
#include "./mavlink_msg_param_value.h"
#include "./mavlink_msg_param_set.h"
#include "./mavlink_msg_raw_imu.h"
#include "./mavlink_msg_raw_pressure.h"
#include "./mavlink_msg_attitude.h"
#include "./mavlink_msg_local_position.h"
#include "./mavlink_msg_gps_raw.h"
#include "./mavlink_msg_gps_status.h"
#include "./mavlink_msg_global_position.h"
#include "./mavlink_msg_sys_status.h"
#include "./mavlink_msg_rc_channels_raw.h"
#include "./mavlink_msg_rc_channels_scaled.h"
#include "./mavlink_msg_waypoint.h"
#include "./mavlink_msg_waypoint_request.h"
#include "./mavlink_msg_waypoint_set_current.h"
#include "./mavlink_msg_waypoint_current.h"
#include "./mavlink_msg_waypoint_request_list.h"
#include "./mavlink_msg_waypoint_count.h"
#include "./mavlink_msg_waypoint_clear_all.h"
#include "./mavlink_msg_waypoint_reached.h"
#include "./mavlink_msg_waypoint_ack.h"
#include "./mavlink_msg_waypoint_set_global_reference.h"
#include "./mavlink_msg_local_position_setpoint_set.h"
#include "./mavlink_msg_local_position_setpoint.h"
#include "./mavlink_msg_attitude_controller_output.h"
#include "./mavlink_msg_position_controller_output.h"
#include "./mavlink_msg_position_target.h"
#include "./mavlink_msg_state_correction.h"
#include "./mavlink_msg_set_altitude.h"
#include "./mavlink_msg_request_data_stream.h"
#include "./mavlink_msg_request_dynamic_gyro_calibration.h"
#include "./mavlink_msg_request_static_calibration.h"
#include "./mavlink_msg_manual_control.h"
#include "./mavlink_msg_debug_vect.h"
#include "./mavlink_msg_gps_local_origin_set.h"
#include "./mavlink_msg_airspeed.h"
#include "./mavlink_msg_global_position_int.h"
#include "./mavlink_msg_statustext.h"
#include "./mavlink_msg_debug.h"
#ifdef __cplusplus
}
#endif
#endif
