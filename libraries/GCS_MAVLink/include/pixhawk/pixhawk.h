/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://qgroundcontrol.org/mavlink/
 *	 Generated on Friday, August 5 2011, 07:37 UTC
 */
#ifndef PIXHAWK_H
#define PIXHAWK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "../protocol.h"

#define MAVLINK_ENABLED_PIXHAWK


#include "../common/common.h"
// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 0
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 0
#endif

// ENUM DEFINITIONS

/** @brief  Content Types for data transmission handshake */
enum DATA_TYPES
{
	DATA_TYPE_JPEG_IMAGE=1,
	DATA_TYPE_RAW_IMAGE=2,
	DATA_TYPE_KINECT=3,
	DATA_TYPES_ENUM_END
};


// MESSAGE DEFINITIONS

#include "./mavlink_msg_attitude_control.h"
#include "./mavlink_msg_set_cam_shutter.h"
#include "./mavlink_msg_image_triggered.h"
#include "./mavlink_msg_image_trigger_control.h"
#include "./mavlink_msg_image_available.h"
#include "./mavlink_msg_vision_position_estimate.h"
#include "./mavlink_msg_vicon_position_estimate.h"
#include "./mavlink_msg_vision_speed_estimate.h"
#include "./mavlink_msg_position_control_setpoint_set.h"
#include "./mavlink_msg_position_control_offset_set.h"
#include "./mavlink_msg_position_control_setpoint.h"
#include "./mavlink_msg_marker.h"
#include "./mavlink_msg_raw_aux.h"
#include "./mavlink_msg_aux_status.h"
#include "./mavlink_msg_watchdog_heartbeat.h"
#include "./mavlink_msg_watchdog_process_info.h"
#include "./mavlink_msg_watchdog_process_status.h"
#include "./mavlink_msg_watchdog_command.h"
#include "./mavlink_msg_pattern_detected.h"
#include "./mavlink_msg_point_of_interest.h"
#include "./mavlink_msg_point_of_interest_connection.h"
#include "./mavlink_msg_data_transmission_handshake.h"
#include "./mavlink_msg_encapsulated_data.h"
#include "./mavlink_msg_brief_feature.h"


// MESSAGE LENGTHS

#undef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS { 3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 14, 14, 0, 0, 0, 5, 5, 26, 16, 36, 5, 6, 56, 0, 21, 18, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 52, 1, 92, 0, 0, 0, 0, 0, 0, 0, 32, 32, 20, 0, 0, 0, 0, 0, 0, 20, 18, 0, 0, 0, 0, 0, 0, 0, 0, 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 12, 0, 0, 0, 0, 0, 0, 0, 4, 255, 12, 6, 18, 0, 0, 0, 0, 0, 106, 42, 54, 0, 0, 0, 0, 0, 0, 0, 8, 255, 53, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51 }

#ifdef __cplusplus
}
#endif
#endif
