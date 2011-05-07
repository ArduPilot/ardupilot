/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://pixhawk.ethz.ch/software/mavlink
 *	 Generated on Saturday, April 16 2011, 04:02 UTC
 */
#ifndef UALBERTA_H
#define UALBERTA_H

#ifdef __cplusplus
extern "C" {
#endif


#include "../protocol.h"

#define MAVLINK_ENABLED_UALBERTA


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


// MESSAGE DEFINITIONS

#include "./mavlink_msg_nav_filter_bias.h"
#include "./mavlink_msg_request_rc_channels.h"
#include "./mavlink_msg_radio_calibration.h"


// MESSAGE LENGTHS

#undef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS { 3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 0, 0, 0, 0, 0, 5, 5, 26, 16, 36, 5, 6, 0, 0, 21, 0, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 1, 84, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51 }

#ifdef __cplusplus
}
#endif
#endif
