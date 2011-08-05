/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://qgroundcontrol.org/mavlink/
 *	 Generated on Friday, August 5 2011, 07:37 UTC
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

/** @brief  Available autopilot modes for ualberta uav */
enum UALBERTA_AUTOPILOT_MODE
{
	MODE_MANUAL_DIRECT=0, /*  */
	MODE_MANUAL_SCALED=1, /*  */
	MODE_AUTO_PID_ATT=2, /*  */
	MODE_AUTO_PID_VEL=3, /*  */
	MODE_AUTO_PID_POS=4, /*  */
	UALBERTA_AUTOPILOT_MODE_ENUM_END
};

/** @brief  Navigation filter mode */
enum UALBERTA_NAV_MODE
{
	NAV_AHRS_INIT=0,
	NAV_AHRS=1, /*  */
	NAV_INS_GPS_INIT=2, /*  */
	NAV_INS_GPS=3, /*  */
	UALBERTA_NAV_MODE_ENUM_END
};

/** @brief  Mode currently commanded by pilot */
enum UALBERTA_PILOT_MODE
{
	PILOT_MANUAL=0, /*  */
	PILOT_AUTO=1, /*  */
	PILOT_ROTO=2, /*  */
	UALBERTA_PILOT_MODE_ENUM_END
};


// MESSAGE DEFINITIONS

#include "./mavlink_msg_nav_filter_bias.h"
#include "./mavlink_msg_radio_calibration.h"
#include "./mavlink_msg_ualberta_sys_status.h"


// MESSAGE LENGTHS

#undef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS { 3, 4, 8, 14, 8, 28, 3, 32, 0, 2, 3, 2, 2, 0, 0, 0, 0, 0, 0, 0, 19, 2, 23, 21, 0, 37, 26, 101, 26, 16, 32, 32, 37, 32, 11, 17, 17, 16, 18, 36, 4, 4, 2, 2, 4, 2, 2, 3, 14, 12, 18, 16, 8, 27, 25, 14, 14, 0, 0, 0, 5, 5, 26, 16, 36, 5, 6, 56, 0, 21, 18, 0, 0, 18, 20, 20, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 32, 42, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 30, 14, 14, 51 }

#ifdef __cplusplus
}
#endif
#endif
