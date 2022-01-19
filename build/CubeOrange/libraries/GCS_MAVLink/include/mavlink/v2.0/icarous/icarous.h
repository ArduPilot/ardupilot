/** @file
 *  @brief MAVLink comm protocol generated from icarous.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_ICAROUS_H
#define MAVLINK_ICAROUS_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ICAROUS.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 3

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{42000, 227, 1, 1, 0, 0, 0}, {42001, 239, 46, 46, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ICAROUS

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_ICAROUS_TRACK_BAND_TYPES
#define HAVE_ENUM_ICAROUS_TRACK_BAND_TYPES
typedef enum ICAROUS_TRACK_BAND_TYPES
{
   ICAROUS_TRACK_BAND_TYPE_NONE=0, /*  | */
   ICAROUS_TRACK_BAND_TYPE_NEAR=1, /*  | */
   ICAROUS_TRACK_BAND_TYPE_RECOVERY=2, /*  | */
   ICAROUS_TRACK_BAND_TYPES_ENUM_END=3, /*  | */
} ICAROUS_TRACK_BAND_TYPES;
#endif

/** @brief  */
#ifndef HAVE_ENUM_ICAROUS_FMS_STATE
#define HAVE_ENUM_ICAROUS_FMS_STATE
typedef enum ICAROUS_FMS_STATE
{
   ICAROUS_FMS_STATE_IDLE=0, /*  | */
   ICAROUS_FMS_STATE_TAKEOFF=1, /*  | */
   ICAROUS_FMS_STATE_CLIMB=2, /*  | */
   ICAROUS_FMS_STATE_CRUISE=3, /*  | */
   ICAROUS_FMS_STATE_APPROACH=4, /*  | */
   ICAROUS_FMS_STATE_LAND=5, /*  | */
   ICAROUS_FMS_STATE_ENUM_END=6, /*  | */
} ICAROUS_FMS_STATE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_icarous_heartbeat.h"
#include "./mavlink_msg_icarous_kinematic_bands.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 3

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_ICAROUS_HEARTBEAT, MAVLINK_MESSAGE_INFO_ICAROUS_KINEMATIC_BANDS}
# define MAVLINK_MESSAGE_NAMES {{ "ICAROUS_HEARTBEAT", 42000 }, { "ICAROUS_KINEMATIC_BANDS", 42001 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_ICAROUS_H
