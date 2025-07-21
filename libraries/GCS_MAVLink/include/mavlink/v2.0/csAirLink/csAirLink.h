/** @file
 *  @brief MAVLink comm protocol generated from csAirLink.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_CSAIRLINK_H
#define MAVLINK_CSAIRLINK_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_CSAIRLINK.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_CSAIRLINK_XML_HASH 1692707236079213978

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{52000, 13, 100, 100, 0, 0, 0}, {52001, 239, 1, 1, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_CSAIRLINK

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_AIRLINK_AUTH_RESPONSE_TYPE
#define HAVE_ENUM_AIRLINK_AUTH_RESPONSE_TYPE
typedef enum AIRLINK_AUTH_RESPONSE_TYPE
{
   AIRLINK_ERROR_LOGIN_OR_PASS=0, /* Login or password error | */
   AIRLINK_AUTH_OK=1, /* Auth successful | */
   AIRLINK_AUTH_RESPONSE_TYPE_ENUM_END=2, /*  | */
} AIRLINK_AUTH_RESPONSE_TYPE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_airlink_auth.h"
#include "./mavlink_msg_airlink_auth_response.h"

// base include



#if MAVLINK_CSAIRLINK_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_AIRLINK_AUTH, MAVLINK_MESSAGE_INFO_AIRLINK_AUTH_RESPONSE}
# define MAVLINK_MESSAGE_NAMES {{ "AIRLINK_AUTH", 52000 }, { "AIRLINK_AUTH_RESPONSE", 52001 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_CSAIRLINK_H
