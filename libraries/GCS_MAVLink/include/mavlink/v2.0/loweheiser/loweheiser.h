/** @file
 *  @brief MAVLink comm protocol generated from loweheiser.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_LOWEHEISER_H
#define MAVLINK_LOWEHEISER_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_LOWEHEISER.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_LOWEHEISER_XML_HASH 7216325201046501494

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {10151, 195, 85, 85, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_LOWEHEISER

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_loweheiser_gov_efi.h"

// base include
#include "../minimal/minimal.h"


#if MAVLINK_LOWEHEISER_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_LOWEHEISER_GOV_EFI}
# define MAVLINK_MESSAGE_NAMES {{ "HEARTBEAT", 0 }, { "LOWEHEISER_GOV_EFI", 10151 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_LOWEHEISER_H
