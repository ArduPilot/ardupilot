#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>

/* Ensure compatibility with GCS_MAVLink library. We need the DATA16
 * and DATA32 mesages. If these aren't present, get them from the mavlink
 * repo message_definitions/ardupilotmega.xml and regenerate the GCS_MAVLink
 * definitions. */
#ifndef MAVLINK_MSG_ID_DATA16
#error GCS_Console module requires Mavlink Message DATA16
#endif
#ifndef MAVLINK_MSG_ID_DATA32
#error GCS_Console module requires Mavlink Message DATA32
#endif

#define DATAMSG_TYPE_CONSOLE 0xFE

void gcs_console_handle_data16(mavlink_message_t* msg);
void gcs_console_handle_data32(mavlink_message_t* msg);

void gcs_console_send(mavlink_channel_t chan);
