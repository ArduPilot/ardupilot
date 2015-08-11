
#include <AP_HAL/AP_HAL.h>
#include "GCS_Console.h"

extern const AP_HAL::HAL& hal;

void gcs_console_handle_data16(mavlink_message_t* msg) {

    mavlink_data16_t data16;
    mavlink_msg_data16_decode(msg, &data16);
    if (data16.type == DATAMSG_TYPE_CONSOLE) {
        hal.console->backend_write(data16.data, data16.len);
    }

}

void gcs_console_handle_data32(mavlink_message_t* msg) {

    mavlink_data32_t data32;
    mavlink_msg_data32_decode(msg, &data32);
    if (data32.type == DATAMSG_TYPE_CONSOLE) {
        hal.console->backend_write(data32.data, data32.len);
    }

}

void gcs_console_send(mavlink_channel_t chan) {
    uint8_t cons[32];
    memset(cons, 0, 32);
    int16_t txspace = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;
    /* There are two bytes of overhead per packet (on top of the non payload
     * bytes). We can read up to 32 bytes from the console if we have 34
     * available for tx, up to 16 bytes if we have more than 18 available.
     * otherwise we have to try again later. */
    int readable = 32;
    if (txspace >= 18 && txspace < 34) {
        readable = 16;
    } else if ( txspace < 18 ) {
        return;
    }
    /* Read from the console backend */
    int avail = hal.console->backend_read(cons, readable);
    /* Send in the smallest packet available. (Don't send if 0.) */
    if (avail > 0 && avail <= 16) {
        mavlink_msg_data16_send(chan, DATAMSG_TYPE_CONSOLE, avail, cons);
    } else if (avail > 16 && avail <= 32) {
        mavlink_msg_data32_send(chan, DATAMSG_TYPE_CONSOLE, avail, cons);
    }
}

