
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <GCS_MAVLink.h>
#include <GCS_Console.h>
#include "simplegcs.h"

void send_heartbeat(mavlink_channel_t chan) {
    uint8_t base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED
                      | MAV_MODE_FLAG_GUIDED_ENABLED
                      | MAV_MODE_FLAG_SAFETY_ARMED
                      | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = 5 ; /* Loiter is mode 5. */

    mavlink_msg_heartbeat_send(
            chan,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            custom_mode,
            system_status);
}

#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

bool try_send_message(mavlink_channel_t chan, int msgid) {

    int payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        return true;
    default:
        return false;
    }
}

bool try_send_statustext(mavlink_channel_t chan, const char *text, int len) {

    int payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;
    CHECK_PAYLOAD_SIZE(STATUSTEXT);

    char statustext[50] = { 0 };
    if (len < 50) {
        memcpy(statustext, text, len);
    }
    mavlink_msg_statustext_send(
            chan,
            1, /* SEVERITY_LOW */
            statustext);
    return true;
}
// -----------------------------------------------------------------------


void simplegcs_update(mavlink_channel_t chan) {
    mavlink_message_t msg;
    mavlink_status_t status;
    while(comm_get_available(chan)){
        uint8_t c = comm_receive_ch(chan);
        bool newmsg = mavlink_parse_char(chan, c, &msg, &status);
        if (newmsg) {
            handle_message(chan, &msg);
        }
    }
}


void handle_message(mavlink_channel_t chan, mavlink_message_t* msg) {
    hal.console->printf_P(PSTR("SimpleGCS Handle Message %d\r\n"), msg->msgid);
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            char param_name[16] = "NOPARAMS";

            /* Send a single parameter.*/
            mavlink_msg_param_value_send(
                    chan,
                    param_name,
                    0.0,                /* param value */
                    MAVLINK_TYPE_FLOAT, /* param type */
                    1,                  /* _queued_parameter_count */
                    0);                 /* _queued_parameter_index */
            break;
        }
        case MAVLINK_MSG_ID_DATA16:
            gcs_console_handle_data16(msg); 
            break;
        case MAVLINK_MSG_ID_DATA32:
            gcs_console_handle_data32(msg); 
            break;
    }
}

