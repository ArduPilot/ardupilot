

#include <AP_HAL.h>

#include "downstream.h"
#include "state.h"

extern const AP_HAL::HAL& hal;
extern mavlink_channel_t downstream_channel;

extern FMStateMachine sm;

static void downstream_handle_heartbeat(mavlink_message_t* msg) __attribute__((noinline));
static void downstream_handle_heartbeat(mavlink_message_t* msg) {
    mavlink_heartbeat_t pkt;
    mavlink_msg_heartbeat_decode(msg, &pkt);
    sm.on_downstream_heartbeat(&pkt);
}

static void downstream_handle_gps(mavlink_message_t* msg) __attribute__((noinline));
static void downstream_handle_gps(mavlink_message_t* msg) {
    mavlink_gps_raw_int_t pkt;
    mavlink_msg_gps_raw_int_decode(msg, &pkt);
    sm.on_downstream_gps_raw_int(&pkt);
}

void downstream_handler(mavlink_channel_t from, mavlink_message_t* msg) {
    switch (msg->msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        downstream_handle_heartbeat(msg); 
        _mavlink_resend_uart(downstream_channel, msg);
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        downstream_handle_gps(msg);
        _mavlink_resend_uart(downstream_channel, msg);
        break;
      default:
        _mavlink_resend_uart(downstream_channel, msg);
    }
}

