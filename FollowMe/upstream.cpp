
#include <AP_HAL.h>

#include "upstream.h"
#include "state.h"

extern const AP_HAL::HAL& hal;
extern mavlink_channel_t upstream_channel;
extern FMStateMachine sm;

// For some commands going upstream, switch off follow-me.
static void upstream_handle_command_long(mavlink_message_t* msg) __attribute__((noinline));
static void upstream_handle_command_long(mavlink_message_t* msg) {
    mavlink_command_long_t pkt;
    mavlink_msg_command_long_decode(msg, &pkt);
    sm.on_upstream_command_long(&pkt);
}

// For change-mode commands going upstream, switch off follow-me.
static void upstream_handle_set_mode(mavlink_message_t* msg) __attribute__((noinline));
static void upstream_handle_set_mode(mavlink_message_t* msg) {
    mavlink_set_mode_t pkt;
    mavlink_msg_set_mode_decode(msg, &pkt);
    sm.on_upstream_set_mode(&pkt);
}

// Handles messages TO the upstream.
void upstream_handler(mavlink_channel_t from, mavlink_message_t* msg) {
    switch (msg->msgid) {
      case MAVLINK_MSG_ID_COMMAND_LONG:
    	// For some commands going upstream, switch off follow-me.
        upstream_handle_command_long(msg);
        _mavlink_resend_uart(upstream_channel, msg);
        break;
      case MAVLINK_MSG_ID_SET_MODE:
      	// For change-mode commands going upstream, switch off follow-me.
        upstream_handle_set_mode(msg);
        _mavlink_resend_uart(upstream_channel, msg);
        break;
      default:
        _mavlink_resend_uart(upstream_channel, msg);
    }
}
