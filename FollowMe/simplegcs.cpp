
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <GCS_MAVLink.h>
#include <GCS_Console.h>
#include "simplegcs.h"

extern mavlink_channel_t downstream_channel;

static volatile uint8_t lock = 0;

void simplegcs_send_console_async(uint32_t machtnichts) {
  if (lock) return;
  lock = 1;
  gcs_console_send(downstream_channel);
  lock = 0;
}

void simplegcs_send_heartbeat_async(uint32_t us) {
  uint32_t ms = us / 1000;
  static uint32_t last_ms = 0;
  if (ms - last_ms < 1000) return;
  if (lock) return;
  last_ms = ms;
  lock = 1;
  {
    uint8_t base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED
                      | MAV_MODE_FLAG_GUIDED_ENABLED
                      | MAV_MODE_FLAG_SAFETY_ARMED
                      | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = 5 ; /* Loiter is mode 5. */
    mavlink_msg_heartbeat_send(
            downstream_channel,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            custom_mode,
            system_status);
  }
  lock = 0;
}

void simplegcs_send_heartbeat(mavlink_channel_t chan) {
    uint8_t base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED
                      | MAV_MODE_FLAG_GUIDED_ENABLED
                      | MAV_MODE_FLAG_SAFETY_ARMED
                      | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = 5 ; /* Loiter is mode 5. */

    while(lock);
    lock = 1;
    mavlink_msg_heartbeat_send(
            chan,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            custom_mode,
            system_status);
    lock = 0;
}


bool simplegcs_try_send_statustext(mavlink_channel_t chan, const char *text, int len) {

    int payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;
    if (payload_space < MAVLINK_MSG_ID_STATUSTEXT_LEN) return false;

    char statustext[50] = { 0 };
    if (len < 50) {
        memcpy(statustext, text, len);
    }
    while(lock);
    lock = 1;
    mavlink_msg_statustext_send(
            chan,
            1, /* SEVERITY_LOW */
            statustext);
    lock = 0;
    return true;
}
// -----------------------------------------------------------------------


void simplegcs_update(mavlink_channel_t chan, simplegcs_handler_t handler) {
    mavlink_message_t msg;
    mavlink_status_t status;
    while(comm_get_available(chan)){
        uint8_t c = comm_receive_ch(chan);
        bool newmsg = mavlink_parse_char(chan, c, &msg, &status);
        if (newmsg) {
            handler(chan, &msg);
        }
    }
}

