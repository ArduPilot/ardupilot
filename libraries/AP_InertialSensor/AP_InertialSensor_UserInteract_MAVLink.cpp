// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdarg.h>
#include <AP_HAL.h>
#include <GCS_MAVLink.h>
#include "AP_InertialSensor_UserInteract_MAVLink.h"

extern const AP_HAL::HAL& hal;

uint8_t AP_InertialSensor_UserInteract_MAVLink::blocking_read(void) 
{
    uint32_t start_ms = hal.scheduler->millis();
    /* Wait for a COMMAND_ACK message to be received from the ground station */
    while (hal.scheduler->millis() - start_ms < 30000U) {
        while (!comm_get_available(_chan)) {
            hal.scheduler->delay(1);
        }
        uint8_t c = comm_receive_ch(_chan);
        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(_chan, c, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                return 0;
            }
        }
    }
    hal.console->println_P(PSTR("Timed out waiting for user response"));
    return 0;
}

void AP_InertialSensor_UserInteract_MAVLink::_printf_P(const prog_char* fmt, ...) 
{
    char msg[50];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf_P(msg, sizeof(msg), (const prog_char_t *)fmt, ap);
    va_end(ap);
    if (msg[strlen(msg)-1] == '\n') {
        // STATUSTEXT messages should not add linefeed
        msg[strlen(msg)-1] = 0;
    }
    while (comm_get_txspace(_chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES < (int)sizeof(mavlink_statustext_t)) {
        hal.scheduler->delay(1);        
    }
    mavlink_msg_statustext_send(_chan, SEVERITY_USER_RESPONSE, msg);
}

