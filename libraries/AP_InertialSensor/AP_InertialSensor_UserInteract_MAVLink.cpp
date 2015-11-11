// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdarg.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_InertialSensor_UserInteract_MAVLink.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// set by _snoop on COMMAND_ACK
static bool _got_ack;

/*
  watch for COMMAND_ACK messages
 */
static void _snoop(const mavlink_message_t* msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        _got_ack = true;
    }
}

bool AP_InertialSensor_UserInteract_MAVLink::blocking_read(void) 
{
    uint32_t start_ms = AP_HAL::millis();
    // setup snooping of packets so we can see the COMMAND_ACK
    _gcs->set_snoop(_snoop);
    _got_ack = false;
    while (AP_HAL::millis() - start_ms < 30000U) {
        hal.scheduler->delay(10);
        if (_got_ack) {
            _gcs->set_snoop(NULL);
            return true;    
        }
    }
    hal.console->println("Timed out waiting for user response");
    _gcs->set_snoop(NULL);
    return false;
}

void AP_InertialSensor_UserInteract_MAVLink::printf(const char* fmt, ...)
{
    char msg[50];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (msg[strlen(msg)-1] == '\n') {
        // STATUSTEXT messages should not add linefeed
        msg[strlen(msg)-1] = 0;
    }
    AP_HAL::UARTDriver *uart = _gcs->get_uart();
    /*
      to ensure these messages get to the user we need to wait for the
      port send buffer to have enough room
     */
    while (uart->txspace() < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_STATUSTEXT_LEN) {
        hal.scheduler->delay(1);
    }
    _gcs->send_text(MAV_SEVERITY_CRITICAL, msg);
}
