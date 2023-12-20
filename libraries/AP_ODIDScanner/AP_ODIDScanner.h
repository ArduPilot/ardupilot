
#pragma once

#define ODID_SCANNER_ENABLED 1
#ifdef ODID_SCANNER_ENABLED

#include "AP_Common/AP_Common.h"
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>

class AP_ODIDScanner
{
public:
    AP_ODIDScanner();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ODIDScanner);
    void init();
    void update();
    mavlink_channel_t _chan; // mavlink channel that communicates with the remote id transceiver
    AP_Int8  _mav_port;
    mavlink_uav_found_t found_msg;
    AP_HaL::UARTDriver* _port;
    bool _initialised;
    uint32_t last_send_ms;
    uint32_t last_dev_hb_ms;
    uint32_t last_dev_hb_msg_ms;

    mavlink_message_t _channel_buffer;
    mavlink_status_t _channel_status;

    mavlink_message_t *channel_buffer() { return &_channel_buffer; }
    mavlink_status_t *channel_status() { return &_channel_status; }
};

#endif
