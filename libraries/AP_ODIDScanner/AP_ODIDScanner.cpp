#include "AP_ODIDScanner.h"
#include "GCS_MAVLink/GCS_MAVLink.h"
#include "GCS_MAVLink/GCS.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>

// TODO: Random default for mav_port needs fix
AP_ODIDScanner::AP_ODIDScanner() : _mav_port(1){

}
void AP_ODIDScanner::init() {
   _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _initialised = true;
    _port = AP::serialmanager().get_serial_by_id(_mav_port);
    if (_port != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Found RID Device");
        _port->begin(57600, 512, 512);
    }
}

void AP_ODIDScanner::update_recv() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t now_ms = AP_HAL::millis();

    status.packet_rx_drop_count = 0;
    if(_port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"_port is null");
        return;
    } 

    const uint16_t nbytes = _port->available();
    if (nbytes > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"available Bytes: %d", nbytes);
    }
    for (uint16_t i=0; i<nbytes; i++)
    {
        const uint8_t c = (uint8_t)_port->read();
        if (mavlink_frame_char_buffer(channel_buffer(), channel_status(), c, &msg, &status) == MAVLINK_FRAMING_OK) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Found message");
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: 
                {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Recv'd heartbeat");
                    mavlink_heartbeat_t packet;
                    mavlink_msg_heartbeat_decode(&msg, &packet);
                    last_dev_hb_ms = now_ms;
                }
            }
        }
    }
}

void AP_ODIDScanner::handle_msg(mavlink_message_t msg) {
    static int i = 0;
    switch(msg.msgid) {
        case MAVLINK_MSG_ID_UAV_FOUND: 
        {
            // mavlink_msg_uav_found_t uav;
            // mavlink_msg_uav_found_decode(&msg, &uav);
            i+=1;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ODIDScanner: uav's here %d", i);
            break;
        }
    }
}
void AP_ODIDScanner::update() {
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_dev_hb_ms > 5000 && now_ms - last_dev_hb_msg_ms > 5000) {
        last_dev_hb_msg_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Device Not Found");
        _port->printf("Scanner: Where is this printing?");
    }
    if (now_ms - last_hb_send_ms > 1000) {
        _port->printf("Scanner: HB Where is this printing?");

        last_hb_send_ms = now_ms;
    mavlink_msg_heartbeat_send(
        _chan,
        gcs().frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        gcs().custom_mode(),
        0);
        } 
}

