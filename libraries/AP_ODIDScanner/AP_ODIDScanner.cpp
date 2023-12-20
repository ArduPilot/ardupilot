#include "AP_ODIDScanner.h"

// TODO: Random default for mav_port needs fix
AP_ODIDScanner::AP_ODIDScanner() : _mav_port(7){

}
void AP_ODIDScanner::init() {
    _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _initialised = true;
    _port = AP::serialmanager().get_serial_by_id(_mav_port);
}

void AP_ODIDScanner::update_recv() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t tstart_us = AP_HAL::micros();
    uint32_t now_ms = AP_HAL::millis();

    status.packet_rx_drop_count = 0;

    const uint16_t nbytes = _port->available();
    for (uint16_t i=0; i<nbytes; i++)
    {
        const uint8_t c = (uint8_t)_port->read();
        if (mavlink_frame_char_buffer(channel_buffer(), channel_status(), c, &msg, &status) == MAVLINK_FRAMING_OK) {
            // TODO: How do I process this?
        }
    }
}
void AP_ODIDScanner::update() {
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_dev_hb_ms > 5000 && now_ms - last_dev_hb_msg_ms > 5000) {
        last_dev_hb_msg_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scanner: Device Not Found");
    }
}
