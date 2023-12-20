#include "AP_ODIDScanner.h"

AP_ODIDScanner::AP_ODIDScanner() {

}
void AP_ODIDScanner::init() {

    _chan = mavlink_channel_t(gcs().get_channel_from_port_number(_mav_port));
    _initialised = true;
}
void AP_ODIDScanner::update() {
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms > 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "The driver is here.");
        last_send_ms =now_ms;
    }
}
