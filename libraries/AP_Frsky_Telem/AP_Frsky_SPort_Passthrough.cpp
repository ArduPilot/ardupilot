#include "AP_Frsky_SPort_Passthrough.h"

bool AP_Frsky_SPort_Passthrough::init()
{
    if (!AP_RCTelemetry::init()) {
        return false;
    }
    return AP_Frsky_SPort::init();
}

bool AP_Frsky_SPort_Passthrough::init_serial_port()
{
    if (_use_external_data) {
        return true;
    }
    return AP_Frsky_SPort::init_serial_port();
}

void  AP_Frsky_SPort_Passthrough::send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    if (_use_external_data) {
        external_data.frame = frame;
        external_data.appid = appid;
        external_data.data = data;
        external_data.pending = true;
        return;
    }

    return AP_Frsky_SPort::send_sport_frame(frame, appid, data);
}
