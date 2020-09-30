#pragma once

#include "AP_Frsky_Backend.h"

class AP_Frsky_SPort : public AP_Frsky_Backend
{

public:

    using AP_Frsky_Backend::AP_Frsky_Backend;

    void send() override;

protected:

    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    struct PACKED {
        bool send_latitude; // sizeof(bool) = 4 ?
        uint32_t gps_lng_sample;
        uint8_t new_byte;
    } _passthrough;

    uint32_t calc_gps_latlng(bool *send_latitude);

private:

    struct {
        bool sport_status;
        bool gps_refresh;
        bool vario_refresh;
        uint8_t fas_call;
        uint8_t gps_call;
        uint8_t vario_call;
        uint8_t various_call;
    } _SPort;
};
