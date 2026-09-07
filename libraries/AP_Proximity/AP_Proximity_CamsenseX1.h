#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_CAMSENSE_X1_ENABLED

#include "AP_Proximity_Backend_Serial.h"

class AP_Proximity_CamsenseX1 : public AP_Proximity_Backend_Serial
{
public:
    
    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    
    void update(void) override;

    
    float distance_max_m() const override { return 8.0f; }
    float distance_min_m() const override { return 0.12f; }



private:

    void get_readings();
    void decode_packet();


    uint8_t _buffer[36];
    uint16_t _byte_count;
    uint32_t _last_distance_received_ms;


    AP_Proximity_Boundary_3D::Face _last_face;
    float _last_angle_deg;
    float _last_distance_m;
    bool _last_distance_valid;


    enum class State {
        WAIT_SYNC1 = 0,
        WAIT_SYNC2,
        RECEIVING
    } _step = State::WAIT_SYNC1;

    static constexpr uint8_t MSG_HEADER1 = 0x55;
    static constexpr uint8_t MSG_HEADER2 = 0xAA;
    static constexpr uint8_t MSG_SAMPLES = 8;
};

#endif // AP_PROXIMITY_CAMSENSE_X1_ENABLED
