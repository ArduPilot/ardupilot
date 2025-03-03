#pragma once

#include "AP_Frsky_Backend.h"

#if AP_FRSKY_D_TELEM_ENABLED

#define FAST_FRAME_INTERVAL     300
#define MID_FRAME_INTERVAL      1500
#define SLOW_FRAME_INTERVAL     3000

class AP_Frsky_D : public AP_Frsky_Backend
{

public:

    using AP_Frsky_Backend::AP_Frsky_Backend;

protected:

    void send() override;
    uint32_t initial_baud() const override
    {
        return 9600;
    }

private:

    // methods related to the nuts-and-bolts of sending data
    void send_byte(uint8_t value);
    void send_uint16(uint16_t id, uint16_t data);

    struct {
        uint32_t last_fast_frame;
        uint32_t last_mid_frame;
        uint32_t last_slow_frame;
    } _D;

};

#endif  // AP_FRSKY_D_TELEM_ENABLED
