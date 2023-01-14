#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_ESC_TELEM_HOBBYWING_ENABLED

#include "AP_HobbyWing_ESC.h"
#include "AP_ESC_Telem_Backend.h"
#include <AP_Param/AP_Param.h>

class AP_ESC_Telem_HobbyWing
    : AP_ESC_Telem_Backend
{
public:

    // constructor
    AP_ESC_Telem_HobbyWing();

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    // entry point for SRV_Channel to prod us to supply data to the
    // AP_ESC_Telem library:
    void update_telemetry();

private:

    AP_Int32 channel_mask_v3;
    AP_Int8 motor_poles_v3;

    AP_Int32 channel_mask_v4;
    AP_Int8 motor_poles_v4;

    AP_Int32 channel_mask_x4;
    AP_Int8 motor_poles_x4;

    static const uint8_t MAX_ESCS { 8 };
    AP_HobbyWing_ESC *escs[MAX_ESCS];
    uint8_t servo_channel[MAX_ESCS];

    uint8_t num_escs;

    void thread_main(void);
};
#endif  // AP_ESC_TELEM_HOBBYWING_ENABLED
