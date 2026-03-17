#pragma once

// Datasheet: https://en.benewake.com/DataDownload/index_pid_20_lcid_104.html

#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include "AP_RangeFinder_Benewake.h"

class AP_RangeFinder_Benewake_TFA1500 : public AP_RangeFinder_Benewake
{
public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_Benewake_TFA1500(_state, _params);
    }

protected:
    float model_dist_max_cm() const override { return 0x3FFFFF; }

private:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
    bool process_byte(uint8_t received_byte, uint32_t &dist_cm);
    bool get_reading(float &reading_m) override;

    union
    {
        uint8_t bytes[5];
        struct PACKED
        {
            uint8_t header;
            uint8_t dist_low;
            uint8_t dist_mid;
            uint8_t dist_high;
            uint8_t crc_sum_of_bytes;
        } packet;
    } tf_frame;

    uint8_t tf_frame_len;
    uint32_t last_init_ms = 0;
};

#endif // AP_RANGEFINDER_BENEWAKE_TFA1500
