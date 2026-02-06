// Datasheet: https://en.benewake.com/DataDownload/index_pid_20_lcid_104.html

#pragma once

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
    void init_serial(uint8_t serial_instance) override;
    ~AP_RangeFinder_Benewake_TFA1500() override;

protected:
    float model_dist_max_cm() const override { return 0x3FFFFF; }

private:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
    bool get_reading(float &reading_m) override;
    uint8_t tf_linebuf[5];
    uint8_t tf_linebuf_len;
    uint32_t last_init_ms = 0;
};

#endif // AP_RANGEFINDER_BENEWAKE_TFA1500
