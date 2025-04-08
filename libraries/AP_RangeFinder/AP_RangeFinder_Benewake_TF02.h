#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TF02_ENABLED

#include "AP_RangeFinder_Benewake.h"

class AP_RangeFinder_Benewake_TF02 : public AP_RangeFinder_Benewake
{
public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Benewake_TF02(_state, _params);
    }

protected:
    float model_dist_max_cm() const override { return 2200; }
    bool has_signal_byte() const override { return true; }

private:

    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;

};

#endif  // AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
