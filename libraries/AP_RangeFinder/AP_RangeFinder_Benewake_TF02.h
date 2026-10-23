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

    AP_RangeFinder_Benewake_TF02(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Benewake(_state, _params)
    {
        // TF02 datasheet: 0.4m minimum, 10m at 10% reflectivity
        params.min_distance.set_default(0.4f);
        params.max_distance.set_default(10.0f);
    }

};

#endif  // AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
