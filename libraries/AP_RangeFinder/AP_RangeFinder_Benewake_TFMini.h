#pragma once

#include "AP_RangeFinder_Benewake.h"

#ifndef AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED

class AP_RangeFinder_Benewake_TFMini : public AP_RangeFinder_Benewake
{
public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_Benewake_TFMini(_state, _params);
    }

protected:
    float model_dist_max_cm() const override { return 1200; }

private:

    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
};

#endif  // AP_RANGEFINDER_BENEWAKE_TFMINI
