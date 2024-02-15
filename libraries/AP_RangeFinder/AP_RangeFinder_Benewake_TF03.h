#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED

#include "AP_RangeFinder_Benewake.h"

class AP_RangeFinder_Benewake_TF03 : public AP_RangeFinder_Benewake
{
public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_Benewake_TF03(_state, _params);
    }

protected:
    float model_dist_max_cm() const override { return 18000; }

private:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
};

#endif  // AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
