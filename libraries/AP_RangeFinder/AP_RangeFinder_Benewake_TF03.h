#pragma once

#include "AP_RangeFinder_Benewake.h"

#ifndef AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TF03_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#if AP_RANGEFINDER_BENEWAKE_TF03_ENABLED

class AP_RangeFinder_Benewake_TF03 : public AP_RangeFinder_Benewake
{
public:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;

protected:
    float model_dist_max_cm() const override { return 18000; }
};

#endif  // AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
