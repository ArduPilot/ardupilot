#pragma once

#include "AP_RangeFinder_Benewake.h"

#ifndef AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

#if AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED

#define TFMINI_ADDR_DEFAULT              0x10        // TFMini default device id

class AP_RangeFinder_Benewake_TFMini : public AP_RangeFinder_Benewake
{
public:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;

protected:
    float model_dist_max_cm() const override { return 1200; }
};

#endif  // AP_RANGEFINDER_BENEWAKE_TFMINI
