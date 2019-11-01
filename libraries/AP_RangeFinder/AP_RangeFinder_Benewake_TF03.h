#pragma once

#include "AP_RangeFinder_Benewake.h"

class AP_RangeFinder_Benewake_TF03 : public AP_RangeFinder_Benewake
{
public:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;

protected:
    float model_dist_max_cm() const override { return 18000; }
};
