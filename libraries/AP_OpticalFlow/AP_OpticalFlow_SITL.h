#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_SITL_ENABLED

#include "AP_OpticalFlow.h"

class AP_OpticalFlow_SITL : public OpticalFlow_backend
{
public:
    /// constructor
    using OpticalFlow_backend::OpticalFlow_backend;

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

private:

    uint32_t last_flow_ms;

    uint8_t next_optflow_index;
    uint8_t optflow_delay;
    AP_OpticalFlow::OpticalFlow_state optflow_data[20];
};

#endif  // AP_OPTICALFLOW_SITL_ENABLED
