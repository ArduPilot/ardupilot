#pragma once

#include "OpticalFlow.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>

class AP_OpticalFlow_SITL : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_SITL(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);

private:
    SITL::SITL *_sitl;
    uint32_t last_flow_ms;

    uint8_t next_optflow_index;
    uint8_t optflow_delay;
    OpticalFlow::OpticalFlow_state optflow_data[20];
};
#endif // CONFIG_HAL_BOARD
