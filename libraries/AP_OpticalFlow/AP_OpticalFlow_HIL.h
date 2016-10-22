#pragma once

#include "OpticalFlow.h"

class AP_OpticalFlow_HIL : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_HIL(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);
};
