/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_OpticalFlow_HIL_H
#define AP_OpticalFlow_HIL_H

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

#endif // AP_OpticalFlow_HIL_H

