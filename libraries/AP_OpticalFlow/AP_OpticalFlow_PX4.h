/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_OpticalFlow_PX4_H
#define AP_OpticalFlow_PX4_H

#include "OpticalFlow.h"

class AP_OpticalFlow_PX4 : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_PX4(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);

private:
    int         _fd;                // file descriptor for sensor
    uint64_t    _last_timestamp;    // time of last update (used to avoid processing old reports)
};

#endif
