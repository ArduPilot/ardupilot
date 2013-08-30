// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_LR4_H__
#define __AP_RANGEFINDER_LR4_H__

#include "RangeFinder.h"

#define AP_RANGE_FINDER_LR4_MIN_DISTANCE  0
#define AP_RANGE_FINDER_LR4_MAX_DISTANCE  5000

class AP_RangeFinder_LR4 : public RangeFinder {
    public:
        AP_RangeFinder_LR4(AP_HAL::UARTDriver *uart, FilterInt16 *filter);
        bool start_reading();
        bool stop_reading();
        int read();
        
    protected:
        AP_HAL::UARTDriver* _port;
        bool reading;
};

#endif  // __AP_RANGEFINDER_LR4_H__
