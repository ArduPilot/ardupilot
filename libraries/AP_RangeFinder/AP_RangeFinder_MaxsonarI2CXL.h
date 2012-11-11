// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-

#ifndef __AP_RANGEFINDER_MAXSONARI2CXL_H__
#define __AP_RANGEFINDER_MAXSONARI2CXL_H__

#include "RangeFinder.h"
#include <I2C.h>                        // Arduino I2C lib

#define AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR   0x70

#define AP_RANGEFINDER_MAXSONARI2CXL                4
#define AP_RANGE_FINDER_MAXSONARI2CXL_SCALER        1.0
#define AP_RANGE_FINDER_MAXSONARI2CXL_MIN_DISTANCE  20
#define AP_RANGE_FINDER_MAXSONARI2CXL_MAX_DISTANCE  765

#define AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING 0x51

class AP_RangeFinder_MaxsonarI2CXL : public RangeFinder
{

public:

    // constructor
    AP_RangeFinder_MaxsonarI2CXL(FilterInt16 *filter);

    // init - simply sets the i2c address
    void init(uint8_t address = AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR) { _addr = address; }

    // take_reading - ask sensor to make a range reading
    bool            take_reading();

    // read value from sensor and return distance in cm
    int             read();

    // heath
    bool            healthy;

protected:
    uint8_t _addr;

};
#endif  // __AP_RANGEFINDER_MAXSONARI2CXL_H__
