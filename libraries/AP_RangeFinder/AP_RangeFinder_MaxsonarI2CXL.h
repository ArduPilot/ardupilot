#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR   0x70

#define AP_RANGEFINDER_MAXSONARI2CXL                4
#define AP_RANGE_FINDER_MAXSONARI2CXL_SCALER        1.0
#define AP_RANGE_FINDER_MAXSONARI2CXL_MIN_DISTANCE  20
#define AP_RANGE_FINDER_MAXSONARI2CXL_MAX_DISTANCE  765

#define AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING 0x51

class AP_RangeFinder_MaxsonarI2CXL : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);

private:
    // constructor
    AP_RangeFinder_MaxsonarI2CXL(RangeFinder &ranger, uint8_t instance,
                                 RangeFinder::RangeFinder_State &_state);

    bool _init(void);
    void _timer(void);

    uint16_t distance;
    bool new_distance;
    
    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
