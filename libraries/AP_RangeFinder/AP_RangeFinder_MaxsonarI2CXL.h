#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MAXSONARI2CXL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

#define AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR   0x70
#define AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING 0x51

class AP_RangeFinder_MaxsonarI2CXL : public AP_RangeFinder_Backend_I2C
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_MaxsonarI2CXL(_state, _params, dev));
    }

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool init(void) override;
    void _timer(void);

    uint16_t distance;
    bool new_distance;
    
    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
};

#endif  // AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
