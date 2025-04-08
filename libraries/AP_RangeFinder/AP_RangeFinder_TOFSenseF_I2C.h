#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

#define TOFSENSEP_I2C_DEFAULT_ADDR   0x08

class AP_RangeFinder_TOFSenseF_I2C : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // constructor
    AP_RangeFinder_TOFSenseF_I2C(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init(void);
    void timer(void);

    uint32_t distance_mm;
    bool new_distance; // true if we have a new distance

    // get a reading
    bool start_reading(void);
    bool get_reading(uint32_t &reading_mm, uint16_t &signal_strength, uint16_t &status);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

#endif  // AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
