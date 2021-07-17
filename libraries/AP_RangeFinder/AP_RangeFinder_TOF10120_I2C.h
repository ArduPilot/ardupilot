#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_TOF10120_I2C_DEFAULT_ADDR 0x52
#define AP_RANGE_FINDER_TOF10120_I2C_RANGE_READING_CMD 0x00

class AP_RangeFinder_TOF10120_I2C : public AP_RangeFinder_Backend
{

public:

    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // constructor
    AP_RangeFinder_TOF10120_I2C(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init(void);
    void timer(void);

    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);

    uint16_t distance;
    bool new_distance;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};