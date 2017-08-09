#pragma once

#include "RangeFinder.h"
#include "AP_RangeFinder_I2C.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_TeraRangerI2C : public AP_RangeFinder_I2C
{
    using AP_RangeFinder_I2C::AP_RangeFinder_I2C;

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    uint8_t addr() const override { return 0x30; }
    bool probe() override;

private:

    bool measure(void);
    bool collect(uint16_t &distance_cm);

    void timer(void);

    struct {
        uint32_t sum;
        uint32_t count;
    } accum;
};
