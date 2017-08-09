#pragma once

#include "RangeFinder.h"
#include "AP_RangeFinder_I2C.h"

class AP_RangeFinder_LightWareI2C : public AP_RangeFinder_I2C
{
    using AP_RangeFinder_I2C::AP_RangeFinder_I2C;

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void);

protected:

    uint8_t addr() const override;
    bool probe() override;

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    void init();
    void timer();

    // get a reading
    bool get_reading(uint16_t &reading_cm);
};
