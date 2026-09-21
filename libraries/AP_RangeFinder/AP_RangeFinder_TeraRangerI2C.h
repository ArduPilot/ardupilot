#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TRI2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

class AP_RangeFinder_TeraRangerI2C : public AP_RangeFinder_Backend_I2C
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &i2c_dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_TeraRangerI2C(_state, _params, i2c_dev));
    }

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // constructor
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool measure(void);
    bool collect_raw(uint16_t &raw_distance);
    bool process_raw_measure(uint16_t raw_distance, uint16_t &distance_cm);

    bool init(void) override;
    void timer(void);

    struct {
        uint32_t sum;
        uint32_t count;
    } accum;
};

#endif  // AP_RANGEFINDER_TRI2C_ENABLED
