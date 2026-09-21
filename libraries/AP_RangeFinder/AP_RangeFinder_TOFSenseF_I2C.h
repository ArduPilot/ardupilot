#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

#define TOFSENSEP_I2C_DEFAULT_ADDR   0x08

class AP_RangeFinder_TOFSenseF_I2C : public AP_RangeFinder_Backend_I2C
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_TOFSenseF_I2C(_state, _params, dev));
    }

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // constructor
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool init(void) override;
    void timer(void);

    uint32_t distance_mm;
    bool new_distance; // true if we have a new distance

    // get a reading
    bool start_reading(void);
    bool get_reading(uint32_t &reading_mm, uint16_t &signal_strength, uint16_t &status);
};

#endif  // AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
