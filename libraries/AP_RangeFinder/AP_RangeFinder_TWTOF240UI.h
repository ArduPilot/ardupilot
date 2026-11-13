#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TWTOF240UI_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

// 7-bit address. RNGFNDx_ADDR overrides this when non-zero.
#define AP_RANGEFINDER_TWTOF240UI_DEFAULT_ADDR 0x36

class AP_RangeFinder_TWTOF240UI : public AP_RangeFinder_Backend_I2C
{
public:
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::I2CDevice &dev) {
        return configure(NEW_NOTHROW AP_RangeFinder_TWTOF240UI(_state, _params, dev));
    }

    void update(void) override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool init(void) override;
    void timer(void);

    // reading_mm is nullptr when only the bus response is required (init).
    bool read_sample(uint16_t *reading_mm);

    uint16_t distance_mm = 0;
    bool new_distance = false;
};

#endif  // AP_RANGEFINDER_TWTOF240UI_ENABLED
