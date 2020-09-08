#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include "AP_RangeFinder_Params.h"

class AP_RangeFinder_HC_SR04 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_HC_SR04(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect(AP_RangeFinder_Params &_params);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    bool check_pins();
    bool check_echo_pin();
    void check_trigger_pin();

    int8_t trigger_pin;
    uint32_t last_reading_ms;      // system time of last read (used for health reporting)
    uint32_t last_distance_cm;     // last distance reported (used to prevent glitches in measurement)
    uint8_t glitch_count;           // glitch counter

    int8_t last_warn_echo_pin;
    AP_HAL::PWMSource pwm_source;

    uint32_t last_ping_ms;
};
