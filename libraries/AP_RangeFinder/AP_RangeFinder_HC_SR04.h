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

    void check_pins();
    void check_echo_pin();
    void check_trigger_pin();
    void irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us);

    int8_t echo_pin;
    int8_t trigger_pin;
    uint32_t last_reading_ms;      // system time of last read (used for health reporting)
    uint32_t last_distance_cm;     // last distance reported (used to prevent glitches in measurement)
    uint8_t glitch_count;           // glitch counter

    // follow are modified by the IRQ handler:
    uint32_t pulse_start_us;      // system time of start of timing pulse
    uint32_t irq_value_us;         // last calculated pwm value (irq copy)

    uint32_t last_ping_ms;
};
