#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_UPFLOW_Tx_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include "AP_RangeFinder_Params.h"
#include "AP_OpticalFlow/AP_OpticalFlow_UPFLOW_Tx.h"
#include "GCS_MAVLink/GCS_MAVLink.h"

class AP_RangeFinder_UPFLOW_TOF : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_UPFLOW_TOF(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    int8_t trigger_pin;
    uint32_t last_reading_ms;      // system time of last read (used for health reporting)
    float last_distance_m;         // last distance reported (used to prevent glitches in measurement)
    uint8_t glitch_count;          // glitch counter

};

#endif  // AP_RANGEFINDER_UPFLOW_Tx_ENABLED
