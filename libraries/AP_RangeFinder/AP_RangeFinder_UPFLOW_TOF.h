#pragma once

#include "AP_RangeFinder_config.h"
#include "../AP_OpticalFlow/AP_OpticalFlow_UPFLOW_Tx.h"

#if AP_RANGEFINDER_UPFLOW_Tx_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include "AP_RangeFinder_Params.h"
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

    uint8_t glitch_count{0};          // glitch counter

};

#endif  // AP_RANGEFINDER_UPFLOW_Tx_ENABLED
