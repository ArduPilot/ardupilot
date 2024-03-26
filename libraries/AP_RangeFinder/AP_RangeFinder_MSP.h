#pragma once

#include "AP_RangeFinder_config.h"

#if HAL_MSP_RANGEFINDER_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

// Data timeout
#define AP_RANGEFINDER_MSP_TIMEOUT_MS 500

class AP_RangeFinder_MSP : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_MSP(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect();

    // empty update
    void update(void) override {}

    // Get update from msp
    void handle_msp(const MSP::msp_rangefinder_data_message_t &pkt) override;

    uint32_t read_timeout_ms() const override { return AP_RANGEFINDER_MSP_TIMEOUT_MS; }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    // start a reading
    static bool start_reading(void);
    static bool get_reading(uint16_t &reading_cm);
};

#endif  //HAL_MSP_RANGEFINDER_ENABLED

