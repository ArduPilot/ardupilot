#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_MAV_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_MAV : public OpticalFlow_backend
{
public:
    /// constructor
    using OpticalFlow_backend::OpticalFlow_backend;

    // initialise the sensor
    void init() override {}

    // read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // get update from mavlink
    void handle_msg(const mavlink_message_t &msg) override;

    // detect if the sensor is available
    static AP_OpticalFlow_MAV *detect(AP_OpticalFlow &_frontend);

private:

    uint64_t prev_frame_us;             // system time of last message when update was last called
    uint64_t latest_frame_us;           // system time of most recent messages processed
    Vector2l flow_sum;                  // sum of sensor's flow_x and flow_y values since last call to update
    uint16_t quality_sum;               // sum of sensor's quality values since last call to update
    uint16_t count;                     // number of sensor readings since last call to update
    uint8_t sensor_id;                  // sensor_id received in latest mavlink message
    Vector2f gyro_sum;                  // sum of gyro sensor values since last frame from flow sensor
    uint16_t gyro_sum_count;            // number of gyro sensor values in sum
};

#endif  // AP_OPTICALFLOW_MAV_ENABLED
