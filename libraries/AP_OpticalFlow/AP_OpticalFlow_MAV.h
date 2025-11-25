#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_MAV_ENABLED

#include "AP_OpticalFlow_Backend.h"


class AP_OpticalFlow_MAV : public OpticalFlow_backend
{
public:

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
    uint64_t prev_frame_us;
    uint64_t latest_frame_us;
    Vector2f flow_sum;
    bool flow_sum_is_rads;
    uint16_t quality_sum;
    uint16_t count;
    uint8_t sensor_id;
    Vector2f gyro_sum;
    uint16_t gyro_sum_count;
};

#endif  // AP_OPTICALFLOW_MAV_ENABLED