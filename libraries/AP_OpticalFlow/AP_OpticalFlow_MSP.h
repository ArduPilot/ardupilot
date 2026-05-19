#pragma once

#include "AP_OpticalFlow_config.h"

#if HAL_MSP_OPTICALFLOW_ENABLED

#include "AP_OpticalFlow.h"

class AP_OpticalFlow_MSP : public OpticalFlow_backend
{
public:
    /// constructor
    using OpticalFlow_backend::OpticalFlow_backend;

    // initialise the sensor
    void init() override {}

    // read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // get update from msp
    void handle_msp(const MSP::msp_opflow_data_message_t &pkt) override;

    // detect if the sensor is available
    static AP_OpticalFlow_MSP *detect(AP_OpticalFlow &_frontend);

private:

    uint64_t prev_frame_us;             // system time of last message when update was last called
    uint64_t latest_frame_us;           // system time of most recent messages processed
    Vector2l flow_sum;                  // sum of sensor's flow_x and flow_y values since last call to update
    uint16_t quality_sum;               // sum of sensor's quality values since last call to update
    uint16_t count;                     // number of sensor readings since last call to update
    Vector2f gyro_sum;                  // sum of gyro sensor values since last frame from flow sensor
    uint16_t gyro_sum_count;            // number of gyro sensor values in sum
};

#endif // HAL_MSP_OPTICALFLOW_ENABLED
