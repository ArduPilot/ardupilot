#pragma once

#include "AP_OpticalFlow.h"

#ifndef AP_OPTICALFLOW_CXOF_ENABLED
#define AP_OPTICALFLOW_CXOF_ENABLED AP_OPTICALFLOW_ENABLED
#endif

#if AP_OPTICALFLOW_CXOF_ENABLED

#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_CXOF : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_CXOF(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *uart);

    // initialise the sensor
    void init() override;

    // read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_CXOF *detect(AP_OpticalFlow &_frontend);

private:

    AP_HAL::UARTDriver *uart;           // uart connected to flow sensor
    uint64_t last_frame_us;             // system time of last message from flow sensor
    uint8_t buf[10];                    // buff of characters received from flow sensor
    uint8_t buf_len;                    // number of characters in buffer
    Vector2f gyro_sum;                  // sum of gyro sensor values since last frame from flow sensor
    uint16_t gyro_sum_count;            // number of gyro sensor values in sum
};

#endif  // AP_OPTICALFLOW_CXOF_ENABLED
