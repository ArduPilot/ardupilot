#pragma once

#include "AP_OpticalFlow.h"

#ifndef AP_OPTICALFLOW_PX4FLOW_ENABLED
#define AP_OPTICALFLOW_PX4FLOW_ENABLED AP_OPTICALFLOW_ENABLED
#endif

#if AP_OPTICALFLOW_PX4FLOW_ENABLED

#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_PX4Flow : public OpticalFlow_backend
{
public:
    /// constructor
    using OpticalFlow_backend::OpticalFlow_backend;

    CLASS_NO_COPY(AP_OpticalFlow_PX4Flow);

    // init - initialise the sensor
    void init() override {}

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_PX4Flow *detect(AP_OpticalFlow &_frontend);

private:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    static const uint8_t REG_INTEGRAL_FRAME = 0x16;
    
    // I2C data on register REG_INTEGRAL_FRAME
    struct PACKED i2c_integral_frame {
        uint16_t frame_count_since_last_readout;
        int16_t pixel_flow_x_integral;
        int16_t pixel_flow_y_integral;
        int16_t gyro_x_rate_integral;
        int16_t gyro_y_rate_integral;
        int16_t gyro_z_rate_integral;
        uint32_t integration_timespan;
        uint32_t sonar_timestamp;
        uint16_t ground_distance;
        int16_t gyro_temperature;
        uint8_t qual;
    };
    
    // scan I2C bus addresses and buses
    bool scan_buses(void);

    // setup sensor
    bool setup_sensor(void);

    void timer(void);
};

#endif  // AP_OPTICALFLOW_PX4FLOW_ENABLED
