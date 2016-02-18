/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * Portions of this driver were borrowed from the PX4Firmware px4flow driver which can be found here:
 *     https://github.com/PX4/Firmware/blob/master/src/drivers/px4flow/px4flow.cpp
 */
#pragma once

#include "OpticalFlow.h"

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

class AP_OpticalFlow_Linux : public OpticalFlow_backend
{
public:
    // constructor
    AP_OpticalFlow_Linux(OpticalFlow &_frontend);

    // initialise the sensor
    void init();

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

private:

    typedef struct PACKED {
        uint16_t frame_count;
        int16_t pixel_flow_x_sum;
        int16_t pixel_flow_y_sum;
        int16_t flow_comp_m_x;
        int16_t flow_comp_m_y;
        int16_t qual;
        int16_t gyro_x_rate;
        int16_t gyro_y_rate;
        int16_t gyro_z_rate;
        uint8_t gyro_range;
        uint8_t sonar_timestamp;
        int16_t ground_distance;
    } i2c_frame;

    typedef struct PACKED {
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
    } i2c_integral_frame;

    typedef struct {
        uint64_t timestamp;                     // in microseconds since system start
        uint8_t sensor_id;                      // id of the sensor emitting the flow value
        float pixel_flow_x_integral;            // accumulated optical flow in radians around x axis
        float pixel_flow_y_integral;            // accumulated optical flow in radians around y axis
        float gyro_x_rate_integral;             // accumulated gyro value in radians around x axis
        float gyro_y_rate_integral;             // accumulated gyro value in radians around y axis
        float gyro_z_rate_integral;             // accumulated gyro value in radians around z axis
        float ground_distance_m;                // Altitude / distance to ground in meters
        uint32_t integration_timespan;          // accumulation time span in microseconds
        uint32_t time_since_last_sonar_update;  // time since last sonar update in microseconds
        uint16_t frame_count_since_last_readout;//number of accumulated frames in time span
        int16_t gyro_temperature;               // Temperature * 100 in centi-degrees celsius
        uint8_t quality;                        // Average of quality of accumulated frames, 0: bad quality, 255: maximum quality
    } optical_flow_s;

    // request the sensor produce a measurement, returns true on success
    bool request_measurement();

    // read from sensor, returns true if successful
    bool read(optical_flow_s* report);

    bool initialised = false;
    uint16_t num_errors = 0;
    uint32_t last_read_ms = 0;
};
