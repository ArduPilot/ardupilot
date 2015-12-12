/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_OpticalFlow_Linux.cpp - ardupilot library for the PX4Flow sensor.
 *          inspired by the PX4Firmware code.
 *      
 *       @author: Víctor Mayoral Vilches
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "OpticalFlow.h"

#define PX4FLOW_DEBUG 1

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#define PX4FLOW_I2C_ADDRESS     0x42    // 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49
#define PX4FLOW_REG             0x16    // Measure Register 22
#define I2C_FRAME_SIZE          (sizeof(i2c_frame))
#define I2C_INTEGRAL_FRAME_SIZE (sizeof(i2c_integral_frame))

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_Linux::AP_OpticalFlow_Linux(OpticalFlow &_frontend) : 
    OpticalFlow_backend(_frontend)
{}

void AP_OpticalFlow_Linux::init(void)
{
    // only initialise once
    if (initialised) {
        return;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();
    if (i2c_sem == NULL) {
        return;
    }

    // take i2c bus sempahore
    if (!i2c_sem->take(200)) {
        return;
    }

    // read from flow sensor to ensure it is not a ll40ls Lidar (which can be on 0x42)
    // read I2C_FRAME_SIZE bytes, the ll40ls will error whereas the flow happily returns data
    uint8_t val[I2C_FRAME_SIZE];
    if (hal.i2c->readRegisters(PX4FLOW_I2C_ADDRESS, 0, I2C_FRAME_SIZE, val) != 0) {
        i2c_sem->give();
        return;
    }

    // success
    initialised = true;
    i2c_sem->give();
}

bool AP_OpticalFlow_Linux::request_measurement()
{
    // send measure request to sensor
    uint8_t cmd = PX4FLOW_REG;
    if (hal.i2c->writeRegisters(PX4FLOW_I2C_ADDRESS, cmd, 0, nullptr) != 0) {
        return false;
    }
    return true;
}

bool AP_OpticalFlow_Linux::read(optical_flow_s* report)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();
    if (i2c_sem == NULL) {
        return false;
    }

    // take i2c bus sempahore (non blocking)
    if (!i2c_sem->take_nonblocking()) {
        return false;
    }

    // request measurement
    request_measurement();

    uint8_t val[I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE] = {};
    i2c_integral_frame f_integral;

    // Perform the writing and reading in a single command
    if (PX4FLOW_REG == 0x00) {
        if (hal.i2c->readRegisters(PX4FLOW_I2C_ADDRESS, 0, I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE, val) != 0) {
            goto fail_transfer;
        }
        memcpy(&f_integral, &(val[I2C_FRAME_SIZE]), I2C_INTEGRAL_FRAME_SIZE);
    }

    if (PX4FLOW_REG == 0x16) {
        if (hal.i2c->readRegisters(PX4FLOW_I2C_ADDRESS, 0, I2C_INTEGRAL_FRAME_SIZE, val) != 0) {
            goto fail_transfer;
        }
        memcpy(&f_integral, val, I2C_INTEGRAL_FRAME_SIZE);
    }

    i2c_sem->give();

    // reduce error count
    if (num_errors > 0) {
        num_errors--;
    }

    report->pixel_flow_x_integral = static_cast<float>(f_integral.pixel_flow_x_integral) / 10000.0f;    //convert to radians
    report->pixel_flow_y_integral = static_cast<float>(f_integral.pixel_flow_y_integral) / 10000.0f;    //convert to radians
    report->frame_count_since_last_readout = f_integral.frame_count_since_last_readout;
    report->ground_distance_m = static_cast<float>(f_integral.ground_distance) / 1000.0f;               // convert to meters
    report->quality = f_integral.qual;                                                                  // 0:bad, 255 max quality
    report->gyro_x_rate_integral = static_cast<float>(f_integral.gyro_x_rate_integral) / 10000.0f;      // convert to radians
    report->gyro_y_rate_integral = static_cast<float>(f_integral.gyro_y_rate_integral) / 10000.0f;      // convert to radians
    report->gyro_z_rate_integral = static_cast<float>(f_integral.gyro_z_rate_integral) / 10000.0f;      // convert to radians
    report->integration_timespan = f_integral.integration_timespan;     // microseconds
    report->time_since_last_sonar_update = f_integral.sonar_timestamp;  // microseconds
    report->gyro_temperature = f_integral.gyro_temperature;             // Temperature * 100 in centi-degrees Celsius
    report->sensor_id = 0;

    return true;

fail_transfer:
    num_errors++;
    i2c_sem->give();
    return false;
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_Linux::update(void)
{
    optical_flow_s report;

    // return immediately if not initialised or more than half of last 40 reads have failed
    if (!initialised || num_errors >= 20) {
        return;
    }

    // throttle reads to no more than 10hz
    uint32_t now = AP_HAL::millis();
    if (now - last_read_ms < 100) {
        return;
    }
    last_read_ms = now;

    // read the report from the sensor
    if (!read(&report)) {
        return;
    }

    // process
    struct OpticalFlow::OpticalFlow_state state;
    state.device_id = report.sensor_id;
    state.surface_quality = report.quality;
    if (report.integration_timespan > 0) {
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        float integralToRate = 1e6f / float(report.integration_timespan);
        state.flowRate.x = flowScaleFactorX * integralToRate * float(report.pixel_flow_x_integral); // rad/sec measured optically about the X sensor axis
        state.flowRate.y = flowScaleFactorY * integralToRate * float(report.pixel_flow_y_integral); // rad/sec measured optically about the Y sensor axis
        state.bodyRate.x = integralToRate * float(report.gyro_x_rate_integral); // rad/sec measured inertially about the X sensor axis
        state.bodyRate.y = integralToRate * float(report.gyro_y_rate_integral); // rad/sec measured inertially about the Y sensor axis
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    // copy results to front end
    _update_frontend(state);

#if PX4FLOW_DEBUG
    hal.console->printf("PX4FLOW id:%u qual:%u FlowRateX:%4.2f Y:%4.2f BodyRateX:%4.2f y:%4.2f\n",
            (unsigned)state.device_id,
            (unsigned)state.surface_quality,
            (double)state.flowRate.x,
            (double)state.flowRate.y,
            (double)state.bodyRate.x,
            (double)state.bodyRate.y);
#endif
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
