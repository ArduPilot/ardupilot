/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADIS16607 : public AP_InertialSensor_Backend {
public:
    virtual ~AP_InertialSensor_ADIS16607();

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_ADIS16607(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation);

    struct PACKED adis_fifo_data {
        uint16_t ax_high;
        uint16_t ax_low;
        uint16_t ay_high;
        uint16_t ay_low;
        uint16_t az_high;
        uint16_t az_low;
        uint16_t gx_high;
        uint16_t gx_low;
        uint16_t gy_high;
        uint16_t gy_low;
        uint16_t gz_high;
        uint16_t gz_low;
        uint16_t temp;
        uint16_t counter;
    };

    /*
      initialise driver
     */
    bool init();
    bool check_dev_id();
    void accumulate_samples(const struct adis_fifo_data *fifo, const uint8_t num);
    void read_sensor_fifo(void);

    // read a 16 bit register
    uint16_t read_reg16(uint8_t regnum) const;
    // write a 16 bit register
    bool write_reg16(uint8_t regnum, uint16_t value, bool confirm=false) const;
    
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
    uint16_t dec_rate;

    // uint32_t lost_sample_cnt {};
    // uint16_t last_counter;
    // bool done_first_read;
    float temp_sum;
    uint8_t temp_count;
    uint16_t expected_sample_rate_hz;
    uint16_t backend_rate_hz;
    bool need_sync;

    void *fifo_buffer;

    float accel_scale;
    float gyro_scale;

    // Size of FIFO sample block
    static constexpr uint8_t ADIS16607_FIFO_SAMPLE_SIZE = sizeof(adis_fifo_data);
    // How many word per FIFO sample block
    static constexpr uint8_t WORD_PER_SAMPLE = (ADIS16607_FIFO_SAMPLE_SIZE / 2);
};
