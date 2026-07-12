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


struct PACKED adis_1650x_burst_data
{
    uint8_t cmd[2];
    uint16_t diag_stat;
    uint16_t x_gyro_lsb;
    uint16_t x_gyro_msb;
    uint16_t y_gyro_lsb;
    uint16_t y_gyro_msb;
    uint16_t z_gyro_lsb;
    uint16_t z_gyro_msb;
    uint16_t x_accel_lsb;
    uint16_t x_accel_msb;
    uint16_t y_accel_lsb;
    uint16_t y_accel_msb;
    uint16_t z_accel_lsb;
    uint16_t z_accel_msb;
    uint16_t temp;
    uint16_t data_cntr;
    uint16_t spi_chksum;    // 16-bit byte-wise sum of all words including DIAG_STAT
};


class AP_InertialSensor_ADIS1650x : public AP_InertialSensor_Backend {
public:
    virtual ~AP_InertialSensor_ADIS1650x();
    static AP_InertialSensor_Backend* probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                            enum Rotation _rotation,
                                            uint8_t drdy_gpio,
                                            uint8_t rst_gpio=0);
    
    void start() override;
    bool update() override;

private: 
    AP_InertialSensor_ADIS1650x(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                enum Rotation _rotation,
                                uint8_t drdy_gpio,
                                uint8_t rst_gpio);
    
    bool init();
    void loop(void);
    void read_sensor32_delta(void);
    void recover(void);
    //This function sets accel and gyro rate based on product id
    bool check_product_id(uint16_t &prod_id);
    uint16_t read_reg16(uint8_t regnum) const;
    bool write_reg(uint8_t regnum, uint16_t value, bool confirm=false) const;
    void accumulate_samples_delta(const adis_1650x_burst_data *data);
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
    uint8_t drdy_pin;
    uint8_t rst_pin;
    uint16_t dec_rate;

    uint16_t last_data_cntr;

    float temp_sum;
    uint8_t temp_count;
    uint16_t expected_sample_rate_hz;
    uint16_t backend_rate_hz;
    float daccel_scale;
    float dgyro_scale;
    uint8_t _error_count;
};
