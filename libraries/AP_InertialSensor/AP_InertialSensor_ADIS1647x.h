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
/*
  the ADIS1647x is unusual as it uses 16 bit registers. It also needs
  to run as the only sensor on the SPI bus for good performance
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADIS1647x : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation,
                                            uint8_t drdy_gpio);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_ADIS1647x(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                uint8_t drdy_gpio);

    /*
      initialise driver
     */
    bool init();
    void read_sensor(void);
    void loop(void);
    bool check_product_id();

    // read a 16 bit register
    uint16_t read_reg16(uint8_t regnum) const;

    // write a 16 bit register
    void write_reg16(uint8_t regnum, uint16_t value) const;
    
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
    uint8_t drdy_pin;

    uint16_t last_counter;
    bool done_first_read;
    float temp_sum;
    uint8_t temp_count;

    float accel_scale;
    float gyro_scale;
};
