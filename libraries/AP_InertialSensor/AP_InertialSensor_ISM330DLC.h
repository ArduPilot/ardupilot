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
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <Filter/LowPassFilter.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ISM330DLC : public AP_InertialSensor_Backend {
public:
    virtual ~AP_InertialSensor_ISM330DLC() { }

    void start(void) override;
    bool update() override;

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

private:
    AP_InertialSensor_ISM330DLC(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation);

    bool init_sensor();
    bool hardware_init();
    bool chip_reset();
    void common_init();
    void gyro_init();
    void accel_init();
    void fifo_init();
    void fifo_reset();
    void fifo_recover();

    uint8_t register_read(uint8_t reg);
    void register_write(uint8_t reg, uint8_t val, bool checked=false);

    uint16_t fifo_status(uint16_t &pattern);
    void read_temperature();
    void poll_data();

    // gyro is configured for +/-2000 dps: 70 mdps/LSB
    static constexpr float gyro_scale = (70.0f / 1000.0f) * DEG_TO_RAD;
    // accel is configured for +/-16 g: 0.488 mg/LSB
    static constexpr float accel_scale = (0.488f / 1000.0f) * GRAVITY_MSS;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    enum Rotation rot;

    uint8_t temperature_counter;
    bool discard_set;
    float temperature_degc;
    LowPassFilter2pFloat temperature_filter{100.0f, 1.0f};
};
