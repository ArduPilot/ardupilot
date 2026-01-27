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

class AP_InertialSensor_SCH16T : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:

    AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation);

    struct SensorData {
        int32_t acc_x;
        int32_t acc_y;
        int32_t acc_z;
        int32_t gyro_x;
        int32_t gyro_y;
        int32_t gyro_z;
        int32_t temp;
    };

    struct SensorStatus {
        uint16_t summary;
        uint16_t saturation;
        uint16_t common;
        uint16_t rate_common;
        uint16_t rate_x;
        uint16_t rate_y;
        uint16_t rate_z;
        uint16_t acc_x;
        uint16_t acc_y;
        uint16_t acc_z;
    };

    struct RegisterConfig {
        RegisterConfig() = default;
        RegisterConfig(uint16_t a, uint16_t v)
            : addr(a)
            , value(v)
        {};
        uint8_t addr;
        uint16_t value;
    };

    void run_state_machine(void);
    bool collect_and_publish();

    void reset_chip();
    bool read_product_id();
    void read_status_registers();
    bool validate_sensor_status();
    void configure_registers();
    bool validate_register_configuration();

    bool read_data(SensorData *data);

    void register_write(uint8_t addr, uint16_t value);
    uint64_t register_read(uint8_t addr);
    uint64_t transfer_spi_frame(uint64_t frame);
    uint8_t calculate_crc8(uint64_t frame);

    enum class State : uint8_t {
        PowerOn,
        Reset,
        Configure,
        LockConfiguration,
        Validate,
        Read,
    } _state = State::PowerOn;

    RegisterConfig _registers[6];
    SensorStatus _sensor_status;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;

    uint8_t accel_instance {};
    uint8_t gyro_instance = {};
    enum Rotation rotation {};
    uint8_t drdy_pin {};

    int failure_count {};
    float expected_sample_rate_hz {};

    float accel_scale {};
    float gyro_scale {};
};
