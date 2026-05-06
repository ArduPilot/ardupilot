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
                                            enum Rotation rotation,
                                            uint8_t drdy_gpio=0);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:

    AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                             AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                             enum Rotation _rotation,
                             uint8_t _drdy_gpio);

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

    bool init();

    void reset_chip();
    bool read_product_id();
    void read_status_registers();
    bool validate_sensor_status();
    void configure_registers();
    bool validate_register_configuration();

    bool read_data(SensorData *data);
    void collect_and_publish();

    void register_write(uint8_t addr, uint16_t value);
    uint64_t register_read(uint8_t addr);
    uint64_t register_read_measure(uint8_t addr);
    uint64_t transfer_spi_frame(uint64_t frame);
    uint8_t calculate_crc8(uint64_t frame);

    void loop(void);

    enum class Sch16t_Type : uint8_t {
        SCH16T_K01 = 0, 
        SCH16T_K10 = 1, 
    };

    RegisterConfig _registers[6];
    SensorStatus _sensor_status;

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t accel_instance {};
    uint8_t gyro_instance {};
    enum Rotation rotation {};
    uint8_t drdy_pin {};

    bool done_first_read;
    // int8_t last_gyro_data_cnt = -1;
    // int8_t last_acc_data_cnt = -1;

    // which sensor type this is
    enum Sch16t_Type sch16t_type {};

    float temp_sum {};
    uint32_t temp_cnt {};

    // int failure_count {};
    uint16_t expected_sample_rate_hz {};

    float accel_scale {};
    float gyro_scale {};
};
