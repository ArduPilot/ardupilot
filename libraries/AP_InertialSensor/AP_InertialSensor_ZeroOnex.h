/*Add commentMore actions
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

class AP_InertialSensor_ZeroOnex : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_ZeroOnex(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation);

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

struct SensorData {
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t temp;
};

struct RegisterConfig {
        RegisterConfig() = default;
        RegisterConfig(uint16_t a, uint32_t v)
            : addr(a)
            , value(v)
        {};
        uint16_t addr;
        uint32_t value;
    };
    RegisterConfig _registers[6];
    SensorStatus _sensor_status;
    
    void init(void);
    void seneor_ctrl(uint8_t fifo_rst) ;
    void fpga_read_config(void);
    void fpga_write_cmd(uint32_t addr, uint8_t ptr, uint8_t read_valid, uint8_t is_sensor, uint8_t offset) ;
    void run_state_machine();
    bool collect_and_publish();
    bool read_data() ;
    void reset_chip();
    bool read_product_id();
    void configure_registers();
    bool validate_sensor_status();
    bool validate_register_configuration();
    void read_status_registers();
    uint8_t register_read(uint16_t addr, uint32_t* value);
    void register_write(uint16_t addr, uint32_t value);
    uint64_t transfer_spi_frame(uint64_t frame);
    uint8_t calculate_crc8(uint64_t frame);
    void fpga_read_fifo8(uint16_t reg, uint8_t* value, uint8_t size) ;
    void fpga_write_reg8(uint16_t reg, uint8_t value) ;
    uint8_t fpga_read_reg8(uint16_t reg) ;
    void fpga_write_reg16(uint16_t reg, uint16_t value) ;
    void fpga_write_ram8(uint16_t reg, uint16_t ram_addr, uint8_t* value, uint16_t size);
    void fpga_read_ram8(uint16_t reg, uint16_t ram_addr, uint8_t* value, uint16_t size);
    uint8_t wait_direct_busy(void);
    uint16_t fpga_read_reg16(uint16_t reg);
    uint16_t _sch16t_read_status(uint16_t addr);
    uint8_t fpga_test(void);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    AP_HAL::Device::PeriodicHandle periodic_handle;

    enum class State : uint8_t {
        PowerOn,
        Reset,
        Configure,
        LockConfiguration,
        Validate,
        Read,
    } _state = State::PowerOn;

    int failure_count {};
    enum Rotation rotation {};
    uint16_t backend_rate_hz;
    uint8_t accel_instance {};
    uint8_t gyro_instance = {};
    float accel_scale {};
    float gyro_scale {};
};