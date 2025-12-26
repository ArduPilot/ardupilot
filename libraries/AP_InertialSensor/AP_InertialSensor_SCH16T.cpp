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

#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_SCH16T.h"
// #include <GCS_MAVLink/GCS.h>

#if defined(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1)
#include <hal.h>
#endif

static constexpr uint16_t EOI = (1 << 1);               // End of Initialization
static constexpr uint16_t EN_SENSOR = (1 << 0);         // Enable RATE and ACC measurement
static constexpr uint16_t DRY_DRV_EN = (1 << 5);        // Enables Data ready function
static constexpr uint16_t FILTER_68HZ = (0x0000);       // 68 Hz default filter
static constexpr uint16_t FILTER_BYPASS = (0b0000000111111111);     // No filtering
static constexpr uint16_t RATE_300DPS_1475HZ = 0b0001001011011011; // Gyro XYZ range 300 deg/s @ 1475Hz
static constexpr uint16_t ACC12_8G_1475HZ = 0b0001001011011011;  // Acc XYZ range 8 G and 1475 update rate
static constexpr uint16_t ACC3_26G = (0b000 << 0);
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);
static constexpr uint32_t POWER_ON_TIME = 250000UL;

// Data registers
#define RATE_X1         0x01 // 20 bit
#define RATE_Y1         0x02 // 20 bit
#define RATE_Z1         0x03 // 20 bit
#define ACC_X1          0x04 // 20 bit
#define ACC_Y1          0x05 // 20 bit
#define ACC_Z1          0x06 // 20 bit
#define ACC_X3          0x07 // 20 bit
#define ACC_Y3          0x08 // 20 bit
#define ACC_Z3          0x09 // 20 bit
#define RATE_X2         0x0A // 20 bit
#define RATE_Y2         0x0B // 20 bit
#define RATE_Z2         0x0C // 20 bit
#define ACC_X2          0x0D // 20 bit
#define ACC_Y2          0x0E // 20 bit
#define ACC_Z2          0x0F // 20 bit
#define TEMP            0x10 // 16 bit
// Status registers
#define STAT_SUM        0x14 // 16 bit
#define STAT_SUM_SAT    0x15 // 16 bit
#define STAT_COM        0x16 // 16 bit
#define STAT_RATE_COM   0x17 // 16 bit
#define STAT_RATE_X     0x18 // 16 bit
#define STAT_RATE_Y     0x19 // 16 bit
#define STAT_RATE_Z     0x1A // 16 bit
#define STAT_ACC_X      0x1B // 16 bit
#define STAT_ACC_Y      0x1C // 16 bit
#define STAT_ACC_Z      0x1D // 16 bit
// Control registers
#define CTRL_FILT_RATE  0x25 // 9 bit
#define CTRL_FILT_ACC12 0x26 // 9 bit
#define CTRL_FILT_ACC3  0x27 // 9 bit
#define CTRL_RATE       0x28 // 15 bit
#define CTRL_ACC12      0x29 // 15 bit
#define CTRL_ACC3       0x2A // 3 bit
#define CTRL_USER_IF    0x33 // 16 bit
#define CTRL_ST         0x34 // 13 bit
#define CTRL_MODE       0x35 // 4 bit
#define CTRL_RESET      0x36 // 4 bit
// Misc registers
#define ASIC_ID         0x3B // 12 bit
#define COMP_ID         0x3C // 16 bit
#define SN_ID1          0x3D // 16 bit
#define SN_ID2          0x3E // 16 bit
#define SN_ID3          0x3F // 16 bit

#define T_STALL_US   20U

#define SPI48_DATA_INT32(a)     (((int32_t)(((a) << 4)  & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a)    ((uint32_t)(((a) >> 8)  & 0x000fffffUL))
#define SPI48_DATA_UINT16(a)    ((uint16_t)(((a) >> 8)  & 0x0000ffffUL))

extern const AP_HAL::HAL& hal;

AP_InertialSensor_SCH16T::AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
    expected_sample_rate_hz = 1475;
    accel_scale = 1.f / 1600.f;
    gyro_scale = radians(1.f / 1600.f);

    _registers[0] = RegisterConfig(CTRL_FILT_RATE,  FILTER_BYPASS);
    _registers[1] = RegisterConfig(CTRL_FILT_ACC12, FILTER_BYPASS);
    _registers[2] = RegisterConfig(CTRL_FILT_ACC3,  FILTER_BYPASS);
    _registers[3] = RegisterConfig(CTRL_RATE,       RATE_300DPS_1475HZ); // +/- 300 deg/s, 1600 LSB/(deg/s) -- default, Decimation 8, 1475Hz
    _registers[4] = RegisterConfig(CTRL_ACC12,      ACC12_8G_1475HZ);    // +/- 80 m/s^2, 3200 LSB/(m/s^2) -- default, Decimation 8, 1475Hz
    _registers[5] = RegisterConfig(CTRL_ACC3,       ACC3_26G);           // +/- 260 m/s^2, 1600 LSB/(m/s^2) -- default
}

AP_InertialSensor_Backend *
AP_InertialSensor_SCH16T::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    auto sensor = new AP_InertialSensor_SCH16T(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_SCH16T::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_SCH16T))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    uint32_t period_us = 1000000UL / expected_sample_rate_hz;
    periodic_handle = dev->register_periodic_callback(period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCH16T::run_state_machine, void));
}

void AP_InertialSensor_SCH16T::run_state_machine()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    switch (_state) {
    case State::PowerOn: {
            _state = State::Reset;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::Reset: {
            failure_count = 0;
            reset_chip();
            _state = State::Configure;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::Configure: {
            if (!read_product_id()) {
                _state = State::Reset;
                dev->adjust_periodic_callback(periodic_handle, 2000000); // 2s
                break;
            }

            configure_registers();
            _state = State::LockConfiguration;
            dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            break;
        }

    case State::LockConfiguration: {
            read_status_registers(); // Read all status registers once
            register_write(CTRL_MODE, (EOI | EN_SENSOR)); // Write EOI and EN_SENSOR
            _state = State::Validate;
            dev->adjust_periodic_callback(periodic_handle, 50000UL); // 50ms
            break;
        }

    case State::Validate: {
            read_status_registers(); // Read all status registers twice
            read_status_registers();

            // Check that registers are configured properly and that the sensor status is OK
            if (validate_sensor_status() && validate_register_configuration()) {
                _state = State::Read;
                dev->adjust_periodic_callback(periodic_handle, 1000000UL / expected_sample_rate_hz);

            } else {
                _state = State::Reset;
                dev->adjust_periodic_callback(periodic_handle, POWER_ON_TIME);
            }

            break;
        }

    case State::Read: {
            if (collect_and_publish()) {
                if (failure_count > 0) {
                    failure_count--;
                }
            } else {
                failure_count++;
            }

            // Reset if successive failures
            if (failure_count > 10) {
                _state = State::Reset;
                return;
            }

            break;
        }

    default:
        break;
    } // end switch/case
}

bool AP_InertialSensor_SCH16T::collect_and_publish()
{
    SensorData data = {};
    bool success = read_data(&data);
    if (success) {
        Vector3f accel{accel_scale*data.acc_x, accel_scale*data.acc_y, accel_scale*data.acc_z};
        Vector3f gyro{gyro_scale*data.gyro_x, gyro_scale*data.gyro_y, gyro_scale*data.gyro_z};

        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel);

        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);

        _publish_temperature(accel_instance, float(data.temp)/100.f);

        // adjust the periodic callback to be synchronous with the incoming data
        dev->adjust_periodic_callback(periodic_handle, 1000000UL / expected_sample_rate_hz);
    }

    return success;
}

void AP_InertialSensor_SCH16T::reset_chip()
{
#if defined(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1)
    palClearLine(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1);
    hal.scheduler->delay(2000);
    palSetLine(HAL_GPIO_PIN_nSPI6_RESET_EXTERNAL1);
#else
    register_write(CTRL_RESET, SPI_SOFT_RESET);
#endif
}

bool AP_InertialSensor_SCH16T::read_data(SensorData *data)
{
    register_read(RATE_X2);
    uint64_t gyro_x = register_read(RATE_Y2);
    uint64_t gyro_y = register_read(RATE_Z2);
    uint64_t gyro_z = register_read(ACC_X3);
    uint64_t acc_x  = register_read(ACC_Y3);
    uint64_t acc_y  = register_read(ACC_Z3);
    uint64_t acc_z  = register_read(TEMP);
    uint64_t temp   = register_read(TEMP);

    static constexpr uint64_t MASK48_ERROR = 0x001E00000000UL;
    uint64_t values[] = { gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp };

    for (auto v : values) {
        // Check for frame errors
        if (v & MASK48_ERROR) {
            return false;
        }

        // Validate the CRC
        if (uint8_t(v & 0xff) != calculate_crc8(v)) {
            return false;
        }
    }

    // Data registers are 20bit 2s complement
    data->acc_x    = SPI48_DATA_INT32(acc_x);
    data->acc_y    = SPI48_DATA_INT32(acc_y);
    data->acc_z    = SPI48_DATA_INT32(acc_z);
    data->gyro_x   = SPI48_DATA_INT32(gyro_x);
    data->gyro_y   = SPI48_DATA_INT32(gyro_y);
    data->gyro_z   = SPI48_DATA_INT32(gyro_z);
    // Temperature data is always 16 bits wide. Drop 4 LSBs as they are not used.
    data->temp    = SPI48_DATA_INT32(temp) >> 4;

    // Convert to RH coordinate system (FLU to FRD)
    data->acc_x = data->acc_x;
    data->acc_y = -data->acc_y;
    data->acc_z = -data->acc_z;
    data->gyro_x = data->gyro_x;
    data->gyro_y = -data->gyro_y;
    data->gyro_z = -data->gyro_z;

    return true;
}

bool AP_InertialSensor_SCH16T::read_product_id()
{
    register_read(COMP_ID);
    uint16_t comp_id = SPI48_DATA_UINT16(register_read(ASIC_ID));
    uint16_t asic_id = SPI48_DATA_UINT16(register_read(ASIC_ID));

    // Debug code
    // register_read(SN_ID1);
    // uint16_t sn_id1 = SPI48_DATA_UINT16(register_read(SN_ID2));
    // uint16_t sn_id2 = SPI48_DATA_UINT16(register_read(SN_ID3));
    // uint16_t sn_id3 = SPI48_DATA_UINT16(register_read(SN_ID3));

    // char serial_str[14];
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, serial_str, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Serial:\t %s", serial_str);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "COMP_ID:\t 0x%0x", comp_id);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ASIC_ID:\t 0x%0x", asic_id);

    // SCH16T-K01   -   ID hex = 0x0020
    // SCH1633-B13  -   ID hex = 0x0017
    bool success = asic_id == 0x20 && comp_id == 0x17;

    return success;
}

void AP_InertialSensor_SCH16T::configure_registers()
{
    for (auto &r : _registers) {
        register_write(r.addr, r.value);
    }

    register_write(CTRL_USER_IF, DRY_DRV_EN); // Enable data ready
    register_write(CTRL_MODE, EN_SENSOR); // Enable the sensor
}

bool AP_InertialSensor_SCH16T::validate_sensor_status()
{
    auto &s = _sensor_status;
    uint16_t values[] = { s.summary, s.saturation, s.common, s.rate_common, s.rate_x, s.rate_y, s.rate_z, s.acc_x, s.acc_y, s.acc_z };

    for (auto v : values) {
        if (v != 0xFFFF) {
            return false;
        }
    }

    return true;
}

bool AP_InertialSensor_SCH16T::validate_register_configuration()
{
    bool success = true;

    for (auto &r : _registers) {
        register_read(r.addr); // double read, wasteful but makes the code cleaner, not high rate so doesn't matter anyway
        auto value = SPI48_DATA_UINT16(register_read(r.addr));

        if (value != r.value) {
            success = false;
        }
    }

    return success;
}

void AP_InertialSensor_SCH16T::read_status_registers()
{
    register_read(STAT_SUM);
    _sensor_status.summary      = SPI48_DATA_UINT16(register_read(STAT_SUM_SAT));
    _sensor_status.saturation   = SPI48_DATA_UINT16(register_read(STAT_COM));
    _sensor_status.common       = SPI48_DATA_UINT16(register_read(STAT_RATE_COM));
    _sensor_status.rate_common  = SPI48_DATA_UINT16(register_read(STAT_RATE_X));
    _sensor_status.rate_x       = SPI48_DATA_UINT16(register_read(STAT_RATE_Y));
    _sensor_status.rate_y       = SPI48_DATA_UINT16(register_read(STAT_RATE_Z));
    _sensor_status.rate_z       = SPI48_DATA_UINT16(register_read(STAT_ACC_X));
    _sensor_status.acc_x        = SPI48_DATA_UINT16(register_read(STAT_ACC_Y));
    _sensor_status.acc_y        = SPI48_DATA_UINT16(register_read(STAT_ACC_Z));
    _sensor_status.acc_z        = SPI48_DATA_UINT16(register_read(STAT_ACC_Z));
}

uint64_t AP_InertialSensor_SCH16T::register_read(uint8_t addr)
{
    uint64_t frame = {};
    frame |= uint64_t(addr) << 38; // Target address offset
    frame |= uint64_t(1) << 35; // FrameType: SPI48BF
    frame |= uint64_t(calculate_crc8(frame));

    return transfer_spi_frame(frame);
}

// Non-data registers are the only writable ones and are 16 bit or less
void AP_InertialSensor_SCH16T::register_write(uint8_t addr, uint16_t value)
{
    uint64_t frame = {};
    frame |= uint64_t(1) << 37; // Write bit
    frame |= uint64_t(addr) << 38; // Target address offset
    frame |= uint64_t(1) << 35; // FrameType: SPI48BF
    frame |= uint64_t(value) << 8;
    frame |= uint64_t(calculate_crc8(frame));

    // We don't care about the return frame on a write
    (void)transfer_spi_frame(frame);
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint64_t AP_InertialSensor_SCH16T::transfer_spi_frame(uint64_t frame)
{
    uint16_t buf[3];
    for (int index = 0; index < 3; index++) {
        uint16_t lower_byte = (frame >> (index << 4)) & 0xFF;
        uint16_t upper_byte = (frame >> ((index << 4) + 8)) & 0xFF;
        buf[3 - index - 1] = (lower_byte << 8) | upper_byte;
    }

    dev->transfer((uint8_t*)buf, 6, (uint8_t*)buf, 6);

    uint64_t value = {};
    for (int index = 0; index < 3; index++) {
        uint16_t lower_byte = buf[index] & 0xFF;
        uint16_t upper_byte = (buf[index] >> 8) & 0xFF;
        value |= (uint64_t)(upper_byte | (lower_byte << 8)) << ((3 - index - 1) << 4);
    }

    return value;
}

uint8_t AP_InertialSensor_SCH16T::calculate_crc8(uint64_t frame)
{
    uint64_t data = frame & 0xFFFFFFFFFF00LL;
    uint8_t crc = 0xFF;

    for (int i = 47; i >= 0; i--) {
        uint8_t data_bit = data >> i & 0x01;
        crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ 0x2F) ^ data_bit : (uint8_t)(crc << 1) | data_bit;
    }

    return crc;
}

bool AP_InertialSensor_SCH16T::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
