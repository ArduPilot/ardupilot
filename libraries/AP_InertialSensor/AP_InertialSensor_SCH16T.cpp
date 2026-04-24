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
static constexpr uint16_t FILTER_LPF5 = (0b0000000101101101);           // LPF5, Gyro: 235Hz, Accel: 210Hz
static constexpr uint16_t RATE_300DPS_1475HZ = (0b0001001011011011);    // Gyro XYZ range 300 deg/s @ 1475Hz, K01
static constexpr uint16_t RATE_2000DPS_1475HZ_K10_200LSB = (0b0011011011011011); // Gyro XYZ range 2000 deg/s @ 1475Hz, 200 LSB/(°/s)
static constexpr uint16_t RATE_2000DPS_2950HZ_K10 = (0b0011011010010010); // Gyro XYZ range 2000 deg/s @ 2950Hz, 200 LSB/(°/s)
static constexpr uint16_t ACC12_8G_1475HZ = (0b0001001011011011);   // Acc XYZ range 8G and 1475Hz, 3200 LSB/(m/s^2)
static constexpr uint16_t ACC12_8G_2950HZ = (0b0001001010010010);   // Acc XYZ range 8G and 2950Hz, 3200 LSB/(m/s^2)
static constexpr uint16_t ACC3_26G = (0b000 << 0);  // 8G measurement range, 26G dynamic range, 1600 LSB/(m/s^2)
static constexpr uint16_t SPI_SOFT_RESET = (0b1010);

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

#define SPI48_DATA_INT32(a)     (((int32_t)(((a) << 4)  & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a)    ((uint32_t)(((a) >> 8)  & 0x000fffffUL))
#define SPI48_DATA_UINT16(a)    ((uint16_t)(((a) >> 8)  & 0x0000ffffUL))
#define SPI48_DATA_GET_DATA_COUNTER(a) ((uint8_t)(((a) >> 29) & 0x0000000fUL))

#define TIMING_DEBUG    0

// use crc table for save some cpu.
static constexpr uint8_t read_data_crc_table[] = {
    0xAC, 0x9A, 0x6D, 0xF6, 0x01, 0x37, 0xC0, 0x2E, 0xD9, 0xEF, 0x18, 0x83, 0x74, 0x42, 0xB5, 0xB1
};

extern const AP_HAL::HAL& hal;

AP_InertialSensor_SCH16T::AP_InertialSensor_SCH16T(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                   enum Rotation _rotation,
                                                   uint8_t _drdy_gpio)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(_drdy_gpio)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_SCH16T::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                uint8_t drdy_gpio)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_SCH16T *sensor = NEW_NOTHROW AP_InertialSensor_SCH16T(imu, std::move(dev), rotation, drdy_gpio);

    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_SCH16T::init()
{
    // Wait 32ms for sensor NVM read and SPI start up
    hal.scheduler->delay(32);

    WITH_SEMAPHORE(dev->get_semaphore());

    if (!read_product_id()) {
        return false;
    }

    reset_chip();

    hal.scheduler->delay(50);

    if (!read_product_id()) {
        return false;
    }

    _registers[0] = RegisterConfig(CTRL_FILT_RATE, FILTER_LPF5);
    _registers[1] = RegisterConfig(CTRL_FILT_ACC12, FILTER_LPF5);
    _registers[2] = RegisterConfig(CTRL_FILT_ACC3, FILTER_LPF5);

    // Configure and enable sensor
    configure_registers();

    hal.scheduler->delay(216);  // after enable sensor, we need wait at least 215ms, from datasheet

    // Read all status registers once
    read_status_registers();

    register_write(CTRL_MODE, (EOI | EN_SENSOR)); // Write EOI and EN_SENSOR

    hal.scheduler->delay(4);    // from datasheet

    // Read all status registers twice
    read_status_registers();
    read_status_registers();

    // Check that registers are configured properly and that the sensor status is OK
    if (!(validate_sensor_status() && validate_register_configuration())) {
        return false;
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
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

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SCH16T::loop, void), "SCH16T",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create SCH16T thread");
    }
}

void AP_InertialSensor_SCH16T::collect_and_publish()
{
    SensorData data = {};
    bool success = read_data(&data);
    if (success) {
        Vector3f accel{accel_scale*data.acc_x, accel_scale*data.acc_y, accel_scale*data.acc_z};
        Vector3f gyro{gyro_scale*data.gyro_x, gyro_scale*data.gyro_y, gyro_scale*data.gyro_z};

        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel, 0);

        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, 0);

        temp_sum += float(data.temp) * 0.01f;
        temp_cnt++;

        // publish temperature at 50Hz
        if (temp_cnt == uint32_t(expected_sample_rate_hz / 50)) {
            _publish_temperature(accel_instance, temp_sum / temp_cnt);
            temp_sum = 0.0f;
            temp_cnt = 0;
        }
    } else {
        _inc_accel_error_count(accel_instance);
        _inc_gyro_error_count(gyro_instance);
    }
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
    uint64_t gyro_x = 0;
    uint64_t gyro_y = 0;
    uint64_t gyro_z = 0;
    uint64_t acc_x  = 0;
    uint64_t acc_y  = 0;
    uint64_t acc_z  = 0;
    uint64_t temp   = 0;

    {
        WITH_SEMAPHORE(dev->get_semaphore());

        if (!done_first_read) {
            (void)register_read_measure(TEMP);
            done_first_read = true;
        }

        temp = register_read_measure(RATE_X2);
        gyro_x = register_read_measure(RATE_Y2);
        gyro_y = register_read_measure(RATE_Z2);
        gyro_z = register_read_measure(ACC_X2);
        acc_x  = register_read_measure(ACC_Y2);
        acc_y  = register_read_measure(ACC_Z2);
        acc_z  = register_read_measure(TEMP);
    }

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

#if TIMING_DEBUG
    // Data counter only available on RATE_*2 and ACC_*2
    const uint8_t gyro_data_cnt[] = {
        SPI48_DATA_GET_DATA_COUNTER(gyro_x),
        SPI48_DATA_GET_DATA_COUNTER(gyro_y),
        SPI48_DATA_GET_DATA_COUNTER(gyro_z)
    };
    const uint8_t acc_data_cnt[] = {
        SPI48_DATA_GET_DATA_COUNTER(acc_x),
        SPI48_DATA_GET_DATA_COUNTER(acc_y),
        SPI48_DATA_GET_DATA_COUNTER(acc_z)
    };

    if ((gyro_data_cnt[0] != gyro_data_cnt[1]) || (gyro_data_cnt[0] != gyro_data_cnt[2])) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gyro cnt error\n");
    } else if ((last_gyro_data_cnt != -1) && (gyro_data_cnt[2] != (last_gyro_data_cnt + 1) % 16)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "gyro lost cnt\n");
    }
    last_gyro_data_cnt = gyro_data_cnt[2];

    if ((acc_data_cnt[0] != acc_data_cnt[1]) || (acc_data_cnt[0] != acc_data_cnt[2])) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "acc cnt error\n");
    } else if ((last_acc_data_cnt != -1) && (acc_data_cnt[2] != (last_acc_data_cnt + 1) % 16)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "acc lost cnt\n");
    }
    last_acc_data_cnt = acc_data_cnt[2];
#endif

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
    // SCH16T-K10   -   ID hex = 0x0021
    bool success = false;
    if ((asic_id == 0x20 && comp_id == 0x17) || (asic_id == 0x21 && comp_id == 0x23)) {
        // SCH16T-K01
        accel_scale = 1.f / 3200.f;
        gyro_scale = radians(1.f / 1600.f);
        sch16t_type = Sch16t_Type::SCH16T_K01;
        _registers[3] = RegisterConfig(CTRL_RATE, RATE_300DPS_1475HZ);  // +/- 300 deg/s, 1600 LSB/(deg/s), Decimation 8, 1475Hz
        _registers[4] = RegisterConfig(CTRL_ACC12, ACC12_8G_1475HZ);    // +/- 80 m/s^2, 3200 LSB/(m/s^2), Decimation 8, 1475Hz
        _registers[5] = RegisterConfig(CTRL_ACC3, ACC3_26G);            // +/- 80 m/s^2, 1600 LSB/(m/s^2)
        expected_sample_rate_hz = 1475;
        _clip_limit = (8.0f - 0.2f) * GRAVITY_MSS;
        success = true;
    } else if (asic_id == 0x21 && comp_id == 0x21) {
        // SCH16T-K10, 20-bit mode  
        accel_scale = 1.f / 3200.f;
        gyro_scale = radians(1.f / 200.f);
        sch16t_type = Sch16t_Type::SCH16T_K10;

        bool fast_sample = false;
        if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate()) {
            fast_sample = (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI);
        }

        if (fast_sample) {
            _registers[3] = RegisterConfig(CTRL_RATE, RATE_2000DPS_2950HZ_K10); // +/- 2000 deg/s, 200 LSB/(deg/s), Decimation 4, 2950Hz
            _registers[4] = RegisterConfig(CTRL_ACC12, ACC12_8G_2950HZ);        // +/- 80 m/s^2, 3200 LSB/(m/s^2), Decimation 4, 2950Hz
            expected_sample_rate_hz = 2950;
        } else {
            _registers[3] = RegisterConfig(CTRL_RATE, RATE_2000DPS_1475HZ_K10_200LSB);  // +/- 2000 deg/s, 200 LSB/(deg/s), Decimation 8, 1475Hz
            _registers[4] = RegisterConfig(CTRL_ACC12, ACC12_8G_1475HZ);        // +/- 80 m/s^2, 3200 LSB/(m/s^2), Decimation 8, 1475Hz
            expected_sample_rate_hz = 1475;
        }

        _registers[5] = RegisterConfig(CTRL_ACC3, ACC3_26G);    // +/- 80 m/s^2, 1600 LSB/(m/s^2)
        _clip_limit = (16.0f - 0.2f) * GRAVITY_MSS;
        success = true;
    }

    return success;
}

void AP_InertialSensor_SCH16T::configure_registers()
{
    for (auto &r : _registers) {
        register_write(r.addr, r.value);
    }

    register_read(CTRL_USER_IF);
    uint16_t ctrl_user_if = SPI48_DATA_UINT16(register_read(CTRL_USER_IF));
    ctrl_user_if |= DRY_DRV_EN;

    register_write(CTRL_USER_IF, ctrl_user_if); // Enable data ready
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

// This function only use for reading sensor measurement data.
uint64_t AP_InertialSensor_SCH16T::register_read_measure(uint8_t addr)
{
    uint8_t buf[6] = {0};
    buf[0] = addr >> 2;
    buf[1] = ((addr & 0x03) << 6) | (1U << 3);
    buf[5] = read_data_crc_table[addr - 1];

    dev->transfer_fullduplex(buf, 6);

    uint64_t value = {};
    for (uint8_t i = 0; i < 6; i++) {
        value |= (uint64_t(buf[i]) << (40 - 8 * i));
    }

    return value;
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
    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++) {
        buf[i] = (frame >> (40 - 8 * i)) & 0xFF;
    }

    dev->transfer_fullduplex(buf, 6);

    uint64_t value = {};
    for (uint8_t i = 0; i < 6; i++) {
        value |= (uint64_t(buf[i]) << (40 - 8 * i));
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

/**
 * @brief Sensor read loop
 */
void AP_InertialSensor_SCH16T::loop(void)
{
    const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
    while (true) {
        uint32_t tstart = AP_HAL::micros();
        // we deliberately set the period a bit fast to ensure we don't lose a sample
        bool wait_ok = false;
        if (drdy_pin != 0) {
            // when we have a DRDY pin then wait for rising edge
            wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, 800);
        }
        collect_and_publish();
        uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us) {
            uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us/2) {
                hal.scheduler->delay_microseconds(wait_us);
            }
        }
    }
}

bool AP_InertialSensor_SCH16T::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
