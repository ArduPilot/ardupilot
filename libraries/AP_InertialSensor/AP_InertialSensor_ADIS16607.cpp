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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_InertialSensor_ADIS16607.h"

/*
  device registers
 */
#define REG_DEV_ID              0x00
#define DEV_ID_16607            0x6000

#define REG_REV_ID              0x01
#define REG_DIAG_STAT           0x05
#define REG_FIFO_DATA           0x29
#define REG_FIFO_CH_INDEX       0x2A
#define REG_FIFO_WORD_CNT       0x2B
#define REG_WRITE_LOCK          0x2D
#define REG_USER_GPIO_CFG1      0x2F
#define REG_SPI_FULLDUPLEX_KEY  0x31
#define REG_SPI_HALFDUPLEX_KEY  0x32

#define REG_USER_DATA_CFG               0x34
#define USER_DATA_CFG_X_ACCEL_EN        (1U<<0)
#define USER_DATA_CFG_Y_ACCEL_EN        (1U<<1)
#define USER_DATA_CFG_Z_ACCEL_EN        (1U<<2)
#define USER_DATA_CFG_X_GYRO_EN         (1U<<3)
#define USER_DATA_CFG_Y_GYRO_EN         (1U<<4)
#define USER_DATA_CFG_Z_GYRO_EN         (1U<<5)
#define USER_DATA_CFG_TEMPERATURE_EN    (1U<<12)
#define USER_DATA_CFG_TIME_STAMP_EN     (1U<<13)
#define USER_DATA_CFG_DATA_CNTR_EN      (1U<<14)
#define USER_DATA_CFG_WORD_SIZE_16      (0U<<15)
#define USER_DATA_CFG_WORD_SIZE_32      (1U<<15)

#define REG_USER_FIFO_CFG               0x35
#define USER_FIFO_CFG_CLEAR_FIFO_B      (1U<<15)

#define REG_SOFT_RESET          0x36
#define REG_MSC_CTRL            0x39

#define REG_DEC_RATE            0x3A
#define DEC_RATE_9560HZ         0
#define DEC_RATE_4780HZ         1
#define DEC_RATE_3186HZ         2
#define DEC_RATE_2390HZ         3
#define DEC_RATE_1912HZ         4
#define DEC_RATE_1195HZ         7

#define REG_DIGITAL_STATUS      0x4E

#define ADIS16607_FIFO_BUFFER_LEN   4
#define ADIS16607_FIFO_THRESHOLD    0x400U
#define ADIS16607_FIFO_CHANNEL_MAX_INDEX 28 // Data Counter Data

extern const AP_HAL::HAL& hal;

AP_InertialSensor_ADIS16607::AP_InertialSensor_ADIS16607(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_ADIS16607::~AP_InertialSensor_ADIS16607()
{
    if (fifo_buffer != nullptr) {
        hal.util->free_type(fifo_buffer, ADIS16607_FIFO_BUFFER_LEN * ADIS16607_FIFO_SAMPLE_SIZE + 2, AP_HAL::Util::MEM_DMA_SAFE);
    }
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS16607::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_ADIS16607 *sensor = NEW_NOTHROW AP_InertialSensor_ADIS16607(imu, std::move(dev), rotation);

    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS16607::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS16607), "ADIS16607") ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS16607))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    fifo_buffer = hal.util->malloc_type(ADIS16607_FIFO_BUFFER_LEN * ADIS16607_FIFO_SAMPLE_SIZE + 2, AP_HAL::Util::MEM_DMA_SAFE);

    if (fifo_buffer == nullptr) {
        AP_HAL::panic("ADIS16607: Unable to allocate FIFO buffer");
    }

    periodic_handle = dev->register_periodic_callback((1000000UL / backend_rate_hz),
                                                      FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS16607::read_sensor_fifo, void));
}

/**
 * @brief Check dev ID
 */
bool AP_InertialSensor_ADIS16607::check_dev_id()
{
    // Lock the SPI mode
    write_reg16(REG_SPI_HALFDUPLEX_KEY, 0xB4B4, false);

    backend_rate_hz = 1000;
    if (enable_fast_sampling(accel_instance) && (get_fast_sampling_rate() > 1) && (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI)) {
        // constrain the gyro rate to be at least the loop rate
        uint8_t loop_limit = 1;
        if (get_loop_rate_hz() > 1000) {
            loop_limit = 2;
        }
        if (get_loop_rate_hz() > 2000) {
            loop_limit = 4;
        }
        // constrain the gyro rate to be a 2^N multiple
        uint8_t fast_sampling_rate = constrain_int16(get_fast_sampling_rate(), loop_limit, 8);
        backend_rate_hz *= fast_sampling_rate;
    }

    if (read_reg16(REG_DEV_ID) == DEV_ID_16607) {
        accel_scale = GRAVITY_MSS / 200000.0f;  // Accel: Dynamic Range ±40g. 24-bit data format 200000.0 LSB/g
        gyro_scale = radians(1.0 / 4000.0);     // Gyro: ADIS16607-3, 24-bit data format 4000.0 LSB/°/sec
        _clip_limit = 39.5f * GRAVITY_MSS;

        switch (backend_rate_hz) {
            case 2000:
                expected_sample_rate_hz = 2390;
                dec_rate = DEC_RATE_2390HZ;
                break;
            case 4000:
                expected_sample_rate_hz = 4780;
                dec_rate = DEC_RATE_4780HZ;
                break;
            case 8000:
                expected_sample_rate_hz = 9560;
                dec_rate = DEC_RATE_9560HZ;
                break;
            default:
                expected_sample_rate_hz = 1195;
                dec_rate = DEC_RATE_1195HZ;
                break;
        }

        return true;
    }

    return false;
}

bool AP_InertialSensor_ADIS16607::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    uint8_t tries = 5;
    do {
        // Enable SPI Writes to Register Map 
        write_reg16(REG_WRITE_LOCK, 0xAAAA, false);
        write_reg16(REG_WRITE_LOCK, 0x5555, false);
        // Perform software reset
        write_reg16(REG_SOFT_RESET, 0x01);
        // Wait 200 ms for part to come out of reset and bootloader to complete.
        hal.scheduler->delay(200);
    } while (!check_dev_id() && --tries);

    if (tries == 0) {
        return false;
    }

    // Ensure DIGITAL_STATUS is 0x0000 for clear possible false errors
    tries = 10;
    while ((read_reg16(REG_DIAG_STAT) != 0x0000) && --tries) {
        hal.scheduler->delay(10);
    }

    if (tries == 0) {
        return false;
    }

    // Ensure that the "BOOTLOAD_BUSY" bit (Bit 0) is low
    tries = 10;
    while ((read_reg16(REG_DIGITAL_STATUS) & 0x01) && --tries) {
        hal.scheduler->delay(10);
    }

    if (tries == 0) {
        return false;
    }

    // Configure GPIO pins(GPIO3 as DR)
    write_reg16(REG_USER_GPIO_CFG1, 0x0200, false);

    // Init USER_DATA_CFG register if we use burst read mode
    const uint16_t user_data_cfg = USER_DATA_CFG_X_ACCEL_EN | USER_DATA_CFG_Y_ACCEL_EN | USER_DATA_CFG_Z_ACCEL_EN |
                                   USER_DATA_CFG_X_GYRO_EN | USER_DATA_CFG_Y_GYRO_EN | USER_DATA_CFG_Z_GYRO_EN |
                                   USER_DATA_CFG_TEMPERATURE_EN | USER_DATA_CFG_DATA_CNTR_EN | USER_DATA_CFG_WORD_SIZE_32;

    if (!write_reg16(REG_USER_DATA_CFG, user_data_cfg, true)) {
        return false;
    }

    // Filter BW. fC=500Hz.
    if (!write_reg16(REG_MSC_CTRL, 0x100, true)) {
        return false;
    }

    /**
     * Bring rate down
     * The actual decimation rate, D, is the DEC_RATE+1. Note that when changing the decimation rate, it is 
     * recommended to first reset DEC_RATE to 0x000 before entering the new value. This 
     * allows the decimation accumulator to reset.
     */
    if (!write_reg16(REG_DEC_RATE, 0, true)) {
        return false;
    }

    if (!write_reg16(REG_DEC_RATE, dec_rate, true)) {
        return false;
    }

    // Clear FIFO data
    if (!write_reg16(REG_USER_FIFO_CFG, USER_FIFO_CFG_CLEAR_FIFO_B | (ADIS16607_FIFO_THRESHOLD << 0), false)) {
        return false;
    }

    // Write lock
    write_reg16(REG_WRITE_LOCK, 0x5555, false);
    write_reg16(REG_WRITE_LOCK, 0xAAAA, false);

    if (read_reg16(REG_WRITE_LOCK) != 1U) {
        return false;
    }

    if (read_reg16(REG_DIAG_STAT) != 0x0000) {
        return false;
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    return true;
}

/**
 * @brief Read a 16 bit register value
 */
inline uint16_t AP_InertialSensor_ADIS16607::read_reg16(uint8_t regnum) const
{
    uint16_t ret = 0;
    uint8_t seq[4] = {0};

    seq[0] = (regnum | 0x80);
    dev->transfer(&seq[0], 1, &seq[1], 3);
    ret = (seq[2]<<8U) | seq[3];

    return ret;
}

/**
 * @brief Write a 16 bit register value
 */
bool AP_InertialSensor_ADIS16607::write_reg16(uint8_t regnum, uint16_t value, bool confirm) const
{
    const uint8_t retries = 8;
    uint8_t seq[3] = {0};

    for (uint8_t i = 0; i < retries; i++) {
        seq[0] = regnum;
        seq[1] = (value>>8) & 0xFF;
        seq[2] = value & 0xFF;
        dev->transfer(seq, sizeof(seq), nullptr, 0);

        if (!confirm || read_reg16(regnum) == value) {
            return true;
        }
    }

    return false;
}

void AP_InertialSensor_ADIS16607::accumulate_samples(const adis_fifo_data *fifo, const uint8_t num)
{
    for (uint8_t i = 0; i < num; i++) {
        const adis_fifo_data &data = fifo[i];

        // Check if we have lost a sample
        // const uint16_t data_counter = be16toh(data.counter);
        // if (done_first_read && (uint16_t(last_counter + 1) != data_counter)) {
        //     lost_sample_cnt += (data_counter - last_counter - 1);
        // }
        // done_first_read = true;
        // last_counter = data_counter;

        Vector3f accel{float(accel_scale * (int32_t((be16toh(data.ax_high) << 16) | be16toh(data.ax_low)) >> 8)),
                    float(accel_scale * (int32_t((be16toh(data.ay_high) << 16) | be16toh(data.ay_low)) >> 8)),
                    float(accel_scale * (int32_t((be16toh(data.az_high) << 16) | be16toh(data.az_low)) >> 8))};
        Vector3f gyro{float(gyro_scale * (int32_t((be16toh(data.gx_high) << 16) | be16toh(data.gx_low)) >> 8)),
                    float(gyro_scale * (int32_t((be16toh(data.gy_high) << 16) | be16toh(data.gy_low)) >> 8)),
                    float(gyro_scale * (int32_t((be16toh(data.gz_high) << 16) | be16toh(data.gz_low)) >> 8))};

        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel);

        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);

        temp_sum += float(int16_t(be16toh(data.temp)) * 0.005f) + 25.0f;
        temp_count++;

        // Publish average temperature at about 20Hz.
        if (temp_count == 50) {
            _publish_temperature(accel_instance, temp_sum/temp_count);
            temp_sum = 0;
            temp_count = 0;
        }
    }
}

/**
 * @brief Read the sensor FIFO value
 */
void AP_InertialSensor_ADIS16607::read_sensor_fifo(void)
{
    uint16_t fifo_word_num = read_reg16(REG_FIFO_WORD_CNT);

    if (fifo_word_num < WORD_PER_SAMPLE) {
        return;
    }

    if (fifo_word_num == 1024) {
        // FIFO full, data may be discontinuity. Reset FIFO
        // Write unlock
        write_reg16(REG_WRITE_LOCK, 0xAAAA, false);
        write_reg16(REG_WRITE_LOCK, 0x5555, false);
        write_reg16(REG_USER_FIFO_CFG, USER_FIFO_CFG_CLEAR_FIFO_B | (ADIS16607_FIFO_THRESHOLD << 0));
        // Write lock
        write_reg16(REG_WRITE_LOCK, 0x5555, false);
        write_reg16(REG_WRITE_LOCK, 0xAAAA, false);
        need_sync = true;
        return;
    }

    uint8_t *buffer = (uint8_t *)fifo_buffer;

    if (need_sync) {
        uint16_t val = read_reg16(REG_FIFO_DATA);
        uint16_t ch = read_reg16(REG_FIFO_CH_INDEX);
        fifo_word_num--;

        if (ch == 0) {
            // Read one block of FIFO data
            buffer[0] = REG_FIFO_DATA | 0x80;
            memset(&buffer[1], 0, (ADIS16607_FIFO_SAMPLE_SIZE - 1));
            if (!dev->transfer_fullduplex(buffer, ADIS16607_FIFO_SAMPLE_SIZE)) {
                return;
            }
            buffer[0] = uint8_t(val >> 8);
            buffer[1] = uint8_t(val & 0xFF);
            accumulate_samples((adis_fifo_data *)&buffer[0], 1);
            need_sync = false;
            fifo_word_num -= (WORD_PER_SAMPLE - 1);
        } else if (ch == ADIS16607_FIFO_CHANNEL_MAX_INDEX) {
            need_sync = false;
        } else {
            // Read to end of FIFO block
            while (fifo_word_num != 0) {
                val = read_reg16(REG_FIFO_DATA);
                ch = read_reg16(REG_FIFO_CH_INDEX);
                fifo_word_num--;
                if (ch == ADIS16607_FIFO_CHANNEL_MAX_INDEX) {
                    need_sync = false;
                    break;
                }
            }
        }
    }

    uint16_t total_block = fifo_word_num / WORD_PER_SAMPLE;
    while (total_block) {
        const uint16_t read_block = MIN(total_block, ADIS16607_FIFO_BUFFER_LEN);
        buffer[0] = REG_FIFO_DATA | 0x80;
        memset(&buffer[1], 0, (read_block * ADIS16607_FIFO_SAMPLE_SIZE + 1));

        if (!dev->transfer_fullduplex(buffer, (read_block * ADIS16607_FIFO_SAMPLE_SIZE + 2))) {
            need_sync = true;
            return;
        }

        accumulate_samples((adis_fifo_data *)&buffer[2], read_block);
        total_block -= read_block;
    }
}

bool AP_InertialSensor_ADIS16607::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
