/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  driver for Invensensev3 IMUs

  Supported:
   ICM-40609
   ICM-42688
   ICM-42605
   ICM-40605 - EOL
   IIM-42652
   ICM-42670
   ICM-45686

  Note that this sensor includes 32kHz internal sampling and an
  anti-aliasing filter, which means this driver can be a lot simpler
  than the Invensense and Invensensev2 drivers which need to handle
  8kHz sample rates to achieve decent aliasing protection
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_Invensensev3.h"
#include <utility>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// set bit 0x80 in register ID for read on SPI
#define BIT_READ_FLAG                           0x80

// registers we use
#define INV3REG_WHOAMI        0x75
#define INV3REG_FIFO_CONFIG   0x16
#define INV3REG_PWR_MGMT0     0x4e
#define INV3REG_GYRO_CONFIG0  0x4f
#define INV3REG_ACCEL_CONFIG0 0x50
#define INV3REG_GYRO_CONFIG1  0x51
#define INV3REG_GYRO_ACCEL_CONFIG0 0x52
#define INV3REG_ACCEL_CONFIG1 0x53
#define INV3REG_FIFO_CONFIG1  0x5f
#define INV3REG_FIFO_CONFIG2  0x60
#define INV3REG_FIFO_CONFIG3  0x61
#define INV3REG_SIGNAL_PATH_RESET 0x4b
#define INV3REG_INTF_CONFIG0  0x4c
#define INV3REG_FIFO_COUNTH   0x2e
#define INV3REG_FIFO_DATA     0x30
#define INV3REG_BANK_SEL      0x76
#define INV3REG_DEVICE_CONFIG 0x11
#define INV3REG_INT_STATUS    0x2D

// ICM42688 bank1
#define INV3REG_GYRO_CONFIG_STATIC2 0x0B
#define INV3REG_GYRO_CONFIG_STATIC3 0x0C
#define INV3REG_GYRO_CONFIG_STATIC4 0x0D
#define INV3REG_GYRO_CONFIG_STATIC5 0x0E

// ICM42688 bank2
#define INV3REG_ACCEL_CONFIG_STATIC2 0x03
#define INV3REG_ACCEL_CONFIG_STATIC3 0x04
#define INV3REG_ACCEL_CONFIG_STATIC4 0x05

// registers for ICM-42670, multi-bank
#define INV3REG_70_PWR_MGMT0  0x1F
#define INV3REG_70_GYRO_CONFIG0  0x20
#define INV3REG_70_GYRO_CONFIG1  0x23
#define INV3REG_70_ACCEL_CONFIG0 0x21
#define INV3REG_70_ACCEL_CONFIG1 0x24
#define INV3REG_70_FIFO_COUNTH   0x3D
#define INV3REG_70_FIFO_DATA     0x3F
#define INV3REG_70_INTF_CONFIG0  0x35
#define INV3REG_70_MCLK_RDY      0x00
#define INV3REG_70_SIGNAL_PATH_RESET 0x02
#define INV3REG_70_FIFO_CONFIG1  0x28
#define INV3REG_BLK_SEL_W 0x79
#define INV3REG_BLK_SEL_R 0x7C
#define INV3REG_MADDR_W   0x7A
#define INV3REG_MADDR_R   0x7D
#define INV3REG_M_W       0x7B
#define INV3REG_M_R       0x7E
#define INV3REG_BANK_MREG1 0x00
#define INV3REG_BANK_MREG2 0x28
#define INV3REG_BANK_MREG3 0x50

#define INV3REG_MREG1_FIFO_CONFIG5 0x1
#define INV3REG_MREG1_SENSOR_CONFIG3 0x06

#define INV3REG_456_WHOAMI          0x72
#define INV3REG_456_PWR_MGMT0       0x10
#define INV3REG_456_INT1_STATUS0    0x19
#define INV3REG_456_ACCEL_CONFIG0   0x1B
#define INV3REG_456_GYRO_CONFIG0    0x1C
#define INV3REG_456_FIFO_CONFIG0    0x1D
#define INV3REG_456_FIFO_CONFIG2    0x20
#define INV3REG_456_FIFO_CONFIG3    0x21
#define INV3REG_456_FIFO_CONFIG4    0x22
#define INV3REG_456_RTC_CONFIG      0x26
#define INV3REG_456_FIFO_COUNTH     0x12
#define INV3REG_456_FIFO_COUNTL     0x13
#define INV3REG_456_FIFO_DATA       0x14
#define INV3REG_456_INTF_CONFIG0    0x2C
#define INV3REG_456_IOC_PAD_SCENARIO 0x2F
#define INV3REG_456_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define INV3REG_456_IOC_PAD_SCENARIO_OVRD 0x31
#define INV3REG_456_PWR_MGMT_AUX1   0x54
#define INV3REG_456_IREG_ADDRH      0x7C
#define INV3REG_456_IREG_ADDRL      0x7D
#define INV3REG_456_IREG_DATA       0x7E
#define INV3REG_456_REG_MISC2       0x7F
#define INV3REG_456_SREG_CTRL       0x63

#define INV3BANK_456_IMEM_SRAM_ADDR 0x0000
#define INV3BANK_456_IPREG_BAR_ADDR 0xA000
#define INV3BANK_456_IPREG_TOP1_ADDR 0xA200
#define INV3BANK_456_IPREG_SYS1_ADDR 0xA400
#define INV3BANK_456_IPREG_SYS2_ADDR 0xA500

// ICM42xxx specific registers
#define INV3REG_42XXX_INTF_CONFIG1  0x4d

// WHOAMI values
#define INV3_ID_ICM40605      0x33
#define INV3_ID_ICM40609      0x3b
#define INV3_ID_ICM42605      0x42
#define INV3_ID_ICM42688      0x47
#define INV3_ID_IIM42652      0x6f
#define INV3_ID_ICM42670      0x67
#define INV3_ID_ICM45686      0xE9

// enable logging at FIFO rate for debugging
#define INV3_ENABLE_FIFO_LOGGING 0

/*
  really nice that this sensor has an option to request little-endian
  data
 */
struct PACKED FIFOData {
    uint8_t header;
    int16_t accel[3];
    int16_t gyro[3];
    int8_t temperature;
    uint16_t timestamp;
};

struct PACKED FIFODataHighRes {
    uint8_t header;
    uint8_t accel[6];
    uint8_t gyro[6];
    int16_t temperature;
    uint16_t timestamp;
    uint8_t gx : 4, ax : 4, gy : 4, ay : 4, gz : 4, az : 4;
};

#define INV3_SAMPLE_SIZE sizeof(FIFOData)
#define INV3_HIGHRES_SAMPLE_SIZE sizeof(FIFODataHighRes)

static_assert(sizeof(FIFOData) == 16);
static_assert(sizeof(FIFODataHighRes) == 20);

#define INV3_FIFO_BUFFER_LEN 8

AP_InertialSensor_Invensensev3::AP_InertialSensor_Invensensev3(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                               enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , rotation(_rotation)
    , dev(std::move(_dev))
{
}

AP_InertialSensor_Invensensev3::~AP_InertialSensor_Invensensev3()
{
#if HAL_INS_HIGHRES_SAMPLE
    if (highres_sampling) {
        if (fifo_buffer != nullptr) {
            hal.util->free_type(fifo_buffer, INV3_FIFO_BUFFER_LEN * INV3_HIGHRES_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
    } else
#endif
    if (fifo_buffer != nullptr) {
        hal.util->free_type(fifo_buffer, INV3_FIFO_BUFFER_LEN * INV3_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_Invensensev3::probe(AP_InertialSensor &imu,
                                                                 AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                                 enum Rotation _rotation)
{
    if (!_dev) {
        return nullptr;
    }

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(BIT_READ_FLAG);
    }

    AP_InertialSensor_Invensensev3 *sensor =
        new AP_InertialSensor_Invensensev3(imu, std::move(_dev), _rotation);
    if (!sensor || !sensor->hardware_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_Invensensev3::fifo_reset()
{
    if (inv3_type == Invensensev3_Type::ICM42670) {
        // FIFO_FLUSH
        register_write(INV3REG_70_SIGNAL_PATH_RESET, 0x04);
    } else if (inv3_type == Invensensev3_Type::ICM45686) {
        // FIFO_FLUSH
        register_write(INV3REG_456_FIFO_CONFIG2, 0x80);
        register_write(INV3REG_456_FIFO_CONFIG2, 0x00, true);
    } else {
        // FIFO_MODE stop-on-full
        register_write(INV3REG_FIFO_CONFIG, 0x80);
        // FIFO partial disable, enable accel, gyro, temperature
        register_write(INV3REG_FIFO_CONFIG1, fifo_config1);
        // little-endian, fifo count in records, last data hold for ODR mismatch
        register_write(INV3REG_INTF_CONFIG0, 0xC0);
        register_write(INV3REG_SIGNAL_PATH_RESET, 2);
    }

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

void AP_InertialSensor_Invensensev3::start()
{
    // pre-fetch instance numbers for checking fast sampling settings
    if (!_imu.get_gyro_instance(gyro_instance) || !_imu.get_accel_instance(accel_instance)) {
        return;
    }
    WITH_SEMAPHORE(dev->get_semaphore());

    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    enum DevTypes devtype = DEVTYPE_INS_ICM42688;
    fifo_config1 = 0x07;

    switch (inv3_type) {
    case Invensensev3_Type::IIM42652:
        devtype = DEVTYPE_INS_IIM42652;
        temp_sensitivity = 1.0 / 2.07;
        break;
    case Invensensev3_Type::ICM42688:
        devtype = DEVTYPE_INS_ICM42688;
        temp_sensitivity = 1.0 / 2.07;
        break;
    case Invensensev3_Type::ICM42605:
        devtype = DEVTYPE_INS_ICM42605;
        temp_sensitivity = 1.0 / 2.07;
        break;
    case Invensensev3_Type::ICM40605:
        devtype = DEVTYPE_INS_ICM40605;
        fifo_config1 = 0x0F;
        temp_sensitivity = 1.0 * 128 / 115.49;
        break;
    case Invensensev3_Type::ICM42670:
        devtype = DEVTYPE_INS_ICM42670;
        temp_sensitivity = 1.0 / 2.0;
        break;
    case Invensensev3_Type::ICM45686:
        devtype = DEVTYPE_INS_ICM45686;
        temp_sensitivity = 1.0 / 2.0;
        gyro_scale = GYRO_SCALE_4000DPS;
        accel_scale = ACCEL_SCALE_32G;
        break;
    case Invensensev3_Type::ICM40609:
        devtype = DEVTYPE_INS_ICM40609;
        temp_sensitivity = 1.0 / 2.07;
        accel_scale = ACCEL_SCALE_32G;
        break;
    }

#if HAL_INS_HIGHRES_SAMPLE
    // now we know who we are, other things can be checked for
    if (enable_highres_sampling(accel_instance)) {
        switch (inv3_type) {
            case Invensensev3_Type::ICM42688: // HiRes 19bit
            case Invensensev3_Type::IIM42652: // HiRes 19bit
            case Invensensev3_Type::ICM42670: // HiRes 19bit
            case Invensensev3_Type::ICM45686: // HiRes 20bit
                highres_sampling = dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;
                break;
            case Invensensev3_Type::ICM40609: // No HiRes
            case Invensensev3_Type::ICM42605:
            case Invensensev3_Type::ICM40605:
                break;
        }
    }

    // optionally enable high resolution mode
    if (highres_sampling) {
        fifo_config1 |= (1U<<4);  // FIFO_HIRES_EN
        gyro_scale = GYRO_SCALE_HIGHRES_2000DPS;
        accel_scale = ACCEL_SCALE_HIGHRES_16G;
        temp_sensitivity = 1.0 / 132.48;
        if (inv3_type == Invensensev3_Type::ICM45686) {
            temp_sensitivity = 1.0 / 128.0;
            accel_scale = ACCEL_SCALE_HIGHRES_32G;
            gyro_scale = GYRO_SCALE_HIGHRES_4000DPS;
        } else if (inv3_type == Invensensev3_Type::ICM42670) {
            temp_sensitivity = 1.0 / 128.0;
        }
    }
#endif
    // always use FIFO
    fifo_reset();

    // setup on-sensor filtering and scaling and backend rate
    if (inv3_type == Invensensev3_Type::ICM42670) {
        set_filter_and_scaling_icm42670();
    } else if (inv3_type == Invensensev3_Type::ICM45686) {
        set_filter_and_scaling_icm456xy();
    } else {
        set_filter_and_scaling();
    }

    // pre-calculate backend period
    backend_period_us = 1000000UL / backend_rate_hz;

    if (!_imu.register_gyro(gyro_instance, backend_rate_hz, dev->get_bus_id_devtype(devtype)) ||
        !_imu.register_accel(accel_instance, backend_rate_hz, dev->get_bus_id_devtype(devtype))) {
        return;
    }

    // update backend sample rate
    _set_accel_raw_sample_rate(accel_instance, backend_rate_hz);
    _set_gyro_raw_sample_rate(gyro_instance, backend_rate_hz);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
    _set_raw_sample_accel_multiplier(accel_instance, multiplier_accel);

    // now that we have initialised, we set the bus speed to high
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // allocate fifo buffer
#if HAL_INS_HIGHRES_SAMPLE
    if (highres_sampling) {
        fifo_buffer = hal.util->malloc_type(INV3_FIFO_BUFFER_LEN * INV3_HIGHRES_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    } else
#endif
    fifo_buffer = hal.util->malloc_type(INV3_FIFO_BUFFER_LEN * INV3_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);

    if (fifo_buffer == nullptr) {
        AP_HAL::panic("Invensensev3: Unable to allocate FIFO buffer");
    }

    // start the timer process to read samples, using the fastest rate avilable
    periodic_handle = dev->register_periodic_callback(backend_period_us, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensensev3::read_fifo, void));
}

// get a startup banner to output to the GCS
bool AP_InertialSensor_Invensensev3::get_output_banner(char* banner, uint8_t banner_len) {
    if (fast_sampling) {
        snprintf(banner, banner_len, "IMU%u: fast%s sampling enabled %.1fkHz",
            gyro_instance,
#if HAL_INS_HIGHRES_SAMPLE
            highres_sampling ? ", high-resolution" :
#endif
            "" , backend_rate_hz * 0.001);
        return true;
    }
    return false;
}

/*
  publish any pending data
 */
bool AP_InertialSensor_Invensensev3::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    _publish_temperature(accel_instance, temp_filtered);

    return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_Invensensev3::accumulate()
{
    // nothing to do
}

bool AP_InertialSensor_Invensensev3::accumulate_samples(const FIFOData *data, uint8_t n_samples)
{
#if INV3_ENABLE_FIFO_LOGGING
    const uint64_t tstart = AP_HAL::micros64();
#endif
    for (uint8_t i = 0; i < n_samples; i++) {
        const FIFOData &d = data[i];

        // we have a header to confirm we don't have FIFO corruption! no more mucking
        // about with the temperature registers
        // ICM45686 - TMST_FIELD_EN bit 3 : 1
        // ICM42688 - HEADER_TIMESTAMP_FSYNC bit 2-3 : 10
        if ((d.header & 0xFC) != 0x68) { // ACCEL_EN | GYRO_EN | TMST_FIELD_EN
            // no or bad data
            return false;
        }

        Vector3f accel{float(d.accel[0]), float(d.accel[1]), float(d.accel[2])};
        Vector3f gyro{float(d.gyro[0]), float(d.gyro[1]), float(d.gyro[2])};

        accel *= accel_scale;
        gyro *= gyro_scale;

#if INV3_ENABLE_FIFO_LOGGING
        Write_GYR(gyro_instance, tstart+(i*backend_period_us), gyro, true);
#endif

        const float temp = d.temperature * temp_sensitivity + temp_zero;

        // these four calls are about 40us
        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);

        _notify_new_accel_raw_sample(accel_instance, accel, 0);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);

        temp_filtered = temp_filter.apply(temp);
    }
    return true;
}

#if HAL_INS_HIGHRES_SAMPLE
// high-resolution packets are always 20-bits, but not always 20-bits of data.
// Scale factors account for the useless bits
static inline float uint20_to_float(uint8_t msb, uint8_t bits, uint8_t lsb)
{
    uint32_t value20bit = uint32_t(msb) << 12U | uint32_t(bits) << 4U | lsb;
    int32_t value32bit;
    // Check the sign bit (MSB)
    if (value20bit & 0x80000) { // MSB is set (negative value)
        // Extend the sign bit to the upper 12 bits of the 32-bit integer
        value32bit = (int32_t)(value20bit | 0xFFF00000);
    } else { // MSB is not set (positive value)
        // Zero-fill the upper 12 bits of the 32-bit integer
        value32bit = value20bit;
    }

    return float(value32bit);
}


bool AP_InertialSensor_Invensensev3::accumulate_highres_samples(const FIFODataHighRes *data, uint8_t n_samples)
{
#if INV3_ENABLE_FIFO_LOGGING
    const uint64_t tstart = AP_HAL::micros64();
#endif
    for (uint8_t i = 0; i < n_samples; i++) {
        const FIFODataHighRes &d = data[i];

        // we have a header to confirm we don't have FIFO corruption! no more mucking
        // about with the temperature registers
        if ((d.header & 0xFC) != 0x78) { // ACCEL_EN | GYRO_EN | HIRES_EN | TMST_FIELD_EN
            // no or bad data
            return false;
        }

        Vector3f accel{uint20_to_float(d.accel[1], d.accel[0], d.ax),
            uint20_to_float(d.accel[3], d.accel[2], d.ay),
            uint20_to_float(d.accel[5], d.accel[4], d.az)};
        Vector3f gyro{uint20_to_float(d.gyro[1], d.gyro[0], d.gx),
            uint20_to_float(d.gyro[3], d.gyro[2], d.gy),
            uint20_to_float(d.gyro[5], d.gyro[4], d.gz)};

        accel *= accel_scale;
        gyro *= gyro_scale;

#if INV3_ENABLE_FIFO_LOGGING
        Write_GYR(gyro_instance, tstart+(i*backend_period_us), gyro, true);
#endif
        const float temp = d.temperature * temp_sensitivity + temp_zero;

        // these four calls are about 40us
        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);

        _notify_new_accel_raw_sample(accel_instance, accel, 0);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);

        temp_filtered = temp_filter.apply(temp);
    }
    return true;
}
#endif

/*
  timer function called at ODR rate
 */
void AP_InertialSensor_Invensensev3::read_fifo()
{
    bool need_reset = false;
    uint16_t n_samples;

    uint8_t reg_counth;
    uint8_t reg_data;

    switch (inv3_type) {
    case Invensensev3_Type::ICM45686:
        reg_counth = INV3REG_456_FIFO_COUNTH;
        reg_data = INV3REG_456_FIFO_DATA;
        break;
    case Invensensev3_Type::ICM42670:
        reg_counth = INV3REG_70_FIFO_COUNTH;
        reg_data = INV3REG_70_FIFO_DATA;
        break;
    default:
        reg_counth = INV3REG_FIFO_COUNTH;
        reg_data = INV3REG_FIFO_DATA;
        break;
    }

#if HAL_INS_HIGHRES_SAMPLE
    const uint8_t fifo_sample_size = highres_sampling ? INV3_HIGHRES_SAMPLE_SIZE : INV3_SAMPLE_SIZE;
#else
    const uint8_t fifo_sample_size = INV3_SAMPLE_SIZE;
#endif
    if (!block_read(reg_counth, (uint8_t*)&n_samples, 2)) {
        goto check_registers;
    }

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        goto check_registers;
    }

    // adjust the periodic callback to be synchronous with the incoming data
    // this means that we rarely run read_fifo() without updating the sensor data
    dev->adjust_periodic_callback(periodic_handle, backend_period_us);

    while (n_samples > 0) {
        uint8_t n = MIN(n_samples, INV3_FIFO_BUFFER_LEN);
        if (!block_read(reg_data, (uint8_t*)fifo_buffer, n * fifo_sample_size)) {
            goto check_registers;
        }
#if HAL_INS_HIGHRES_SAMPLE
        if (highres_sampling) {
            if (!accumulate_highres_samples((FIFODataHighRes*)fifo_buffer, n)) {
                need_reset = true;
                break;
            }
        } else
#endif
        if (!accumulate_samples((FIFOData*)fifo_buffer, n)) {
            need_reset = true;
            break;
        }
        n_samples -= n;
    }

    if (need_reset) {
        fifo_reset();
    }
    
check_registers:
    // check next register value for correctness
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    AP_HAL::Device::checkreg reg;
    if (!dev->check_next_register(reg)) {
        log_register_change(dev->get_bus_id(), reg);
        _inc_gyro_error_count(gyro_instance);
        _inc_accel_error_count(accel_instance);
    }
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

bool AP_InertialSensor_Invensensev3::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_Invensensev3::register_read(uint8_t reg)
{
    uint8_t val = 0;
    dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_Invensensev3::register_write(uint8_t reg, uint8_t val, bool checked)
{
    dev->write_register(reg, val, checked);
}

/*
  read a bank register, only used on startup
 */
uint8_t AP_InertialSensor_Invensensev3::register_read_bank(uint8_t bank, uint8_t reg)
{
    if (inv3_type == Invensensev3_Type::ICM42670) {
        // the ICM42670 has a complex bank setup
        register_write(INV3REG_BLK_SEL_R, bank);
        register_write(INV3REG_MADDR_R, reg);
        hal.scheduler->delay_microseconds(10);
        const uint8_t val = register_read(INV3REG_M_R);
        hal.scheduler->delay_microseconds(10);
        register_write(INV3REG_BLK_SEL_R, 0);
        return val;
    }
    register_write(INV3REG_BANK_SEL, bank);
    const uint8_t val = register_read(reg);
    register_write(INV3REG_BANK_SEL, 0);
    return val;
}

/*
  write to a bank register. This is only used on startup, so can use
  sleeps to wait for success
 */
void AP_InertialSensor_Invensensev3::register_write_bank(uint8_t bank, uint8_t reg, uint8_t val)
{
    if (inv3_type == Invensensev3_Type::ICM42670) {
        // the ICM42670 has a complex bank setup
        register_write(INV3REG_BLK_SEL_W, bank);
        register_write(INV3REG_MADDR_W, reg);
        register_write(INV3REG_M_W, val);
        hal.scheduler->delay_microseconds(10);
        register_write(INV3REG_BLK_SEL_W, 0);
        hal.scheduler->delay_microseconds(10);
    } else {
        register_write(INV3REG_BANK_SEL, bank);
        register_write(reg, val);
        register_write(INV3REG_BANK_SEL, 0);
    }
}

// calculate the fast sampling backend rate
uint16_t AP_InertialSensor_Invensensev3::calculate_fast_sampling_backend_rate(uint16_t base_odr, uint16_t max_odr) const
{
    // constrain the gyro rate to be at least the loop rate
    uint8_t loop_limit = 1;
    if (get_loop_rate_hz() > base_odr) {
        loop_limit = 2;
    }
    if (get_loop_rate_hz() > base_odr * 2) {
        loop_limit = 4;
    }
    // constrain the gyro rate to be a 2^N multiple
    uint8_t fast_sampling_rate = constrain_int16(get_fast_sampling_rate(), loop_limit, 8);

    // calculate rate we will be giving samples to the backend
    return constrain_int16(base_odr * fast_sampling_rate, base_odr, max_odr);
}

/*
  set the filter frequencies and scaling

  The AAF for gyros needs to be high enough to avoid group delay and low enough to have
  (ideally) 40dB at the nyquist frequency so that noise above this is not folded into the
  range seen by ArduPilot. A reasonable approximation for the former is 1Khz and for the latter
  1/4 of the sample frequency, so for 1/4 sample frequency > 1Khz we pick 1Khz and for 1/4 sample
  frequency < 1Khz we use 1/4 sample frequency.

  The AAF for accels is set lower to minimise noise and clipping. The constraint is that the
  group delay between gyros and accels should be <5ms to avoid inertial nav errors.

  The UI filter block cannot be disabled and is fixed at ODR/4. This is a 2p filter by default
  (as is the AAF). Since the order of the UI filter does not appear to significantly affect
  group delay at higher ODRs it is left at the default. The group delay of the AAF is not documented,
  but we assume it is similar to the UI 2p performance:

  2Khz  - 0.2ms
  1Khz  - 0.4ms
  666Hz - 0.6ms
  500Hz - 0.8ms
  333Hz - 2.0ms
  190Hz - 2.4ms

  Since the UI group delay is the same for both accels and gyros we only need to consider the
  difference in group delay for the AAFs. At the highest ODR of 4Khz or 8Khz the group delay for
  gyros will be 0.4ms thus the accel AAF can safely be set to ~190Hz.
 */
void AP_InertialSensor_Invensensev3::set_filter_and_scaling(void)
{
    // 1KHz by default
    backend_rate_hz = 1000;
    uint8_t odr_config = 0x06;

    // AAF at ~1/4 of 1Khz by default for gyros- 258Hz
    // AAF at 213Hz for accels
    uint8_t aaf_delt = 6, accel_aaf_delt = 5;
    uint16_t aaf_deltsqr = 36, accel_aaf_deltsqr = 25;
    uint8_t aaf_bitshift = 10, accel_aaf_bitshift = 10;

    // limited filtering on ICM-42605
    if (inv3_type == Invensensev3_Type::ICM42605) {
        // 249Hz AAF gyros
        aaf_delt = 21;
        aaf_deltsqr = 440;
        aaf_bitshift = 6;
        // 184Hz AAF accels
        accel_aaf_delt = 16;
        accel_aaf_deltsqr = 256;
        accel_aaf_bitshift = 7;
    }

    // checked for
    // ICM-40609
    // ICM-42688
    // ICM-42605
    // IIM-42652
    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        fast_sampling = dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;

        if (fast_sampling) {
            backend_rate_hz = calculate_fast_sampling_backend_rate(backend_rate_hz, 8 * backend_rate_hz);

            // limited filtering on ICM-42605
            if (inv3_type == Invensensev3_Type::ICM42605) {
                switch (backend_rate_hz) {
                    case 2000: // 2KHz
                        odr_config = 0x05;
                        // 507Hz AAF
                        aaf_delt = 47;
                        aaf_deltsqr = 2208;
                        aaf_bitshift = 4;
                        break;
                    case 4000: // 4KHz
                        // 995Hz AAF
                        aaf_delt = 63;
                        aaf_deltsqr = 3968;
                        aaf_bitshift = 3;
                        odr_config = 0x04;
                        break;
                    case 8000: // 8Khz
                        // 995Hz AAF
                        aaf_delt = 63;
                        aaf_deltsqr = 3968;
                        aaf_bitshift = 3;
                        odr_config = 0x03;
                        break;
                    default: // 1Khz, 334Hz AAF
                        break;
                }
            } else {
                // ICM-42688 / ICM-40609 / IIM-426525
                switch (backend_rate_hz) {
                    case 2000: // 2KHz
                        odr_config = 0x05;
                        // 536Hz AAF
                        aaf_delt = 12;
                        aaf_deltsqr = 144;
                        aaf_bitshift = 8;
                        break;
                    case 4000: // 4KHz
                        odr_config = 0x04;
                        // 997Hz AAF
                        aaf_delt = 21;
                        aaf_deltsqr = 440;
                        aaf_bitshift = 6;
                        break;
                    case 8000: // 8Khz
                        odr_config = 0x03;
                        // 997Hz AAF
                        aaf_delt = 21;
                        aaf_deltsqr = 440;
                        aaf_bitshift = 6;
                        break;
                    default: // 1KHz, 348Hz AAF
                        break;
                }
            }
        }
    }

    // disable gyro and accel as per 12.9 in the ICM-42688 docs
    register_write(INV3REG_PWR_MGMT0, 0x00);

    // setup gyro for backend rate
    register_write(INV3REG_GYRO_CONFIG0, odr_config);
    // setup accel for backend rate
    register_write(INV3REG_ACCEL_CONFIG0, odr_config);

    // setup anti-alias filters for gyro at 1/4 ODR, notch left at default
    // The defaults for 42605 and 42609 are different to the 42688, make sure AAF and notch are enabled on all
    uint8_t aaf_enable = register_read_bank(1, INV3REG_GYRO_CONFIG_STATIC2);
    register_write_bank(1, INV3REG_GYRO_CONFIG_STATIC2, aaf_enable & ~0x03);
    register_write_bank(1, INV3REG_GYRO_CONFIG_STATIC3, aaf_delt);   // GYRO_AAF_DELT
    register_write_bank(1, INV3REG_GYRO_CONFIG_STATIC4, (aaf_deltsqr & 0xFF)); // GYRO_AAF_DELTSQR
    register_write_bank(1, INV3REG_GYRO_CONFIG_STATIC5, ((aaf_bitshift<<4) & 0xF0) | ((aaf_deltsqr>>8) & 0x0F)); // GYRO_AAF_BITSHIFT | GYRO_AAF_DELTSQR

    // setup accel AAF at fixed ~500Hz
    register_write_bank(2, INV3REG_ACCEL_CONFIG_STATIC2, accel_aaf_delt<<1); // ACCEL_AAF_DELT | enabled bit
    register_write_bank(2, INV3REG_ACCEL_CONFIG_STATIC3, (accel_aaf_deltsqr & 0xFF)); // ACCEL_AAF_DELTSQR
    register_write_bank(2, INV3REG_ACCEL_CONFIG_STATIC4, ((accel_aaf_bitshift<<4) & 0xF0) | ((accel_aaf_deltsqr>>8) & 0x0F)); // ACCEL_AAF_BITSHIFT | ACCEL_AAF_DELTSQR

    switch (inv3_type) {
    case Invensensev3_Type::ICM42688:
    case Invensensev3_Type::ICM42605:
    case Invensensev3_Type::IIM42652:
    case Invensensev3_Type::ICM42670: {
        /*
          fix for the "stuck gyro" issue, which affects all IxM42xxx
          sensors. This disables the AFSR feature which changes the
          noise sensitivity with angular rate. When the switch happens
          (at around 100 deg/sec) the gyro gets stuck for around 2ms,
          producing constant output which causes a DC gyro bias
        */
        const uint8_t v = register_read(INV3REG_42XXX_INTF_CONFIG1);
        register_write(INV3REG_42XXX_INTF_CONFIG1, (v & 0x3F) | 0x40, true);
        break;
    }
    case Invensensev3_Type::ICM40605:
    case Invensensev3_Type::ICM40609:
    case Invensensev3_Type::ICM45686:
        break;
    }

    // enable gyro and accel in low-noise modes
    register_write(INV3REG_PWR_MGMT0, 0x0F);
    hal.scheduler->delay_microseconds(300);
}

/*
  set the filter frequencies and scaling for the ICM-42670
 */
void AP_InertialSensor_Invensensev3::set_filter_and_scaling_icm42670(void)
{
    backend_rate_hz = 1600;
    // use low-noise mode
    register_write(INV3REG_70_PWR_MGMT0, 0x0f);
    hal.scheduler->delay_microseconds(300);

    // setup gyro for 1.6kHz, 2000dps range
    register_write(INV3REG_70_GYRO_CONFIG0, 0x05);
    // Low noise mode uses an AAF with fixed bandwidth, so disable LPF
    register_write(INV3REG_70_GYRO_CONFIG1, 0x30);

    // setup accel for 1.6kHz, 16g range
    register_write(INV3REG_70_ACCEL_CONFIG0, 0x05);
    // AAF is not available for accels, so LPF at 180Hz
    register_write(INV3REG_70_ACCEL_CONFIG1, 0x01);

    // fix "stuck" gyro issue
    const uint8_t v = register_read(INV3REG_42XXX_INTF_CONFIG1);
    register_write(INV3REG_42XXX_INTF_CONFIG1, (v & 0x3F) | 0x40);
}

/*
  set the filter frequencies and scaling for the ICM-456xy
 */
void AP_InertialSensor_Invensensev3::set_filter_and_scaling_icm456xy(void)
{
    uint8_t odr_config = 4;
    backend_rate_hz = 1600;
    // always fast sampling
    fast_sampling = dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;

    if (enable_fast_sampling(accel_instance) && get_fast_sampling_rate() > 1) {
        backend_rate_hz = calculate_fast_sampling_backend_rate(backend_rate_hz, backend_rate_hz * 4);
    }

    // this sensor actually only supports 2 speeds
    backend_rate_hz = constrain_int16(backend_rate_hz, 3200, 6400);

    switch (backend_rate_hz) {
    case 6400: // 6.4Khz
        odr_config = 3;
        break;
    case 3200: // 3.2Khz
        odr_config = 4;
        break;
    default:
        break;
    }

    // Disable FIFO first
    register_write(INV3REG_456_FIFO_CONFIG3, 0x00);
    register_write(INV3REG_456_FIFO_CONFIG0, 0x00);

    // setup gyro for 1.6-6.4kHz, 4000dps range
    register_write(INV3REG_456_GYRO_CONFIG0, (0x0 << 4) | odr_config); // GYRO_UI_FS_SEL b4-7, GYRO_ODR b0-3

    // setup accel for 1.6-6.4kHz, 32g range
    register_write(INV3REG_456_ACCEL_CONFIG0, (0x0 << 4) | odr_config); // ACCEL_UI_FS_SEL b4-6, ACCEL_ODR b0-3

    // enable timestamps on FIFO data 
    // SMC_CONTROL_0
    uint8_t reg = register_read_bank_icm456xy(INV3BANK_456_IPREG_TOP1_ADDR, 0x58);
#ifdef ICM45686_CLKIN
    reg |= (0x1<<4U); // ACCEL_LP_CLK_SEL
#endif
    register_write_bank_icm456xy(INV3BANK_456_IPREG_TOP1_ADDR, 0x58, reg | 0x01);

    uint8_t fifo_config = (1U<<2 | 1U<<1); // FIFO_ACCEL_EN | FIFO_GYRO_EN, FIFO_IF_EN disabled
#if HAL_INS_HIGHRES_SAMPLE
    // optionally enable high resolution mode
    if (highres_sampling) {
        fifo_config |= (1U<<3);  // FIFO_HIRES_EN
    }
#endif
    // enable FIFO for each sensor
    register_write(INV3REG_456_FIFO_CONFIG3, fifo_config, true);

    // FIFO enabled - stop-on-full, disable bypass and 2K FIFO
    register_write(INV3REG_456_FIFO_CONFIG0, (2 << 6) | 0x07, true);

    // enable Interpolator and Anti Aliasing Filter on Gyro
    reg = register_read_bank_icm456xy(INV3BANK_456_IPREG_SYS1_ADDR, 0xA6);  // GYRO_SRC_CTRL b5-6
    register_write_bank_icm456xy(INV3BANK_456_IPREG_SYS1_ADDR, 0xA6, (reg & ~(0x3 << 5)) | (0x2 << 5));

    // enable Interpolator and Anti Aliasing Filter on accel
    reg = register_read_bank_icm456xy(INV3BANK_456_IPREG_SYS2_ADDR, 0x7B); // ACCEL_SRC_CTRL b0-1
    register_write_bank_icm456xy(INV3BANK_456_IPREG_SYS2_ADDR, 0x7B, (reg & ~0x3) | 0x2);

    // enable FIFO sensor registers
    fifo_config |= (1U<<0);  // FIFO_IF_EN
    register_write(INV3REG_456_FIFO_CONFIG3, fifo_config, true);
}

/*
  check whoami for sensor type
 */
bool AP_InertialSensor_Invensensev3::check_whoami(void)
{
    uint8_t whoami = register_read(INV3REG_WHOAMI);

    switch (whoami) {
    case INV3_ID_ICM40609:
        inv3_type = Invensensev3_Type::ICM40609;
        return true;
    case INV3_ID_ICM42688:
        inv3_type = Invensensev3_Type::ICM42688;
        return true;
    case INV3_ID_ICM42605:
        inv3_type = Invensensev3_Type::ICM42605;
        return true;
    case INV3_ID_ICM40605:
        inv3_type = Invensensev3_Type::ICM40605;
        return true;
    case INV3_ID_IIM42652:
        inv3_type = Invensensev3_Type::IIM42652;
        return true;
    case INV3_ID_ICM42670:
        inv3_type = Invensensev3_Type::ICM42670;
        return true;
    }
    // check 456 who am i
    whoami = register_read(INV3REG_456_WHOAMI);
    switch (whoami) {
    case INV3_ID_ICM45686:
        inv3_type = Invensensev3_Type::ICM45686;
        return true;
    }
    // not a value WHOAMI result
    return false;
}

uint8_t AP_InertialSensor_Invensensev3::register_read_bank_icm456xy(uint16_t bank_addr, uint16_t reg)
{
    // combine addr
    uint16_t addr = bank_addr + reg;

    uint8_t send[] = {INV3REG_456_IREG_ADDRH, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};

    // set indirect register address
    dev->transfer(send, sizeof(send), nullptr, 0);

    // try reading IREG_DATA on ready
    for (uint8_t i=0; i<10; i++) {
        if (register_read(INV3REG_456_REG_MISC2) & 0x01) {
            break;
        }
        // minimum wait time-gap between IREG access is 4us
        hal.scheduler->delay_microseconds(10);
    }

    // read the data
    return register_read(INV3REG_456_IREG_DATA);
}

void AP_InertialSensor_Invensensev3::register_write_bank_icm456xy(uint16_t bank_addr, uint16_t reg, uint8_t val)
{
    // combine addr
    uint16_t addr = bank_addr + reg;

    uint8_t send[] = {INV3REG_456_IREG_ADDRH, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), val};

    // set indirect register address
    dev->transfer(send, sizeof(send), nullptr, 0);

    //check if IREG_DATA ready, we can return immediately if so
    if (register_read(INV3REG_456_REG_MISC2) & 0x01) {
        return;
    }
    // minimum wait time-gap between IREG access is 4us
    hal.scheduler->delay_microseconds(10);
}


bool AP_InertialSensor_Invensensev3::hardware_init(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->setup_checked_registers(8, dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C?200:20);

    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!check_whoami()) {
        return false;
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    switch (inv3_type) {
    case Invensensev3_Type::ICM45686:
    case Invensensev3_Type::ICM40609:
        _clip_limit = 29.5f * GRAVITY_MSS;
        break;
    case Invensensev3_Type::ICM42688:
    case Invensensev3_Type::IIM42652:
    case Invensensev3_Type::ICM42605:
    case Invensensev3_Type::ICM40605:
    case Invensensev3_Type::ICM42670:
        _clip_limit = 15.5f * GRAVITY_MSS;
        break;
    }

    if (inv3_type == Invensensev3_Type::ICM42670) {
        // the ICM-42670 needs some more power-up config
        for (uint8_t tries=0; tries<50; tries++) {
            // initiate a power up sequence
            register_write(INV3REG_70_SIGNAL_PATH_RESET, 0x10);
            hal.scheduler->delay_microseconds(1000);
            register_write(INV3REG_70_PWR_MGMT0, 0x0f, true);
            if (register_read(INV3REG_70_MCLK_RDY) != 0) {
                break;
            }
            hal.scheduler->delay(5);
        }
        if (register_read(INV3REG_70_MCLK_RDY) == 0) {
            return false;
        }

        // disable APEX for larger FIFO
        register_write_bank(INV3REG_BANK_MREG1, INV3REG_MREG1_SENSOR_CONFIG3, 0x40);

        // use 16 bit data, gyro+accel
        uint8_t fifo_config = 0x03;
#if HAL_INS_HIGHRES_SAMPLE
        // optionally enable high resolution mode
        if (highres_sampling) {
            fifo_config |= (1U<<3);  // FIFO_HIRES_EN
        }
#endif
        register_write_bank(INV3REG_BANK_MREG1, INV3REG_MREG1_FIFO_CONFIG5, fifo_config);

        // FIFO stop-on-full, disable bypass
        register_write(INV3REG_70_FIFO_CONFIG1, 0x2, true);
        
        // little-endian, fifo count in records
        register_write(INV3REG_70_INTF_CONFIG0, 0x40, true);
    } else if (inv3_type == Invensensev3_Type::ICM45686) {

        // do soft reset
        register_write(INV3REG_456_REG_MISC2, 0x02);
        hal.scheduler->delay_microseconds(1000);
        // check if reset done
        if (!(register_read(INV3REG_456_INT1_STATUS0) & 0x80)) {
            // failed to reset
            return false;
        }
        // turn off aux1
        register_write(INV3REG_456_PWR_MGMT_AUX1, 0x3);

        // gyro and accel in low-noise mode
        register_write(INV3REG_456_PWR_MGMT0, 0x0f);

#ifdef ICM45686_CLKIN
        /*************************CLKIN setting*************************/
        // override INT2 pad as CLKIN, AUX1 disabled
        register_write(INV3REG_456_IOC_PAD_SCENARIO_OVRD, (0x1 << 2)| 0x2 , true);

        // disable AUX1
        register_write(INV3REG_456_IOC_PAD_SCENARIO_AUX_OVRD, (0x1<<1U), true);

        // enable RTC MODE
        register_write(INV3REG_456_RTC_CONFIG, (0x1<<5U));
#endif
        /*************************CLKIN setting*************************/
        // disable STC
        uint8_t reg = register_read_bank_icm456xy(INV3BANK_456_IPREG_TOP1_ADDR, 0x68);  // I3C_STC_MODE b2
        register_write_bank_icm456xy(INV3BANK_456_IPREG_TOP1_ADDR, 0x68, reg & ~0x04);
    }

    return true;
}
