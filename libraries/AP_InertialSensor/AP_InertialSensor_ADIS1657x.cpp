
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

#include "AP_InertialSensor_ADIS1657x.h"
#include <GCS_MAVLink/GCS.h>
/*
    Device Registeers
*/

#define BIT(x) (1 << (x))
#define REG_PROD_ID 0x72
#define PROD_ID_16575 0x40BF
#define PROD_ID_16576 0x40C0
#define PROD_ID_16577 0x40C1

// Hardware Filter Control - either 0 or 1, used to control@compile_time the use of hardware filtering
#ifndef HAL_ADIS1657X_FILT_FILTER
#define HAL_ADIS1657X_FILT_FILTER 1
#endif

// 3 bit value for setting, at cojmpile time, the strength of the filter
// Check the datasheet for info
#ifndef HAL_ADIS1657X_FILT_FILTER_STRENGTH
#define HAL_ADIS1657X_FILT_FILTER_STRENGTH 0b000
#endif

#define REG_FILT_CTRL 0x5C

#define REG_DIAG_STAT 0x04
#define REG_FIFO_CTRL 0x5A
#define REG_FIFO_CTRL_WM_ENABLE BIT(2)
#define REG_FIFO_CTRL_OVERFLOW BIT(1)
#define REG_FIFO_CTRL_ENABLE BIT(0)
#define REG_FIFO_CNT 0x3C
// This can be used to identify if the device is high range or low range
#define REG_RNG_MDL 0x5E
#define RNG_MDL_LOW_RANGE 0b0111
#define RNG_MDL_HIGH_RANGE 0b1111
#define REG_MSC_CTRL 0x60
#define REG_MSC_CTRL_BURST32 BIT(9)
#define REG_MSC_CTRL_BURSTSEL BIT(8)
#define REG_MSC_CTRL_GCOMP BIT(7)
#define REG_MSC_CTRL_PCOMP BIT(6)
#define REG_MSC_CTRL_DR_POL BIT(0)
#define REG_GLOB_CMD 0x68
#define REG_GLOB_CMD_SW_RESET BIT(7)

// Decimation rate slashes the polling rate by the value in the register + 1
// Example: for sampling at 1000Hz, you need to set Dec Rate to 1
#define REG_DEC_RATE 0x64

#define ADIS1657X_SPI_STALL_US  5
#define ADIS1657X_SW_RESET_MS   350
#define ADIS1657X_HW_RESET_MS   350

extern const AP_HAL::HAL &hal;

static inline uint16_t assemble16(uint8_t lsb, uint8_t msb)
{
    return msb << 8 | lsb;
}

static inline int32_t assemble32(uint16_t lsb_field, uint16_t msb_field)
{
    return (int32_t)((uint32_t)be16toh(msb_field) << 16 |
                     (uint16_t)be16toh(lsb_field));
};

AP_InertialSensor_ADIS1657x::AP_InertialSensor_ADIS1657x(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation,
                                                         uint8_t drdy_gpio,
                                                         uint8_t rst_gpio)
    : AP_InertialSensor_Backend(imu),
      dev(std::move(_dev)),
      rotation(_rotation),
      drdy_pin(drdy_gpio),
      rst_pin(rst_gpio),
      dec_rate(0),
      last_data_cntr(0),
      temp_sum(0.0f),
      temp_count(0),
      expected_sample_rate_hz(0),
      backend_rate_hz(0),
      accel_scale(0.0f),
      gyro_scale(0.0f),
      _error_count(0)
{
}

AP_InertialSensor_ADIS1657x::~AP_InertialSensor_ADIS1657x()
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS1657x::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                   enum Rotation _rotation,
                                   uint8_t drdy_gpio,
                                   uint8_t rst_gpio)
{
    if (!_dev)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: probe failed - no device");
        return nullptr;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: probe starting");
    auto sensor = NEW_NOTHROW AP_InertialSensor_ADIS1657x(imu, std::move(_dev), _rotation, drdy_gpio, rst_gpio);

    if (!sensor)
    {
        return nullptr;
    }

    if (!sensor->init())
    {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

uint16_t AP_InertialSensor_ADIS1657x::read_reg16(uint8_t regnum) const
{
    uint8_t req[2] = {regnum, 0};
    uint8_t reply[2]{};
    dev->transfer(req, sizeof(req), nullptr, 0);
    hal.scheduler->delay_microseconds(ADIS1657X_SPI_STALL_US);
    dev->transfer(nullptr, 0, reply, sizeof(reply));

    return assemble16(reply[1], reply[0]);
}

bool AP_InertialSensor_ADIS1657x::write_reg(uint8_t regnum, uint16_t value, bool confirm) const
{
    const uint8_t retries = 16;
    for (uint8_t i = 0; i < retries; i++)
    {
        uint8_t req[2];
        req[0] = (regnum | 0x80);
        req[1] = value & 0xFF;
        dev->transfer(req, sizeof(req), nullptr, 0);
        hal.scheduler->delay_microseconds(ADIS1657X_SPI_STALL_US);

        req[0] = ((regnum + 1) | 0x80);
        req[1] = (value >> 8) & 0xFF;
        dev->transfer(req, sizeof(req), nullptr, 0);
        hal.scheduler->delay_microseconds(ADIS1657X_SPI_STALL_US);

        if (!confirm || read_reg16(regnum) == value)
        {
            return true;
        }
    }
    return false;
}

bool AP_InertialSensor_ADIS1657x::check_product_id(uint16_t &prod_id)
{
    prod_id = read_reg16(REG_PROD_ID);
    switch (prod_id)
    {
    case PROD_ID_16575:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: Found product: ADIS16575");
        accel_scale = 100.0f / (float)0x7FFFFFFF; 
        break;
    case PROD_ID_16576:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: Found product: ADIS16576");
        accel_scale = 125.0f / (float)0x7FFFFFFF; 
        break;

    case PROD_ID_16577:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: Found product: ADIS16577");
        accel_scale = 400.0f / (float)0x7FFFFFFF; 
        break;
    default:
        return false;
    }
    // Gyro scale: 1/40 or 1/10 deg/LSB is the 16-bit sensitivity.
    // BURST32 outputs 32-bit values, so divide by 65536 to get rad/s per 32-bit LSB.
    uint16_t rng_mdl = read_reg16(REG_RNG_MDL);
    if (rng_mdl == RNG_MDL_LOW_RANGE)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: gyro range low (+-450 deg/s)");
        gyro_scale = radians(720.0f) / (float)0x7FFFFFFF;
    }
    else if (rng_mdl == RNG_MDL_HIGH_RANGE)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: gyro range high (+-2000 deg/s)");
        gyro_scale = radians(2160.0f) / (float)0x7FFFFFFF;
    }
    else
    {
        return false;
    }

    return true;
}

void AP_InertialSensor_ADIS1657x::start()
{
    // Infer backend rate at runtime based on loop rate and fast sampling capability
    // Must be done after register_accel so accel_instance is valid
    backend_rate_hz = 1000;
    if (enable_fast_sampling(accel_instance)) {
        if (get_loop_rate_hz() >= 2000) {
            backend_rate_hz = 2000;
        }
    }
    expected_sample_rate_hz = backend_rate_hz;
    // output_rate = 2000 / (1 + dec_rate) => dec_rate = (2000 / rate) - 1
    dec_rate = (2000U / backend_rate_hz) - 1U;

    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1657X)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1657X)))
    {
        return;
    }
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    {
        WITH_SEMAPHORE(dev->get_semaphore());
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: rate=%uHz dec_rate=%u", (unsigned)backend_rate_hz, (unsigned)dec_rate);
        write_reg(REG_DEC_RATE, dec_rate);
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: starting at %uHz", (unsigned)backend_rate_hz);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS1657x::loop, void),
                                      "ADIS1657x",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1))
    {
        AP_HAL::panic("Failed to create ADIS1657x thread");
    }
}

bool AP_InertialSensor_ADIS1657x::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    // Hardware reset via RST pin (active low) if available
    if (rst_pin != 0) {
        hal.gpio->pinMode(rst_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(rst_pin, 0);           // assert reset
        hal.scheduler->delay_microseconds(100);
        hal.gpio->write(rst_pin, 1);           // release reset
        hal.scheduler->delay(ADIS1657X_HW_RESET_MS);
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ADIS1657X: hardware reset done");
    }

    uint8_t tries = 10;
    uint16_t prod_id = 0;
    do
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: reset attempt %u/10", (unsigned)(11-tries));
        write_reg(REG_GLOB_CMD, REG_GLOB_CMD_SW_RESET);
        hal.scheduler->delay(ADIS1657X_SW_RESET_MS);
        prod_id = read_reg16(REG_PROD_ID);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: PROD_ID=0x%04X", (unsigned)prod_id);
    } while (!check_product_id(prod_id) && --tries);
    if (tries == 0)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADIS1657X: Initialization FAILED! Last PROD_ID=0x%04X", (unsigned)prod_id);
        return false;
    }

    // Enable burst 32-bit mode, data-ready active high
    write_reg(REG_MSC_CTRL, REG_MSC_CTRL_GCOMP | REG_MSC_CTRL_BURSTSEL | REG_MSC_CTRL_PCOMP | REG_MSC_CTRL_DR_POL | REG_MSC_CTRL_BURST32);

    if (HAL_ADIS1657X_FILT_FILTER)
    {
        write_reg(REG_FILT_CTRL, (uint16_t)HAL_ADIS1657X_FILT_FILTER_STRENGTH);
    }

    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    return true;
}

void AP_InertialSensor_ADIS1657x::loop(void)
{
    while (true)
    {
        uint32_t tstart = AP_HAL::micros();
        const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
        bool wait_ok = false;
        if (drdy_pin != 0)
        {
            wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, 2100);
        }
        read_sensor32();

        if (_error_count >= 10) {
            recover();
        }

        uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us)
        {
            uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us / 2)
            {
                hal.scheduler->delay_microseconds(wait_us);
            }
        }
    }
}

bool AP_InertialSensor_ADIS1657x::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

void AP_InertialSensor_ADIS1657x::accumulate_samples(const adis_burst_data *data)
{
    Vector3f gyro{
        float(gyro_scale * assemble32(data->x_gyro_lsb, data->x_gyro_msb)),
        float(gyro_scale * assemble32(data->y_gyro_lsb, data->y_gyro_msb)),
        float(gyro_scale * assemble32(data->z_gyro_lsb, data->z_gyro_msb))};
    Vector3f accel{
        float(accel_scale * assemble32(data->x_accel_lsb, data->x_accel_msb)),
        float(accel_scale * assemble32(data->y_accel_lsb, data->y_accel_msb)),
        float(accel_scale * assemble32(data->z_accel_lsb, data->z_accel_msb))};

    _notify_new_delta_angle(gyro_instance, gyro);
    _notify_new_delta_velocity(accel_instance, accel);

    temp_sum += int16_t(be16toh(data->temp)) * 0.1f;
    if (++temp_count == 50)
    {
        _publish_temperature(accel_instance, temp_sum / temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
}

void AP_InertialSensor_ADIS1657x::read_sensor32(void)
{
    adis_burst_data data{};
    WITH_SEMAPHORE(dev->get_semaphore());
    data.cmd[0] = REG_GLOB_CMD;
    if (!dev->transfer_fullduplex((uint8_t *)&data, sizeof(data)))
    {
        _error_count++;
        return;
    }

    // datasheet: byte-wise sum of all words excluding DIAG_STAT equals SPI_CHKSUM
    uint16_t sum = 0;
    const uint8_t *p = (const uint8_t *)&data.x_gyro_lsb;
    const uint8_t *end = (const uint8_t *)&data.spi_chksum;
    while (p < end) {
        sum += *p++;
    }
    if (sum != be16toh(data.spi_chksum)) {
        _error_count++;
        return;
    }

    // Check DIAG_STAT from burst frame - non-zero means sensor error
    uint16_t diag = be16toh(data.diag_stat);
    if (diag != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADIS1657X: DIAG_STAT=0x%04X", (unsigned)diag);
        _error_count++;
        return;
    }

    // spi_chksum == 0 means all data bytes summed to zero — all-zero frame, reject it
    if (data.spi_chksum == 0) {
        _error_count++;
        return;
    }

    // Discard stale samples — if data_cntr hasn't advanced the sensor hasn't produced new data
    uint16_t cntr = be16toh(data.data_cntr);
    if (cntr == last_data_cntr) {
        return;
    }
    last_data_cntr = cntr;

    _error_count = 0;
    accumulate_samples(&data);
}

void AP_InertialSensor_ADIS1657x::recover(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADIS1657X: recovering after %u errors", (unsigned)_error_count);
    _error_count = 0;

    WITH_SEMAPHORE(dev->get_semaphore());

    if (rst_pin != 0) {
        hal.gpio->write(rst_pin, 0);
        hal.scheduler->delay_microseconds(100);
        hal.gpio->write(rst_pin, 1);
        hal.scheduler->delay(ADIS1657X_HW_RESET_MS);
    } else {
        write_reg(REG_GLOB_CMD, REG_GLOB_CMD_SW_RESET);
        hal.scheduler->delay(ADIS1657X_SW_RESET_MS);
    }

    write_reg(REG_MSC_CTRL, REG_MSC_CTRL_GCOMP | REG_MSC_CTRL_BURSTSEL | REG_MSC_CTRL_PCOMP | REG_MSC_CTRL_DR_POL | REG_MSC_CTRL_BURST32);
    if (HAL_ADIS1657X_FILT_FILTER) {
        write_reg(REG_FILT_CTRL, (uint16_t)HAL_ADIS1657X_FILT_FILTER_STRENGTH);
    }
    write_reg(REG_DEC_RATE, dec_rate);
}
