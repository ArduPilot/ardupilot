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
  driver for all supported Invensensev2 IMUs
  ICM-20608 and ICM-20602
 */

#include <assert.h>
#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_Invensensev2.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// hal.console can be accessed from bus threads on ChibiOS
#define debug(fmt, args ...)  do {hal.console->printf("INV2: " fmt "\n", ## args); } while(0)
#else
#define debug(fmt, args ...)  do {printf("INV2: " fmt "\n", ## args); } while(0)
#endif

/*
 *  DS-000189-ICM-20948-v1.3.pdf, page 11, section 3.1 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

/*
  EXT_SYNC allows for frame synchronisation with an external device
  such as a camera. When enabled the LSB of AccelZ holds the FSYNC bit
 */
#ifndef INVENSENSE_EXT_SYNC_ENABLE
#define INVENSENSE_EXT_SYNC_ENABLE 0
#endif

#include "AP_InertialSensor_Invensensev2_registers.h"

#define INV2_SAMPLE_SIZE 14
#define INV2_FIFO_BUFFER_LEN 16

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])


AP_InertialSensor_Invensensev2::AP_InertialSensor_Invensensev2(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                           enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _temp_filter(1125, 1)
    , _rotation(rotation)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_Invensensev2::~AP_InertialSensor_Invensensev2()
{
    if (_fifo_buffer != nullptr) {
        hal.util->free_type(_fifo_buffer, INV2_FIFO_BUFFER_LEN * INV2_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
    _dev->deregister_bankselect_callback();
    //delete _auxiliary_bus;
}

AP_InertialSensor_Backend *AP_InertialSensor_Invensensev2::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Invensensev2 *sensor =
        new AP_InertialSensor_Invensensev2(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    sensor->_id = HAL_INS_INV2_I2C;

    return sensor;
}


AP_InertialSensor_Backend *AP_InertialSensor_Invensensev2::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Invensensev2 *sensor;

    dev->set_read_flag(0x80);

    sensor = new AP_InertialSensor_Invensensev2(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    sensor->_id = HAL_INS_INV2_SPI;

    return sensor;
}

bool AP_InertialSensor_Invensensev2::_init()
{
#ifdef INVENSENSEV2_DRDY_PIN
    _drdy_pin = hal.gpio->channel(INVENSENSEV2_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    bool success = _hardware_init();

    return success;
}

void AP_InertialSensor_Invensensev2::_fifo_reset()
{
    uint8_t user_ctrl = _last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_EN);

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    _register_write(INV2REG_FIFO_EN_2, 0);
    _register_write(INV2REG_USER_CTRL, user_ctrl);
    _register_write(INV2REG_FIFO_RST, 0x0F);
    _register_write(INV2REG_FIFO_RST, 0x00);
    _register_write(INV2REG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    _register_write(INV2REG_FIFO_EN_2, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN, true);
    hal.scheduler->delay_microseconds(1);
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

bool AP_InertialSensor_Invensensev2::_has_auxiliary_bus()
{
    return _dev->bus_type() != AP_HAL::Device::BUS_TYPE_I2C;
}

void AP_InertialSensor_Invensensev2::start()
{
    _dev->get_semaphore()->take_blocking();

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    _register_write(INV2REG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

    // always use FIFO
    _fifo_reset();

    // grab the used instances
    enum DevTypes gdev, adev;
    switch (_inv2_type) {
    case Invensensev2_ICM20648:
        gdev = DEVTYPE_INS_ICM20648;
        adev = DEVTYPE_INS_ICM20648;
        // using 16g full range, 2048 LSB/g
        _accel_scale = (GRAVITY_MSS / 2048);
        break;
    case Invensensev2_ICM20649:
        // 20649 is setup for 30g full scale, 1024 LSB/g
        gdev = DEVTYPE_INS_ICM20649;
        adev = DEVTYPE_INS_ICM20649;
        _accel_scale = (GRAVITY_MSS / 1024);
        break;
    case Invensensev2_ICM20948:
    default:
        gdev = DEVTYPE_INS_ICM20948;
        adev = DEVTYPE_INS_ICM20948;
        // using 16g full range, 2048 LSB/g
        _accel_scale = (GRAVITY_MSS / 2048);
        break;
    }

    _gyro_instance = _imu.register_gyro(1125, _dev->get_bus_id_devtype(gdev));
    _accel_instance = _imu.register_accel(1125, _dev->get_bus_id_devtype(adev));

    // setup on-sensor filtering and scaling
    _set_filter_and_scaling();
#if INVENSENSE_EXT_SYNC_ENABLE
    _register_write(INV2REG_FSYNC_CONFIG, FSYNC_CONFIG_EXT_SYNC_AZ, true);
#endif
    // update backend sample rate
    _set_accel_raw_sample_rate(_accel_instance, _accel_backend_rate_hz);
    _set_gyro_raw_sample_rate(_gyro_instance, _gyro_backend_rate_hz);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
    _set_raw_sample_accel_multiplier(_accel_instance, multiplier_accel);

    // set sample rate to 1.125KHz
    _register_write(INV2REG_GYRO_SMPLRT_DIV, 0, true);
    hal.scheduler->delay(1);
    
    // configure interrupt to fire when new data arrives
    _register_write(INV2REG_INT_ENABLE_1, 0x01);
    hal.scheduler->delay(1);

    // now that we have initialised, we set the bus speed to high
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    // setup scale factors for fifo data after downsampling
    _fifo_accel_scale = _accel_scale / _accel_fifo_downsample_rate;
    _fifo_gyro_scale = GYRO_SCALE / _gyro_fifo_downsample_rate;
    
    // allocate fifo buffer
    _fifo_buffer = (uint8_t *)hal.util->malloc_type(INV2_FIFO_BUFFER_LEN * INV2_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    if (_fifo_buffer == nullptr) {
        AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
    }

    // start the timer process to read samples
    _dev->register_periodic_callback(1265625UL / _gyro_backend_rate_hz, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensensev2::_poll_data, void));
}

// get a startup banner to output to the GCS
bool AP_InertialSensor_Invensensev2::get_output_banner(char* banner, uint8_t banner_len) {
    if (_fast_sampling) {
        snprintf(banner, banner_len, "IMU%u: fast sampling enabled %.1fkHz/%.1fkHz",
            _gyro_instance, _gyro_backend_rate_hz * _gyro_fifo_downsample_rate / 1000.0, _gyro_backend_rate_hz / 1000.0);
        return true;
    }
    return false;
}

/*
  publish any pending data
 */
bool AP_InertialSensor_Invensensev2::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    _publish_temperature(_accel_instance, _temp_filtered);

    return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_Invensensev2::accumulate()
{
    // nothing to do
}

AuxiliaryBus *AP_InertialSensor_Invensensev2::get_auxiliary_bus()
{
    if (_auxiliary_bus) {
        return _auxiliary_bus;
    }

    if (_has_auxiliary_bus()) {
        _auxiliary_bus = new AP_Invensensev2_AuxiliaryBus(*this, _dev->get_bus_id());
    }

    return _auxiliary_bus;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_Invensensev2::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }
    uint8_t status = _register_read(INV2REG_INT_STATUS_1);
    return status != 0;
}

/*
 * Timer process to poll for new data from the Invensense. Called from bus thread with semaphore held
 */
void AP_InertialSensor_Invensensev2::_poll_data()
{
    _read_fifo();
}

bool AP_InertialSensor_Invensensev2::_accumulate(uint8_t *samples, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + INV2_SAMPLE_SIZE * i;
        Vector3f accel, gyro;
        bool fsync_set = false;

#if INVENSENSE_EXT_SYNC_ENABLE
        fsync_set = (int16_val(data, 2) & 1U) != 0;
#endif
        
        accel = Vector3f(int16_val(data, 1),
                         int16_val(data, 0),
                         -int16_val(data, 2));
        accel *= _accel_scale;

        int16_t t2 = int16_val(data, 6);
        if (!_check_raw_temp(t2)) {
            if (!hal.scheduler->in_expected_delay()) {
                debug("temp reset IMU[%u] %d %d", _accel_instance, _raw_temp, t2);
            }
            _fifo_reset();
            return false;
        }
        float temp = t2 * temp_sensitivity + temp_zero;
        
        gyro = Vector3f(int16_val(data, 4),
                        int16_val(data, 3),
                        -int16_val(data, 5));
        gyro *= GYRO_SCALE;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel, 0, fsync_set);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);

        _temp_filtered = _temp_filter.apply(temp);
    }
    return true;
}

/*
  when doing fast sampling the sensor gives us 9k samples/second. Every 2nd accel sample is a duplicate.

  To filter this we first apply a 1p low pass filter at 188Hz, then we
  average over 8 samples to bring the data rate down to 1kHz. This
  gives very good aliasing rejection at frequencies well above what
  can be handled with 1kHz sample rates.
 */
bool AP_InertialSensor_Invensensev2::_accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    int32_t unscaled_clip_limit = _clip_limit / _accel_scale;
    bool clipped = false;
    bool ret = true;

    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + INV2_SAMPLE_SIZE * i;

        // use temperature to detect FIFO corruption
        int16_t t2 = int16_val(data, 6);
        if (!_check_raw_temp(t2)) {
            if (!hal.scheduler->in_expected_delay()) {
                debug("temp reset IMU[%u] %d %d", _accel_instance, _raw_temp, t2);
            }
            _fifo_reset();
            ret = false;
            break;
        }
        tsum += t2;
        if (_accum.gyro_count % 2 == 0) {
            // accel data is at 4kHz or 1kHz
            Vector3f a(int16_val(data, 1),
                       int16_val(data, 0),
                       -int16_val(data, 2));
            if (fabsf(a.x) > unscaled_clip_limit ||
                fabsf(a.y) > unscaled_clip_limit ||
                fabsf(a.z) > unscaled_clip_limit) {
                clipped = true;
            }
            _accum.accel += _accum.accel_filter.apply(a);

            Vector3f a2 = a * _accel_scale;
            _notify_new_accel_sensor_rate_sample(_accel_instance, a2);

            _accum.accel_count++;

            if (_accum.accel_count % _accel_fifo_downsample_rate == 0) {
                _accum.accel *= _fifo_accel_scale;
                _rotate_and_correct_accel(_accel_instance, _accum.accel);
                _notify_new_accel_raw_sample(_accel_instance, _accum.accel, 0, false);
                _accum.accel.zero();
                _accum.accel_count = 0;
                // we assume that the gyro rate is always >= and a multiple of the accel rate
                _accum.gyro_count = 0;
            }
        }

        _accum.gyro_count++;

        Vector3f g(int16_val(data, 4),
                   int16_val(data, 3),
                   -int16_val(data, 5));

        Vector3f g2 = g * GYRO_SCALE;
        _notify_new_gyro_sensor_rate_sample(_gyro_instance, g2);

        _accum.gyro += g;

        if (_accum.gyro_count % _gyro_fifo_downsample_rate == 0) {
            _accum.gyro *= _fifo_gyro_scale;
            _rotate_and_correct_gyro(_gyro_instance, _accum.gyro);
            _notify_new_gyro_raw_sample(_gyro_instance, _accum.gyro);
            _accum.gyro.zero();
        }
    }

    if (clipped) {
        increment_clip_count(_accel_instance);
    }

    if (ret) {
        float temp = (static_cast<float>(tsum)/n_samples)*temp_sensitivity + temp_zero;
        _temp_filtered = _temp_filter.apply(temp);
    }
    
    return ret;
}

void AP_InertialSensor_Invensensev2::_read_fifo()
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t *rx = _fifo_buffer;
    bool need_reset = false;

    if (!_block_read(INV2REG_FIFO_COUNTH, rx, 2)) {
        goto check_registers;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / INV2_SAMPLE_SIZE;

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        goto check_registers;
    }

    /*
      testing has shown that if we have more than 32 samples in the
      FIFO then some of those samples will be corrupt. It always is
      the ones at the end of the FIFO, so clear those with a reset
      once we've read the first 24. Reading 24 gives us the normal
      number of samples for fast sampling at 400Hz

      On I2C with the much lower clock rates we need a lower threshold
      or we may never catch up
     */
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C) {
        if (n_samples > 4) {
            need_reset = true;
            n_samples = 4;
        }
    } else {
        if (n_samples > 32) {
            need_reset = true;
            n_samples = 24;
        }
    }
    while (n_samples > 0) {
        uint8_t n = MIN(n_samples, INV2_FIFO_BUFFER_LEN);
        if (!_dev->set_chip_select(true)) {
            if (!_block_read(INV2REG_FIFO_R_W, rx, n * INV2_SAMPLE_SIZE)) {
                goto check_registers;
            }
        } else {
            // this ensures we keep things nicely setup for DMA
            uint8_t reg = GET_REG(INV2REG_FIFO_R_W) | 0x80;
            if (!_dev->transfer_bank(GET_BANK(INV2REG_FIFO_R_W), &reg, 1, nullptr, 0)) {
                _dev->set_chip_select(false);
                goto check_registers;
            }
            memset(rx, 0, n * INV2_SAMPLE_SIZE);
            if (!_dev->transfer(rx, n * INV2_SAMPLE_SIZE, rx, n * INV2_SAMPLE_SIZE)) {
                if (!hal.scheduler->in_expected_delay()) {
                    debug("INV2: error in fifo read %u bytes\n", n * INV2_SAMPLE_SIZE);
                }
                _dev->set_chip_select(false);
                goto check_registers;
            }
            _dev->set_chip_select(false);
        }

        if (_fast_sampling) {
            if (!_accumulate_sensor_rate_sampling(rx, n)) {
                if (!hal.scheduler->in_expected_delay()) {
                    debug("IMU[%u] stop at %u of %u", _accel_instance, n_samples, bytes_read/INV2_SAMPLE_SIZE);
                }
                break;
            }
        } else {
            if (!_accumulate(rx, n)) {
                break;
            }
        }
        n_samples -= n;
    }

    if (need_reset) {
        //debug("fifo reset n_samples %u", bytes_read/INV2_SAMPLE_SIZE);
        _fifo_reset();
    }
    
check_registers:
    // check next register value for correctness
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!_dev->check_next_register()) {
        _inc_gyro_error_count(_gyro_instance);
        _inc_accel_error_count(_accel_instance);
    }
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

/*
  fetch temperature in order to detect FIFO sync errors
*/
bool AP_InertialSensor_Invensensev2::_check_raw_temp(int16_t t2)
{
    if (abs(t2 - _raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (_block_read(INV2REG_TEMP_OUT_H, trx, 2)) {
        _raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - _raw_temp) < 400);
}

bool AP_InertialSensor_Invensensev2::_block_read(uint16_t reg, uint8_t *buf,
                                            uint32_t size)
{
    return _dev->read_bank_registers(GET_BANK(reg), GET_REG(reg), buf, size);
}

uint8_t AP_InertialSensor_Invensensev2::_register_read(uint16_t reg)
{
    uint8_t val = 0;
    _dev->read_bank_registers(GET_BANK(reg), GET_REG(reg), &val, 1);
    return val;
}

void AP_InertialSensor_Invensensev2::_register_write(uint16_t reg, uint8_t val, bool checked)
{
    (void)checked;
    _dev->write_bank_register(GET_BANK(reg), GET_REG(reg), val, checked);
}

bool AP_InertialSensor_Invensensev2::_select_bank(uint8_t bank)
{
    if (_current_bank != bank) {
        if (!_dev->write_register(INV2REG_BANK_SEL, bank << 4, true)) {
            return false;
        }
        _current_bank = bank;
    }
    return true;
}

/*
  set the DLPF filter frequency and Gyro Accel Scaling. Assumes caller has taken semaphore
 */
void AP_InertialSensor_Invensensev2::_set_filter_and_scaling(void)
{
    uint8_t gyro_config = (_inv2_type == Invensensev2_ICM20649)?BITS_GYRO_FS_2000DPS_20649 : BITS_GYRO_FS_2000DPS;
    uint8_t accel_config = (_inv2_type == Invensensev2_ICM20649)?BITS_ACCEL_FS_30G_20649:BITS_ACCEL_FS_16G;

    // assume 1.125kHz sampling to start
    _gyro_fifo_downsample_rate = _accel_fifo_downsample_rate = 1;
    _gyro_backend_rate_hz = _accel_backend_rate_hz =  1125;
    
    if (enable_fast_sampling(_accel_instance)) {
        _fast_sampling = _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;
        if (_fast_sampling) {
            // constrain the gyro rate to be at least the loop rate
            uint8_t loop_limit = 1;
            if (get_loop_rate_hz() > 1125) {
                loop_limit = 2;
            }
            if (get_loop_rate_hz() > 2250) {
                loop_limit = 4;
            }
            // constrain the gyro rate to be a 2^N multiple
            uint8_t fast_sampling_rate = constrain_int16(get_fast_sampling_rate(), loop_limit, 8);

            // calculate rate we will be giving gyro samples to the backend
            _gyro_fifo_downsample_rate = 8 / fast_sampling_rate;
            _gyro_backend_rate_hz *= fast_sampling_rate;

            // calculate rate we will be giving accel samples to the backend
            _accel_fifo_downsample_rate = MAX(4 / fast_sampling_rate, 1);
            _accel_backend_rate_hz *= MIN(fast_sampling_rate, 4);

            // for logging purposes set the oversamping rate
            _set_accel_oversampling(_accel_instance, _accel_fifo_downsample_rate);
            _set_gyro_oversampling(_gyro_instance, _gyro_fifo_downsample_rate);

            _set_accel_sensor_rate_sampling_enabled(_accel_instance, true);
            _set_gyro_sensor_rate_sampling_enabled(_gyro_instance, true);

            /* set divider for internal sample rate to 0x1F when fast
             sampling enabled. This reduces the impact of the slave
             sensor on the sample rate.
             */
            _register_write(INV2REG_I2C_SLV4_CTRL, 0x1F);
        }
    }
    
    if (_fast_sampling) {
        // this gives us 9kHz sampling on gyros
        gyro_config |= BIT_GYRO_NODLPF_9KHZ;
        accel_config |= BIT_ACCEL_NODLPF_4_5KHZ;
    } else {
        // limit to 1.125kHz if not on SPI
        gyro_config |= BIT_GYRO_DLPF_ENABLE | (GYRO_DLPF_CFG_188HZ << GYRO_DLPF_CFG_SHIFT);
        accel_config |= BIT_ACCEL_DLPF_ENABLE | (ACCEL_DLPF_CFG_265HZ << ACCEL_DLPF_CFG_SHIFT);
    }
    _register_write(INV2REG_GYRO_CONFIG_1, gyro_config, true);
    _register_write(INV2REG_ACCEL_CONFIG, accel_config, true);
    _register_write(INV2REG_FIFO_MODE, 0xF, true);
}

/*
  check whoami for sensor type
 */
bool AP_InertialSensor_Invensensev2::_check_whoami(void)
{
    uint8_t whoami = _register_read(INV2REG_WHO_AM_I);
    switch (whoami) {
    case INV2_WHOAMI_ICM20648:
        _inv2_type = Invensensev2_ICM20648;
        return true;
    case INV2_WHOAMI_ICM20948:
        _inv2_type = Invensensev2_ICM20948;
        return true;
    case INV2_WHOAMI_ICM20649:
        _inv2_type = Invensensev2_ICM20649;
        return true;
    }
    // not a value WHOAMI result
    return false;
}

bool AP_InertialSensor_Invensensev2::_hardware_init(void)
{
    _dev->get_semaphore()->take_blocking();

    // disabled setup of checked registers as it can't cope with bank switching
    _dev->setup_checked_registers(7, _dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C?200:20);
    _dev->setup_bankselect_callback(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensensev2::_select_bank, bool, uint8_t));

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!_check_whoami()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        _last_stat_user_ctrl = _register_read(INV2REG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            _register_write(INV2REG_USER_CTRL, _last_stat_user_ctrl);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(INV2REG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
        if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
            _register_write(INV2REG_USER_CTRL, _last_stat_user_ctrl);
        }

        // Wake up device and select Auto clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(INV2REG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_AUTO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(INV2REG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_AUTO) {
            break;
        }

        hal.scheduler->delay(10);
        if (_data_ready()) {
            break;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (tries == 5) {
        hal.console->printf("Failed to boot Invensense 5 times\n");
        _dev->get_semaphore()->give();
        return false;
    }

    if (_inv2_type == Invensensev2_ICM20649) {
        _clip_limit = 29.5f * GRAVITY_MSS;
    }
    _dev->get_semaphore()->give();
    
    return true;
}

AP_Invensensev2_AuxiliaryBusSlave::AP_Invensensev2_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr,
                                                         uint8_t instance)
    : AuxiliaryBusSlave(bus, addr, instance)
    , _inv2_addr(INV2REG_I2C_SLV0_ADDR + _instance * 4)
    , _inv2_reg(_inv2_addr + 1)
    , _inv2_ctrl(_inv2_addr + 2)
    , _inv2_do(_inv2_addr + 3)
{
}

int AP_Invensensev2_AuxiliaryBusSlave::_set_passthrough(uint8_t reg, uint8_t size,
                                                  uint8_t *out)
{
    auto &backend = AP_InertialSensor_Invensensev2::from(_bus.get_backend());
    uint8_t addr;

    /* Ensure the slave read/write is disabled before changing the registers */
    backend._register_write(_inv2_ctrl, 0);

    if (out) {
        backend._register_write(_inv2_do, *out);
        addr = _addr;
    } else {
        addr = _addr | BIT_READ_FLAG;
    }

    backend._register_write(_inv2_addr, addr);
    backend._register_write(_inv2_reg, reg);
    backend._register_write(_inv2_ctrl, BIT_I2C_SLVX_EN | size);
    return 0;
}

int AP_Invensensev2_AuxiliaryBusSlave::passthrough_read(uint8_t reg, uint8_t *buf,
                                                   uint8_t size)
{
    assert(buf);

    if (_registered) {
        hal.console->printf("Error: can't passthrough when slave is already configured\n");
        return -1;
    }

    int r = _set_passthrough(reg, size);
    if (r < 0) {
        return r;
    }

    /* wait the value to be read from the slave and read it back */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_Invensensev2::from(_bus.get_backend());
    if (!backend._block_read(INV2REG_EXT_SLV_SENS_DATA_00 + _ext_sens_data, buf, size)) {
        return -1;
    }

    /* disable new reads */
    backend._register_write(_inv2_ctrl, 0);

    return size;
}

int AP_Invensensev2_AuxiliaryBusSlave::passthrough_write(uint8_t reg, uint8_t val)
{
    if (_registered) {
        hal.console->printf("Error: can't passthrough when slave is already configured\n");
        return -1;
    }

    int r = _set_passthrough(reg, 1, &val);
    if (r < 0) {
        return r;
    }

    /* wait the value to be written to the slave */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_Invensensev2::from(_bus.get_backend());

    /* disable new writes */
    backend._register_write(_inv2_ctrl, 0);

    return 1;
}

int AP_Invensensev2_AuxiliaryBusSlave::read(uint8_t *buf)
{
    if (!_registered) {
        hal.console->printf("Error: can't read before configuring slave\n");
        return -1;
    }

    auto &backend = AP_InertialSensor_Invensensev2::from(_bus.get_backend());
    if (!backend._block_read(INV2REG_EXT_SLV_SENS_DATA_00 + _ext_sens_data, buf, _sample_size)) {
        return -1;
    }

    return _sample_size;
}

/* Invensense provides up to 5 slave devices, but the 5th is way too different to
 * configure and is seldom used */
AP_Invensensev2_AuxiliaryBus::AP_Invensensev2_AuxiliaryBus(AP_InertialSensor_Invensensev2 &backend, uint32_t devid)
    : AuxiliaryBus(backend, 4, devid)
{
}

AP_HAL::Semaphore *AP_Invensensev2_AuxiliaryBus::get_semaphore()
{
    return static_cast<AP_InertialSensor_Invensensev2&>(_ins_backend)._dev->get_semaphore();
}

AuxiliaryBusSlave *AP_Invensensev2_AuxiliaryBus::_instantiate_slave(uint8_t addr, uint8_t instance)
{
    /* Enable slaves on Invensense if this is the first time */
    if (_ext_sens_data == 0) {
        _configure_slaves();
    }

    return new AP_Invensensev2_AuxiliaryBusSlave(*this, addr, instance);
}

void AP_Invensensev2_AuxiliaryBus::_configure_slaves()
{
    auto &backend = AP_InertialSensor_Invensensev2::from(_ins_backend);

    backend._dev->get_semaphore()->take_blocking();

    /* Enable the I2C master to slaves on the auxiliary I2C bus*/
    if (!(backend._last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN)) {
        backend._last_stat_user_ctrl |= BIT_USER_CTRL_I2C_MST_EN;
        backend._register_write(INV2REG_USER_CTRL, backend._last_stat_user_ctrl);
    }

    /* stop condition between reads; clock at 400kHz */
    backend._register_write(INV2REG_I2C_MST_CTRL,
                            BIT_I2C_MST_P_NSR | BIT_I2C_MST_CLK_400KHZ);

    /* Hard-code divider for internal sample rate, 1.125 kHz, resulting in a
     * sample rate of ~100Hz */
    backend._register_write(INV2REG_I2C_SLV4_CTRL, 10);

    /* All slaves are subject to the sample rate */
    backend._register_write(INV2REG_I2C_MST_DELAY_CTRL,
                            BIT_I2C_SLV0_DLY_EN | BIT_I2C_SLV1_DLY_EN |
                            BIT_I2C_SLV2_DLY_EN | BIT_I2C_SLV3_DLY_EN);

    backend._dev->get_semaphore()->give();
}

int AP_Invensensev2_AuxiliaryBus::_configure_periodic_read(AuxiliaryBusSlave *slave,
                                                     uint8_t reg, uint8_t size)
{
    if (_ext_sens_data + size > MAX_EXT_SENS_DATA) {
        return -1;
    }

    AP_Invensensev2_AuxiliaryBusSlave *inv2_slave =
        static_cast<AP_Invensensev2_AuxiliaryBusSlave*>(slave);
    inv2_slave->_set_passthrough(reg, size);
    inv2_slave->_ext_sens_data = _ext_sens_data;
    _ext_sens_data += size;

    return 0;
}

AP_HAL::Device::PeriodicHandle AP_Invensensev2_AuxiliaryBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    auto &backend = AP_InertialSensor_Invensensev2::from(_ins_backend);
    return backend._dev->register_periodic_callback(period_usec, cb);
}
