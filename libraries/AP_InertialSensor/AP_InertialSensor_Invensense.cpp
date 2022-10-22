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
  driver for all supported Invensense IMUs, including
  MPU6000, MPU9250,  ICM20608, ICM20602, ICM20601, ICM20789, ICM20689
 */
#define AP_INLINE_VECTOR_OPS

#include <assert.h>
#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_InertialSensor_Invensense.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#define INVENSENSE_DRDY_PIN BBB_P8_14
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#define INVENSENSE_EXT_SYNC_ENABLE 1
#endif
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// hal.console can be accessed from bus threads on ChibiOS
#define debug(fmt, args ...)  do {hal.console->printf("MPU: " fmt "\n", ## args); } while(0)
#else
#define debug(fmt, args ...)  do {printf("MPU: " fmt "\n", ## args); } while(0)
#endif

/*
  EXT_SYNC allows for frame synchronisation with an external device
  such as a camera. When enabled the LSB of AccelZ holds the FSYNC bit
 */
#ifndef INVENSENSE_EXT_SYNC_ENABLE
#define INVENSENSE_EXT_SYNC_ENABLE 0
#endif

// code to debug unexpected register changes
#define INVENSENSE_DEBUG_REG_CHANGE 0

#if INVENSENSE_DEBUG_REG_CHANGE
#include <GCS_MAVLink/GCS.h>
#endif

#include "AP_InertialSensor_Invensense_registers.h"

#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_BUFFER_LEN 8

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_Invensense::AP_InertialSensor_Invensense(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                           enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _temp_filter(1000, 1)
    , _rotation(rotation)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_Invensense::~AP_InertialSensor_Invensense()
{
    if (_fifo_buffer != nullptr) {
        hal.util->free_type(_fifo_buffer, MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
    delete _auxiliary_bus;
}

AP_InertialSensor_Backend *AP_InertialSensor_Invensense::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Invensense *sensor =
        new AP_InertialSensor_Invensense(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    if (sensor->_mpu_type == Invensense_MPU9250) {
        sensor->_id = HAL_INS_MPU9250_I2C;
    } else {
        sensor->_id = HAL_INS_MPU60XX_I2C;
    }

    return sensor;
}


AP_InertialSensor_Backend *AP_InertialSensor_Invensense::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Invensense *sensor;

    dev->set_read_flag(0x80);

    sensor = new AP_InertialSensor_Invensense(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    if (sensor->_mpu_type == Invensense_MPU9250) {
        sensor->_id = HAL_INS_MPU9250_SPI;
    } else if (sensor->_mpu_type == Invensense_MPU6500) {
        sensor->_id = HAL_INS_MPU6500;
    } else {
        sensor->_id = HAL_INS_MPU60XX_SPI;
    }

    return sensor;
}

bool AP_InertialSensor_Invensense::_init()
{
#ifdef INVENSENSE_DRDY_PIN
    _drdy_pin = hal.gpio->channel(INVENSENSE_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    bool success = _hardware_init();

    return success;
}

void AP_InertialSensor_Invensense::_fifo_reset(bool log_error)
{
    uint32_t now = AP_HAL::millis();
    if (log_error &&
        !hal.scheduler->in_expected_delay() &&
        now - last_reset_ms < 10000) {
        reset_count++;
        if (reset_count == 10) {
            // 10 resets, each happening within 10s, triggers an internal error
            INTERNAL_ERROR(AP_InternalError::error_t::imu_reset);
            reset_count = 0;
        }
    } else if (log_error &&
        !hal.scheduler->in_expected_delay() &&
        now - last_reset_ms > 10000) {
        //if last reset was more than 10s ago consider this the first reset
        reset_count = 1;
    }
    last_reset_ms = now;

    uint8_t user_ctrl = _last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_RESET | BIT_USER_CTRL_FIFO_EN);

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    _register_write(MPUREG_FIFO_EN, 0);
    _register_write(MPUREG_USER_CTRL, user_ctrl);
    _register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_RESET);
    _register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    _register_write(MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN, true);
    hal.scheduler->delay_microseconds(1);
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

void AP_InertialSensor_Invensense::_fast_fifo_reset()
{
    fast_reset_count++;
    _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl | BIT_USER_CTRL_FIFO_RESET);

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}


bool AP_InertialSensor_Invensense::_has_auxiliary_bus()
{
    return _dev->bus_type() != AP_HAL::Device::BUS_TYPE_I2C;
}

void AP_InertialSensor_Invensense::start()
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    _register_write(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

    // always use FIFO
    _fifo_reset(false);

    // grab the used instances
    enum DevTypes gdev, adev;
    switch (_mpu_type) {
    case Invensense_MPU9250:
        gdev = DEVTYPE_GYR_MPU9250;
        adev = DEVTYPE_ACC_MPU9250;
        _enable_offset_checking = true;
        break;
    case Invensense_ICM20602:
        gdev = DEVTYPE_INS_ICM20602;
        adev = DEVTYPE_INS_ICM20602;
        // ICM20602 has a bug where sometimes the data gets a huge offset
        // this seems to be fixed by doing a quick FIFO reset via USR_CTRL
        // reg
        _enable_fast_fifo_reset = true;
        _enable_offset_checking = true;
        break;
    case Invensense_ICM20601:
        gdev = DEVTYPE_INS_ICM20601;
        adev = DEVTYPE_INS_ICM20601;
        _enable_offset_checking = true;
        break;
    case Invensense_ICM20608:
        // unfortunately we don't have a separate ID for 20608, and we
        // can't change this without breaking existing calibrations
        gdev = DEVTYPE_GYR_MPU6000;
        adev = DEVTYPE_ACC_MPU6000;
        _enable_offset_checking = true;
        break;
    case Invensense_ICM20789:
        gdev = DEVTYPE_INS_ICM20789;
        adev = DEVTYPE_INS_ICM20789;
        _enable_offset_checking = true;
        break;
    case Invensense_ICM20689:
        gdev = DEVTYPE_INS_ICM20689;
        adev = DEVTYPE_INS_ICM20689;
        _enable_offset_checking = true;
        break;
    case Invensense_MPU6000:
    case Invensense_MPU6500:
    default:
        gdev = DEVTYPE_GYR_MPU6000;
        adev = DEVTYPE_ACC_MPU6000;
        break;
    }

    /*
      setup temperature sensitivity and offset. This varies
      considerably between parts
     */
    switch (_mpu_type) {
    case Invensense_MPU9250:
        temp_zero = 21.0f;
        temp_sensitivity = 1.0f/340;
        break;

    case Invensense_MPU6000:
    case Invensense_MPU6500:
        temp_zero = 36.53f;
        temp_sensitivity = 1.0f/340;
        break;

    case Invensense_ICM20608:
    case Invensense_ICM20602:
    case Invensense_ICM20601:
        temp_zero = 25.0f;
        temp_sensitivity = 1.0f/326.8f;
        break;

    case Invensense_ICM20789:
        temp_zero = 25.0f;
        temp_sensitivity = 0.003f;
        break;
    case Invensense_ICM20689:
        temp_zero = 25.0f;
        temp_sensitivity = 0.003f;
        break;
    }

    if (!_imu.register_gyro(_gyro_instance, 1000, _dev->get_bus_id_devtype(gdev)) ||
        !_imu.register_accel(_accel_instance, 1000, _dev->get_bus_id_devtype(adev))) {
        return;
    }

    // setup ODR and on-sensor filtering
    _set_filter_register();

    // update backend sample rate
    _set_accel_raw_sample_rate(_accel_instance, _accel_backend_rate_hz);
    _set_gyro_raw_sample_rate(_gyro_instance, _gyro_backend_rate_hz);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
    _set_raw_sample_accel_multiplier(_accel_instance, multiplier_accel);

    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    _register_write(MPUREG_SMPLRT_DIV, 0, true);
    hal.scheduler->delay(1);

    // Gyro scale 2000º/s
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS, true);
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    uint8_t product_id = _register_read(MPUREG_PRODUCT_ID);

    if (_mpu_type == Invensense_MPU6000 &&
        ((product_id == MPU6000ES_REV_C4) ||
         (product_id == MPU6000ES_REV_C5) ||
         (product_id == MPU6000_REV_C4)   ||
         (product_id == MPU6000_REV_C5))) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3, true);
        _accel_scale = GRAVITY_MSS / 4096.f;
        _gyro_scale = (radians(1) / 16.4f);
    } else if (_mpu_type == Invensense_ICM20601) {
        // Accel scale 32g (4096 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,1<<3, true);
        _accel_scale = GRAVITY_MSS / 4096.f;
        _gyro_scale = (radians(1) / 8.2f);
        _clip_limit = 29.5f * GRAVITY_MSS;
    } else {
        // Accel scale 16g (2048 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,3<<3, true);
        _accel_scale = GRAVITY_MSS / 2048.f;
        _gyro_scale = (radians(1) / 16.4f);
    }
    hal.scheduler->delay(1);

    if (_mpu_type == Invensense_ICM20608 ||
        _mpu_type == Invensense_ICM20602 ||
        _mpu_type == Invensense_ICM20601) {
        // this avoids a sensor bug, see description above
        _register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
    }
    
    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt. We don't do this for the 20789 as
    // that sensor has already setup the appropriate config inside the
    // baro driver.
    if (_mpu_type != Invensense_ICM20789) {    
        uint8_t v = _register_read(MPUREG_INT_PIN_CFG) | BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN;
        v &= BIT_BYPASS_EN;
        _register_write(MPUREG_INT_PIN_CFG, v);
    }

    if (_enable_offset_checking) {
        /*
          there is a bug in at least the ICM-20602 where the
          MPUREG_ACC_OFF_Y_H changes at runtime, which adds an offset
          on the Y accelerometer. To prevent this we read the factory
          cal values of the sensor at startup and write them back as
          checked register values. Then we rely on the register
          checking code to detect the change and fix it
         */
        uint8_t regs[] = { MPUREG_ACC_OFF_X_H, MPUREG_ACC_OFF_X_L,
                           MPUREG_ACC_OFF_Y_H, MPUREG_ACC_OFF_Y_L,
                           MPUREG_ACC_OFF_Z_H, MPUREG_ACC_OFF_Z_L };
        for (uint8_t i=0; i<ARRAY_SIZE(regs); i++) {
            _register_write(regs[i], _register_read(regs[i]), true);
        }
    }


    if (_mpu_type == Invensense_ICM20602) {
        /*
          save y offset high separately for ICM20602 to quickly
          recover from a change in this register. The ICM20602 has a
          bug where every few hours it can change the factory Y offset
          register, which leads to a sudden change in Y accelerometer
          output
         */
        _saved_y_ofs_high = _register_read(MPUREG_ACC_OFF_Y_H);
    }

    // now that we have initialised, we set the bus speed to high
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    // setup scale factors for fifo data after downsampling
    _fifo_accel_scale = _accel_scale / _accel_fifo_downsample_rate;
    _fifo_gyro_scale = _gyro_scale / _gyro_fifo_downsample_rate;
    
    // allocate fifo buffer
    _fifo_buffer = (uint8_t *)hal.util->malloc_type(MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    if (_fifo_buffer == nullptr) {
        AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
    }

    // start the timer process to read samples, using the fastest rate avilable
    _dev->register_periodic_callback(1000000UL / _gyro_backend_rate_hz, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensense::_poll_data, void));
}

// get a startup banner to output to the GCS
bool AP_InertialSensor_Invensense::get_output_banner(char* banner, uint8_t banner_len) {
    if (_fast_sampling) {
        snprintf(banner, banner_len, "IMU%u: fast sampling enabled %.1fkHz/%.1fkHz",
            _gyro_instance, _gyro_backend_rate_hz * _gyro_fifo_downsample_rate * 0.001, _gyro_backend_rate_hz * 0.001);
        return true;
    }
    return false;
}


/*
  publish any pending data
 */
bool AP_InertialSensor_Invensense::update() /* front end */
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    _publish_temperature(_accel_instance, _temp_filtered);

    if (fast_reset_count) {
        // check if we have reported in the last 1 seconds or
        // fast_reset_count changed
#if HAL_GCS_ENABLED && BOARD_FLASH_SIZE > 1024
        if (AP_HAL::millis() - last_fast_reset_count_report_ms > 1000) {
            last_fast_reset_count_report_ms = AP_HAL::millis();
            char param_name[sizeof("IMUx_RST")];
            if (_gyro_instance <= 9) {
                snprintf(param_name, sizeof(param_name), "IMU%u_RST", _gyro_instance);
            } else {
                snprintf(param_name, sizeof(param_name), "IMUx_RST");
            }
            gcs().send_named_float(param_name, fast_reset_count);
        }
#endif
#if HAL_LOGGING_ENABLED
        if (last_fast_reset_count != fast_reset_count) {
            AP::logger().Write_MessageF("IMU%u fast fifo reset %u", _gyro_instance, fast_reset_count);
        }
#endif
    }

    return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_Invensense::accumulate()
{
    // nothing to do
}

AuxiliaryBus *AP_InertialSensor_Invensense::get_auxiliary_bus()
{
    if (_auxiliary_bus) {
        return _auxiliary_bus;
    }

    if (_has_auxiliary_bus()) {
        _auxiliary_bus = new AP_Invensense_AuxiliaryBus(*this, _dev->get_bus_id());
    }

    return _auxiliary_bus;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_Invensense::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }
    uint8_t status = _register_read(MPUREG_INT_STATUS);
    return (status & BIT_RAW_RDY_INT) != 0;
}

/*
 * Timer process to poll for new data from the Invensense. Called from bus thread with semaphore held
 */
void AP_InertialSensor_Invensense::_poll_data()
{
    _read_fifo();

#if INVENSENSE_DEBUG_REG_CHANGE
    _check_register_change();
#endif // INVENSENSE_DEBUG_REG_CHANGE
}

#if INVENSENSE_DEBUG_REG_CHANGE
/*
  catch unexpected register changes on an IMU. This was used to
  find the bug in the ICM-20602 that causes the Y accel offset
  trim register to change value in flight
*/
void AP_InertialSensor_Invensense::_check_register_change(void)
{
    if (_mpu_type != Invensense_ICM20602) {
        return;
    }
    static uint16_t counter;
    if (++counter < 100) {
        return;
    }
    counter = 0;
    static bool reg_init;
    static uint8_t reg_value[0x80];
    static uint8_t next_reg;
    if (!reg_init) {
        reg_init = true;
        for (uint8_t i=0; i<ARRAY_SIZE(reg_value); i++) {
            reg_value[i] = _register_read(i);
        }
    }
    bool skip = false;
    if ((next_reg >= MPUREG_ACCEL_XOUT_H && next_reg <= MPUREG_GYRO_ZOUT_L) ||
        next_reg == MPUREG_MEM_R_W || next_reg == MPUREG_FIFO_R_W ||
        next_reg == MPUREG_INT_STATUS ||
        next_reg == MPUREG_FIFO_COUNTH || next_reg == MPUREG_FIFO_COUNTL) {
        skip = true;
    }
    if (!skip) {
        uint8_t v = _register_read(next_reg);
        if (v != reg_value[next_reg]) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "change[%02x] 0x%02x -> 0x%02x",
                          next_reg, reg_value[next_reg], v);
            reg_value[next_reg] = v;
        }
    }
    next_reg = (next_reg+1) % ARRAY_SIZE(reg_value);
}
#endif // INVENSENSE_DEBUG_REG_CHANGE

bool AP_InertialSensor_Invensense::_accumulate(uint8_t *samples, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;
        Vector3f accel, gyro;
        bool fsync_set = false;

#if INVENSENSE_EXT_SYNC_ENABLE
        fsync_set = (int16_val(data, 2) & 1U) != 0;
#endif
        
        accel = Vector3f(int16_val(data, 1),
                         int16_val(data, 0),
                         -int16_val(data, 2));
        accel *= _accel_scale;

        int16_t t2 = int16_val(data, 3);
        if (!_check_raw_temp(t2)) {
            if (_enable_fast_fifo_reset) {
                _fast_fifo_reset();
                return false;
            } else {
                if (!hal.scheduler->in_expected_delay()) {
                    debug("temp reset IMU[%u] %d %d", _accel_instance, _raw_temp, t2);
                }
                _fifo_reset(true);
                return false;
            }
        }
        float temp = t2 * temp_sensitivity + temp_zero;
        
        gyro = Vector3f(int16_val(data, 5),
                        int16_val(data, 4),
                        -int16_val(data, 6));
        gyro *= _gyro_scale;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel, 0, fsync_set);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);

        _temp_filtered = _temp_filter.apply(temp);
    }
    return true;
}

/*
  when doing fast sampling the sensor gives us 8k samples/second. Every 2nd accel sample is a duplicate.

  To filter this we first apply a 1p low pass filter at 188Hz, then we
  average over 8 samples to bring the data rate down to 1kHz. This
  gives very good aliasing rejection at frequencies well above what
  can be handled with 1kHz sample rates.
 */
bool AP_InertialSensor_Invensense::_accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    const int32_t unscaled_clip_limit = _clip_limit / _accel_scale;
    bool clipped = false;
    bool ret = true;
    
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;

        // use temperature to detect FIFO corruption
        int16_t t2 = int16_val(data, 3);
        if (!_check_raw_temp(t2)) {
            if (_enable_fast_fifo_reset) {
                _fast_fifo_reset();
                ret = false;
            } else {
                if (!hal.scheduler->in_expected_delay()) {
                    debug("temp reset IMU[%u] %d %d", _accel_instance, _raw_temp, t2);
                }
                _fifo_reset(true);
                ret = false;
            }
            break;
        }
        tsum += t2;

        if (_accum.gyro_count % _gyro_to_accel_sample_ratio == 0) {
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

        Vector3f g(int16_val(data, 5),
                   int16_val(data, 4),
                   -int16_val(data, 6));

        Vector3f g2 = g * _gyro_scale;
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

void AP_InertialSensor_Invensense::_read_fifo()
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t *rx = _fifo_buffer;
    bool need_reset = false;

    if (!_block_read(MPUREG_FIFO_COUNTH, rx, 2)) {
        goto check_registers;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / MPU_SAMPLE_SIZE;

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
        uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);
        if (!_dev->set_chip_select(true)) {
            if (!_block_read(MPUREG_FIFO_R_W, rx, n * MPU_SAMPLE_SIZE)) {
                goto check_registers;
            }
        } else {
            // this ensures we keep things nicely setup for DMA
            uint8_t reg = MPUREG_FIFO_R_W | 0x80;
            if (!_dev->transfer(&reg, 1, nullptr, 0)) {
                _dev->set_chip_select(false);
                goto check_registers;
            }
            memset(rx, 0, n * MPU_SAMPLE_SIZE);
            if (!_dev->transfer(rx, n * MPU_SAMPLE_SIZE, rx, n * MPU_SAMPLE_SIZE)) {
                if (!hal.scheduler->in_expected_delay()) {
                    debug("MPU60x0: error in fifo read %u bytes\n", n * MPU_SAMPLE_SIZE);
                }
                _dev->set_chip_select(false);
                goto check_registers;
            }
            _dev->set_chip_select(false);
        }

        if (_fast_sampling) {
            if (!_accumulate_sensor_rate_sampling(rx, n)) {
                if (!hal.scheduler->in_expected_delay() && !_enable_fast_fifo_reset) {
                    debug("IMU[%u] stop at %u of %u", _accel_instance, n_samples, bytes_read/MPU_SAMPLE_SIZE);
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
        //debug("fifo reset n_samples %u", bytes_read/MPU_SAMPLE_SIZE);
        _fifo_reset(false);
    }
    
check_registers:
    // check next register value for correctness

    if (_mpu_type == Invensense_ICM20602) {
        const uint8_t y_ofs = _register_read(MPUREG_ACC_OFF_Y_H);
        if (y_ofs != _saved_y_ofs_high) {
            /*
              we check and restore the ICM20602 Y offset high register
              on every update. We don't mark the IMU unhealthy when we
              do this. This is a workaround for a bug in the ICM-20602
              where this register can change in flight. We log these
              events to help with log analysis, but don't shout at the
              GCS to prevent possible flood
            */
#if HAL_LOGGING_ENABLED
            AP::logger().Write_MessageF("ICM20602 yofs fix: %x %x", y_ofs, _saved_y_ofs_high);
#endif
            _register_write(MPUREG_ACC_OFF_Y_H, _saved_y_ofs_high);
        }
    }


    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_gyro_error_count(_gyro_instance);
        _inc_accel_error_count(_accel_instance);
    }
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

/*
  fetch temperature in order to detect FIFO sync errors
*/
bool AP_InertialSensor_Invensense::_check_raw_temp(int16_t t2)
{
    if (abs(t2 - _raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (_block_read(MPUREG_TEMP_OUT_H, trx, 2)) {
        _raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - _raw_temp) < 800);
}

bool AP_InertialSensor_Invensense::_block_read(uint8_t reg, uint8_t *buf,
                                            uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_Invensense::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_Invensense::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_Invensense::_set_filter_register(void)
{
    uint8_t config;

#if INVENSENSE_EXT_SYNC_ENABLE
    // add in EXT_SYNC bit if enabled
    config = (MPUREG_CONFIG_EXT_SYNC_AZ << MPUREG_CONFIG_EXT_SYNC_SHIFT);
#else
    config = 0;
#endif

    // assume 1kHz sampling to start
    _gyro_fifo_downsample_rate = _accel_fifo_downsample_rate = 1;
    _gyro_to_accel_sample_ratio = 2;
    _gyro_backend_rate_hz = _accel_backend_rate_hz =  1000;
    
    if (enable_fast_sampling(_accel_instance)) {
        _fast_sampling = _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI;
        if (_fast_sampling) {
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

            // calculate rate we will be giving gyro samples to the backend
            _gyro_fifo_downsample_rate = 8 / fast_sampling_rate;
            _gyro_backend_rate_hz *= fast_sampling_rate;

            // calculate rate we will be giving accel samples to the backend
            if (_mpu_type >= Invensense_MPU9250) {
                _accel_fifo_downsample_rate = MAX(4 / fast_sampling_rate, 1);
                _accel_backend_rate_hz *= MIN(fast_sampling_rate, 4);
            } else {
                _gyro_to_accel_sample_ratio = 8;
                _accel_fifo_downsample_rate = 1;
                _accum.accel_filter.set_cutoff_frequency(1000, 188);
            }

            // for logging purposes set the oversamping rate
            _set_accel_oversampling(_accel_instance, _accel_fifo_downsample_rate);
            _set_gyro_oversampling(_gyro_instance, _gyro_fifo_downsample_rate);

            _set_accel_sensor_rate_sampling_enabled(_accel_instance, true);
            _set_gyro_sensor_rate_sampling_enabled(_gyro_instance, true);

            /* set divider for internal sample rate to 0x1F when fast
             sampling enabled. This reduces the impact of the slave
             sensor on the sample rate. It ends up with around 75Hz
             slave rate, and reduces the impact on the gyro and accel
             sample rate, ending up with around 7760Hz gyro rate and
             3880Hz accel rate
             */
            _register_write(MPUREG_I2C_SLV4_CTRL, 0x1F);
        }
    }
    
    if (_fast_sampling) {
        // this gives us 8kHz sampling on gyros and 4kHz on accels
        config |= BITS_DLPF_CFG_256HZ_NOLPF2;
    } else {
        // limit to 1kHz if not on SPI
        config |= BITS_DLPF_CFG_188HZ;
    }

    config |= MPUREG_CONFIG_FIFO_MODE_STOP;
    _register_write(MPUREG_CONFIG, config, true);

    if (_mpu_type != Invensense_MPU6000) {
        if (_fast_sampling) {
            // setup for 4kHz accels
            _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_FCHOICE_B, true);
        } else {
            uint8_t fifo_size = (_mpu_type == Invensense_ICM20789 || _mpu_type == Invensense_ICM20689) ? 1:0;
            _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_DLPF_CFG_218HZ | (fifo_size<<6), true);
        }
    }
}

/*
  check whoami for sensor type
 */
bool AP_InertialSensor_Invensense::_check_whoami(void)
{
    uint8_t whoami = _register_read(MPUREG_WHOAMI);
    switch (whoami) {
    case MPU_WHOAMI_6000:
        _mpu_type = Invensense_MPU6000;
        return true;
    case MPU_WHOAMI_6500:
        _mpu_type = Invensense_MPU6500;
        return true;
    case MPU_WHOAMI_MPU9250:
    case MPU_WHOAMI_MPU9255:
        _mpu_type = Invensense_MPU9250;
        return true;
    case MPU_WHOAMI_20608D:    
    case MPU_WHOAMI_20608G:
        _mpu_type = Invensense_ICM20608;
        return true;
    case MPU_WHOAMI_20602:
        _mpu_type = Invensense_ICM20602;
        return true;
    case MPU_WHOAMI_20601:
        _mpu_type = Invensense_ICM20601;
        return true;
    case MPU_WHOAMI_ICM20789:
    case MPU_WHOAMI_ICM20789_R1:
        _mpu_type = Invensense_ICM20789;
        return true;
    case MPU_WHOAMI_ICM20689:
        _mpu_type = Invensense_ICM20689;
        return true;
    }
    // not a value WHOAMI result
    return false;
}


bool AP_InertialSensor_Invensense::_hardware_init(void)
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    // setup for register checking. We check much less often on I2C
    // where the cost of the checks is higher
    _dev->setup_checked_registers(13, _dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C?200:20);
    
    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!_check_whoami()) {
        return false;
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        _last_stat_user_ctrl = _register_read(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
        if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
            /* reset signal path as recommended in the datasheet */
            if (_mpu_type == Invensense_MPU6000 || _mpu_type == Invensense_MPU6500) {
                _register_write(MPUREG_SIGNAL_PATH_RESET,
                    BIT_SIGNAL_PATH_RESET_TEMP_RESET|BIT_SIGNAL_PATH_RESET_ACCEL_RESET|BIT_SIGNAL_PATH_RESET_GYRO_RESET);
                hal.scheduler->delay(100);
            }

            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
            _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl);
        }

        /* bus-dependent initialization */
        if ((_dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C) && (_mpu_type == Invensense_MPU9250 || _mpu_type == Invensense_ICM20789)) {
            /* Enable I2C bypass to access internal device */
            _register_write(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN);
        }


        // Wake up device and select GyroZ clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        hal.scheduler->delay(10);
        if (_data_ready()) {
            break;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (tries == 5) {
        DEV_PRINTF("Failed to boot Invensense 5 times\n");
        return false;
    }

    if (_mpu_type == Invensense_ICM20608 ||
        _mpu_type == Invensense_ICM20602 ||
        _mpu_type == Invensense_ICM20601) {
        // this avoids a sensor bug, see description above
        _register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
    }

    return true;
}

AP_Invensense_AuxiliaryBusSlave::AP_Invensense_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr,
                                                         uint8_t instance)
    : AuxiliaryBusSlave(bus, addr, instance)
    , _mpu_addr(MPUREG_I2C_SLV0_ADDR + _instance * 3)
    , _mpu_reg(_mpu_addr + 1)
    , _mpu_ctrl(_mpu_addr + 2)
    , _mpu_do(MPUREG_I2C_SLV0_DO + _instance)
{
}

int AP_Invensense_AuxiliaryBusSlave::_set_passthrough(uint8_t reg, uint8_t size,
                                                  uint8_t *out)
{
    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());
    uint8_t addr;

    /* Ensure the slave read/write is disabled before changing the registers */
    backend._register_write(_mpu_ctrl, 0);

    if (out) {
        backend._register_write(_mpu_do, *out);
        addr = _addr;
    } else {
        addr = _addr | BIT_READ_FLAG;
    }

    backend._register_write(_mpu_addr, addr);
    backend._register_write(_mpu_reg, reg);
    backend._register_write(_mpu_ctrl, BIT_I2C_SLVX_EN | size);

    return 0;
}

int AP_Invensense_AuxiliaryBusSlave::passthrough_read(uint8_t reg, uint8_t *buf,
                                                   uint8_t size)
{
    if (_registered) {
        DEV_PRINTF("Error: can't passthrough when slave is already configured\n");
        return -1;
    }

    int r = _set_passthrough(reg, size);
    if (r < 0) {
        return r;
    }

    /* wait the value to be read from the slave and read it back */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, size)) {
        return -1;
    }

    /* disable new reads */
    backend._register_write(_mpu_ctrl, 0);

    return size;
}

int AP_Invensense_AuxiliaryBusSlave::passthrough_write(uint8_t reg, uint8_t val)
{
    if (_registered) {
        DEV_PRINTF("Error: can't passthrough when slave is already configured\n");
        return -1;
    }

    int r = _set_passthrough(reg, 1, &val);
    if (r < 0) {
        return r;
    }

    /* wait the value to be written to the slave */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());

    /* disable new writes */
    backend._register_write(_mpu_ctrl, 0);

    return 1;
}

int AP_Invensense_AuxiliaryBusSlave::read(uint8_t *buf)
{
    if (!_registered) {
        DEV_PRINTF("Error: can't read before configuring slave\n");
        return -1;
    }

    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, _sample_size)) {
        return -1;
    }

    return _sample_size;
}

/* Invensense provides up to 5 slave devices, but the 5th is way too different to
 * configure and is seldom used */
AP_Invensense_AuxiliaryBus::AP_Invensense_AuxiliaryBus(AP_InertialSensor_Invensense &backend, uint32_t devid)
    : AuxiliaryBus(backend, 4, devid)
{
}

AP_HAL::Semaphore *AP_Invensense_AuxiliaryBus::get_semaphore()
{
    return static_cast<AP_InertialSensor_Invensense&>(_ins_backend)._dev->get_semaphore();
}

AuxiliaryBusSlave *AP_Invensense_AuxiliaryBus::_instantiate_slave(uint8_t addr, uint8_t instance)
{
    /* Enable slaves on Invensense if this is the first time */
    if (_ext_sens_data == 0) {
        _configure_slaves();
    }

    return new AP_Invensense_AuxiliaryBusSlave(*this, addr, instance);
}

void AP_Invensense_AuxiliaryBus::_configure_slaves()
{
    auto &backend = AP_InertialSensor_Invensense::from(_ins_backend);

    if (backend._mpu_type == AP_InertialSensor_Invensense::Invensense_ICM20789) {
        // on 20789 we can't enable slaves if we want to be able to use the baro
        return;
    }
    
    WITH_SEMAPHORE(backend._dev->get_semaphore());

    /* Enable the I2C master to slaves on the auxiliary I2C bus*/
    if (!(backend._last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN)) {
        backend._last_stat_user_ctrl |= BIT_USER_CTRL_I2C_MST_EN;
        backend._register_write(MPUREG_USER_CTRL, backend._last_stat_user_ctrl);
    }

    /* stop condition between reads; clock at 400kHz */
    backend._register_write(MPUREG_I2C_MST_CTRL,
                            BIT_I2C_MST_P_NSR | BIT_I2C_MST_CLK_400KHZ);

    /* Hard-code divider for internal sample rate, 1 kHz, resulting in a
     * sample rate of 100Hz */
    backend._register_write(MPUREG_I2C_SLV4_CTRL, 9);

    /* All slaves are subject to the sample rate */
    backend._register_write(MPUREG_I2C_MST_DELAY_CTRL,
                            BIT_I2C_SLV0_DLY_EN | BIT_I2C_SLV1_DLY_EN |
                            BIT_I2C_SLV2_DLY_EN | BIT_I2C_SLV3_DLY_EN);
}

int AP_Invensense_AuxiliaryBus::_configure_periodic_read(AuxiliaryBusSlave *slave,
                                                     uint8_t reg, uint8_t size)
{
    if (_ext_sens_data + size > MAX_EXT_SENS_DATA) {
        return -1;
    }

    AP_Invensense_AuxiliaryBusSlave *mpu_slave =
        static_cast<AP_Invensense_AuxiliaryBusSlave*>(slave);
    mpu_slave->_set_passthrough(reg, size);
    mpu_slave->_ext_sens_data = _ext_sens_data;
    _ext_sens_data += size;

    return 0;
}

AP_HAL::Device::PeriodicHandle AP_Invensense_AuxiliaryBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    auto &backend = AP_InertialSensor_Invensense::from(_ins_backend);
    return backend._dev->register_periodic_callback(period_usec, cb);
}
