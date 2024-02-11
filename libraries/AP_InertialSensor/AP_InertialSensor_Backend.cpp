#define AP_INLINE_VECTOR_OPS

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#if AP_MODULE_SUPPORTED
#include <AP_Module/AP_Module.h>
#endif
#include <stdio.h>

#define SENSOR_RATE_DEBUG 0

#ifndef AP_HEATER_IMU_INSTANCE
#define AP_HEATER_IMU_INSTANCE 0
#endif

const extern AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu)
{
}

/*
  notify of a FIFO reset so we don't use bad data to update observed sensor rate
 */
void AP_InertialSensor_Backend::notify_accel_fifo_reset(uint8_t instance)
{
    _imu._sample_accel_count[instance] = 0;
    _imu._sample_accel_start_us[instance] = 0;    
}

/*
  notify of a FIFO reset so we don't use bad data to update observed sensor rate
 */
void AP_InertialSensor_Backend::notify_gyro_fifo_reset(uint8_t instance)
{
    _imu._sample_gyro_count[instance] = 0;
    _imu._sample_gyro_start_us[instance] = 0;
}

// set the amount of oversamping a accel is doing
void AP_InertialSensor_Backend::_set_accel_oversampling(uint8_t instance, uint8_t n)
{
    _imu._accel_over_sampling[instance] = n;
}

// set the amount of oversamping a gyro is doing
void AP_InertialSensor_Backend::_set_gyro_oversampling(uint8_t instance, uint8_t n)
{
    _imu._gyro_over_sampling[instance] = n;
}

/*
  update the sensor rate for FIFO sensors

  FIFO sensors produce samples at a fixed rate, but the clock in the
  sensor may vary slightly from the system clock. This slowly adjusts
  the rate to the observed rate
*/
void AP_InertialSensor_Backend::_update_sensor_rate(uint16_t &count, uint32_t &start_us, float &rate_hz) const
{
    uint32_t now = AP_HAL::micros();
    if (start_us == 0) {
        count = 0;
        start_us = now;
    } else {
        count++;
        if (now - start_us > 1000000UL) {
            float observed_rate_hz = count * 1.0e6f / (now - start_us);
#if 0
            printf("IMU RATE: %.1f should be %.1f\n", observed_rate_hz, rate_hz);
#endif
            float filter_constant = 0.98f;
            float upper_limit = 1.05f;
            float lower_limit = 0.95f;
            if (sensors_converging()) {
                // converge quickly for first 30s, then more slowly
                filter_constant = 0.8f;
                upper_limit = 2.0f;
                lower_limit = 0.5f;
            }
            observed_rate_hz = constrain_float(observed_rate_hz, rate_hz*lower_limit, rate_hz*upper_limit);
            rate_hz = filter_constant * rate_hz + (1-filter_constant) * observed_rate_hz;
            count = 0;
            start_us = now;
        }
    }
}

void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) 
{
    /*
      accel calibration is always done in sensor frame with this
      version of the code. That means we apply the rotation after the
      offsets and scaling.
     */

    // rotate for sensor orientation
    accel.rotate(_imu._accel_orientation[instance]);

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    if (_imu.tcal_learning) {
        _imu.tcal(instance).update_accel_learning(accel, _imu.get_temperature(instance));
    }
#endif

    if (!_imu._calibrating_accel && (_imu._acal == nullptr
#if HAL_INS_ACCELCAL_ENABLED
        || !_imu._acal->running()
#endif
    )) {

#if HAL_INS_TEMPERATURE_CAL_ENABLE
        // apply temperature corrections
        _imu.tcal(instance).correct_accel(_imu.get_temperature(instance), _imu.caltemp_accel(instance), accel);
#endif

        // apply offsets
        accel -= _imu._accel_offset(instance);


        // apply scaling
        const Vector3f &accel_scale = _imu._accel_scale(instance).get();
        accel.x *= accel_scale.x;
        accel.y *= accel_scale.y;
        accel.z *= accel_scale.z;
    }

    // rotate to body frame
    accel.rotate(_imu._board_orientation);
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) 
{
    // rotate for sensor orientation
    gyro.rotate(_imu._gyro_orientation[instance]);

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    if (_imu.tcal_learning) {
        _imu.tcal(instance).update_gyro_learning(gyro, _imu.get_temperature(instance));
    }
#endif
    
    if (!_imu._calibrating_gyro) {

#if HAL_INS_TEMPERATURE_CAL_ENABLE
        // apply temperature corrections
        _imu.tcal(instance).correct_gyro(_imu.get_temperature(instance), _imu.caltemp_gyro(instance), gyro);
#endif

        // gyro calibration is always assumed to have been done in sensor frame
        gyro -= _imu._gyro_offset(instance);
    }

    gyro.rotate(_imu._board_orientation);
}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_publish_gyro(uint8_t instance, const Vector3f &gyro) /* front end */
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    _imu._gyro[instance] = gyro;
    _imu._gyro_healthy[instance] = true;

    // publish delta angle
    _imu._delta_angle[instance] = _imu._delta_angle_acc[instance];
    _imu._delta_angle_dt[instance] = _imu._delta_angle_acc_dt[instance];
    _imu._delta_angle_valid[instance] = true;

    _imu._delta_angle_acc[instance].zero();
    _imu._delta_angle_acc_dt[instance] = 0;
}


void AP_InertialSensor_Backend::save_gyro_window(const uint8_t instance, const Vector3f &gyro, uint8_t phase)
{
#if HAL_GYROFFT_ENABLED
    // capture gyro window for FFT analysis
    if (_imu._fft_window_phase == phase) {
        if (_imu._gyro_window_size > 0) {
            Vector3f scaled_gyro = gyro * _imu._gyro_raw_sampling_multiplier[instance];
            // LPF always must come last to remove high-frequency shot noise, but the FFT still
            // needs to see the same data so gets its own LPF at the tap point
            if (_imu._post_filter_fft) {
                scaled_gyro = _imu._post_filter_gyro_filter[instance].apply(scaled_gyro);
            }
            _imu._gyro_window[instance][0].push(scaled_gyro.x);
            _imu._gyro_window[instance][1].push(scaled_gyro.y);
            _imu._gyro_window[instance][2].push(scaled_gyro.z);
            _imu._last_gyro_for_fft[instance] = scaled_gyro;
        } else {
            _imu._last_gyro_for_fft[instance] = gyro * _imu._gyro_raw_sampling_multiplier[instance];;
        }
    }
#endif
}

/*
  apply harmonic notch and low pass gyro filters
 */
void AP_InertialSensor_Backend::apply_gyro_filters(const uint8_t instance, const Vector3f &gyro)
{
    uint8_t filter_phase = 0;
    save_gyro_window(instance, gyro, filter_phase++);

    Vector3f gyro_filtered = gyro;
#if AP_AHRS_ENABLED
    const uint8_t primary_gyro = AP::ahrs().get_primary_gyro_index();
#endif

    // apply the harmonic notch filters
    for (auto &notch : _imu.harmonic_notches) {
        if (!notch.params.enabled()) {
            continue;
        }
        bool inactive = notch.is_inactive();
#if AP_AHRS_ENABLED
        // by default we only run the expensive notch filters on the
        // currently active IMU we reset the inactive notch filters so
        // that if we switch IMUs we're not left with old data
        if (!notch.params.hasOption(HarmonicNotchFilterParams::Options::EnableOnAllIMUs) &&
            instance != primary_gyro) {
            inactive = true;
        }
#endif
        if (inactive) {
            // while inactive we reset the filter so when it activates the first output
            // will be the first input sample
            notch.filter[instance].reset();
        } else {
            gyro_filtered = notch.filter[instance].apply(gyro_filtered);
        }
        save_gyro_window(instance, gyro_filtered, filter_phase++);
    }

    // apply the low pass filter last to attenuate any notch induced noise
    gyro_filtered = _imu._gyro_filter[instance].apply(gyro_filtered);

    // if the filtering failed in any way then reset the filters and keep the old value
    if (gyro_filtered.is_nan() || gyro_filtered.is_inf()) {
        _imu._gyro_filter[instance].reset();
#if HAL_GYROFFT_ENABLED
        _imu._post_filter_gyro_filter[instance].reset();
#endif
        for (auto &notch : _imu.harmonic_notches) {
            notch.filter[instance].reset();
        }
    } else {
        _imu._gyro_filtered[instance] = gyro_filtered;
    }

#if AP_AHRS_ENABLED
    if (_imu.rate_loop_sem != nullptr && instance == primary_gyro) {
        /*
          tell the rate thread we have a new sample
        */
        _imu._gyro[instance] = gyro_filtered;
        _imu.rate_loop_sem->signal();
    }
#endif
}

void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(uint8_t instance,
                                                            const Vector3f &gyro,
                                                            uint64_t sample_us)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_gyro_count[instance], _imu._sample_gyro_start_us[instance],
                        _imu._gyro_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._gyro_last_sample_us[instance];

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _imu._gyro_last_sample_us[instance] != 0) {
        dt = (sample_us - _imu._gyro_last_sample_us[instance]) * 1.0e-6f;
        _imu._gyro_last_sample_us[instance] = sample_us;
    } else {
        // don't accept below 40Hz
        if (_imu._gyro_raw_sample_rates[instance] < 40) {
            return;
        }

        dt = 1.0f / _imu._gyro_raw_sample_rates[instance];
        _imu._gyro_last_sample_us[instance] = AP_HAL::micros64();
        sample_us = _imu._gyro_last_sample_us[instance];
    }

#if AP_MODULE_SUPPORTED
    // call gyro_sample hook if any
    AP_Module::call_hook_gyro_sample(instance, dt, gyro);
#endif

    // push gyros if optical flow present
    if (hal.opticalflow) {
        hal.opticalflow->push_gyro(gyro.x, gyro.y, dt);
    }
    
    // compute delta angle
    Vector3f delta_angle = (gyro + _imu._last_raw_gyro[instance]) * 0.5f * dt;

    // compute coning correction
    // see page 26 of:
    // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
    // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
    // see also examples/coning.py
    Vector3f delta_coning = (_imu._delta_angle_acc[instance] +
                             _imu._last_delta_angle[instance] * (1.0f / 6.0f));
    delta_coning = delta_coning % delta_angle;
    delta_coning *= 0.5f;

    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_angle_acc[instance].zero();
            _imu._delta_angle_acc_dt[instance] = 0;
            dt = 0;
            delta_angle.zero();
        }

        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
        _imu._delta_angle_acc_dt[instance] += dt;

        // save previous delta angle for coning correction
        _imu._last_delta_angle[instance] = delta_angle;
        _imu._last_raw_gyro[instance] = gyro;

        // apply gyro filters and sample for FFT
        apply_gyro_filters(instance, gyro);

        _imu._new_gyro_data[instance] = true;
    }

    // 5us
    log_gyro_raw(instance, sample_us, gyro, _imu._gyro_filtered[instance]);
}

/*
  handle a delta-angle sample from the backend. This assumes FIFO
  style sampling and the sample should not be rotated or corrected for
  offsets.
  This function should be used when the sensor driver can directly
  provide delta-angle values from the sensor.
 */
void AP_InertialSensor_Backend::_notify_new_delta_angle(uint8_t instance, const Vector3f &dangle)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_gyro_count[instance], _imu._sample_gyro_start_us[instance],
                        _imu._gyro_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._gyro_last_sample_us[instance];

    // don't accept below 40Hz
    if (_imu._gyro_raw_sample_rates[instance] < 40) {
        return;
    }

    dt = 1.0f / _imu._gyro_raw_sample_rates[instance];
    _imu._gyro_last_sample_us[instance] = AP_HAL::micros64();
    uint64_t sample_us = _imu._gyro_last_sample_us[instance];

    Vector3f gyro = dangle / dt;

    _rotate_and_correct_gyro(instance, gyro);

#if AP_MODULE_SUPPORTED
    // call gyro_sample hook if any
    AP_Module::call_hook_gyro_sample(instance, dt, gyro);
#endif

    // push gyros if optical flow present
    if (hal.opticalflow) {
        hal.opticalflow->push_gyro(gyro.x, gyro.y, dt);
    }
    
    // compute delta angle, including corrections
    Vector3f delta_angle = gyro * dt;

    // compute coning correction
    // see page 26 of:
    // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
    // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
    // see also examples/coning.py
    Vector3f delta_coning = (_imu._delta_angle_acc[instance] +
                             _imu._last_delta_angle[instance] * (1.0f / 6.0f));
    delta_coning = delta_coning % delta_angle;
    delta_coning *= 0.5f;

    {
        WITH_SEMAPHORE(_sem);
        uint64_t now = AP_HAL::micros64();

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_angle_acc[instance].zero();
            _imu._delta_angle_acc_dt[instance] = 0;
            dt = 0;
            delta_angle.zero();
        }

        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
        _imu._delta_angle_acc_dt[instance] += dt;

        // save previous delta angle for coning correction
        _imu._last_delta_angle[instance] = delta_angle;
        _imu._last_raw_gyro[instance] = gyro;

        // apply gyro filters and sample for FFT
        apply_gyro_filters(instance, gyro);

        _imu._new_gyro_data[instance] = true;
    }

    log_gyro_raw(instance, sample_us, gyro, _imu._gyro_filtered[instance]);
}

void AP_InertialSensor_Backend::log_gyro_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &raw_gyro, const Vector3f &filtered_gyro)
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        // should not have been called
        return;
    }

#if AP_AHRS_ENABLED
    const bool log_because_primary_gyro = _imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::PRIMARY_GYRO_ONLY) && (instance == AP::ahrs().get_primary_gyro_index());
#else
    const bool log_because_primary_gyro = false;
#endif

    if (_imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::ALL_GYROS) ||
        log_because_primary_gyro ||
        should_log_imu_raw()) {

        if (_imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::PRE_AND_POST_FILTER)) {
            // Both pre and post, offset post instance as batch sampler does
            Write_GYR(instance, sample_us, raw_gyro);
            Write_GYR(instance + _imu._gyro_count, sample_us, filtered_gyro);

        } else if (_imu.raw_logging_option_set(AP_InertialSensor::RAW_LOGGING_OPTION::POST_FILTER)) {
            // Just post
            Write_GYR(instance, sample_us, filtered_gyro);

        } else {
            // Just pre
            Write_GYR(instance, sample_us, raw_gyro);

        }
    } else {
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
        if (!_imu.batchsampler.doing_sensor_rate_logging()) {
            _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_GYRO, sample_us,
                                     !_imu.batchsampler.doing_post_filter_logging() ? raw_gyro : filtered_gyro);
        }
#endif
    }
#endif
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_publish_accel(uint8_t instance, const Vector3f &accel) /* front end */
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    _imu._accel[instance] = accel;
    _imu._accel_healthy[instance] = true;

    // publish delta velocity
    _imu._delta_velocity[instance] = _imu._delta_velocity_acc[instance];
    _imu._delta_velocity_dt[instance] = _imu._delta_velocity_acc_dt[instance];
    _imu._delta_velocity_valid[instance] = true;

    _imu._delta_velocity_acc[instance].zero();
    _imu._delta_velocity_acc_dt[instance] = 0;

    if (_imu._accel_calibrator != nullptr && _imu._accel_calibrator[instance].get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        Vector3f cal_sample = _imu._delta_velocity[instance];

        // remove rotation. Note that we don't need to remove offsets or scale factor as those
        // are not applied when calibrating
        cal_sample.rotate_inverse(_imu._board_orientation);

        _imu._accel_calibrator[instance].new_sample(cal_sample, _imu._delta_velocity_dt[instance]);
    }
}

void AP_InertialSensor_Backend::_notify_new_accel_raw_sample(uint8_t instance,
                                                             const Vector3f &accel,
                                                             uint64_t sample_us,
                                                             bool fsync_set)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_accel_count[instance], _imu._sample_accel_start_us[instance],
                        _imu._accel_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._accel_last_sample_us[instance];

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _imu._accel_last_sample_us[instance] != 0) {
        dt = (sample_us - _imu._accel_last_sample_us[instance]) * 1.0e-6f;
        _imu._accel_last_sample_us[instance] = sample_us;
    } else {
        // don't accept below 40Hz
        if (_imu._accel_raw_sample_rates[instance] < 40) {
            return;
        }

        dt = 1.0f / _imu._accel_raw_sample_rates[instance];
        _imu._accel_last_sample_us[instance] = AP_HAL::micros64();
        sample_us = _imu._accel_last_sample_us[instance];
    }

#if AP_MODULE_SUPPORTED
    // call accel_sample hook if any
    AP_Module::call_hook_accel_sample(instance, dt, accel, fsync_set);
#endif    
    
    _imu.calc_vibration_and_clipping(instance, accel, dt);

    {
        WITH_SEMAPHORE(_sem);

        uint64_t now = AP_HAL::micros64();

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_velocity_acc[instance].zero();
            _imu._delta_velocity_acc_dt[instance] = 0;
            dt = 0;
        }
        
        // delta velocity
        _imu._delta_velocity_acc[instance] += accel * dt;
        _imu._delta_velocity_acc_dt[instance] += dt;

        _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);
        if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
            _imu._accel_filter[instance].reset();
        }

        _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);

        _imu._new_accel_data[instance] = true;
    }

    // 5us
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    if (!_imu.batchsampler.doing_post_filter_logging()) {
        log_accel_raw(instance, sample_us, accel);
    } else {
        log_accel_raw(instance, sample_us, _imu._accel_filtered[instance]);
    }
#else
    // assume we're doing pre-filter logging:
    log_accel_raw(instance, sample_us, accel);
#endif
}

/*
  handle a delta-velocity sample from the backend. This assumes FIFO style sampling and
  the sample should not be rotated or corrected for offsets

  This function should be used when the sensor driver can directly
  provide delta-velocity values from the sensor.
 */
void AP_InertialSensor_Backend::_notify_new_delta_velocity(uint8_t instance, const Vector3f &dvel)
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    float dt;

    _update_sensor_rate(_imu._sample_accel_count[instance], _imu._sample_accel_start_us[instance],
                        _imu._accel_raw_sample_rates[instance]);

    uint64_t last_sample_us = _imu._accel_last_sample_us[instance];

    // don't accept below 40Hz
    if (_imu._accel_raw_sample_rates[instance] < 40) {
        return;
    }

    dt = 1.0f / _imu._accel_raw_sample_rates[instance];
    _imu._accel_last_sample_us[instance] = AP_HAL::micros64();
    uint64_t sample_us = _imu._accel_last_sample_us[instance];

    Vector3f accel = dvel / dt;

    _rotate_and_correct_accel(instance, accel);

#if AP_MODULE_SUPPORTED
    // call accel_sample hook if any
    AP_Module::call_hook_accel_sample(instance, dt, accel, false);
#endif    
    
    _imu.calc_vibration_and_clipping(instance, accel, dt);

    {
        WITH_SEMAPHORE(_sem);

        uint64_t now = AP_HAL::micros64();

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            _imu._delta_velocity_acc[instance].zero();
            _imu._delta_velocity_acc_dt[instance] = 0;
            dt = 0;
        }
        
        // delta velocity including corrections
        _imu._delta_velocity_acc[instance] += accel * dt;
        _imu._delta_velocity_acc_dt[instance] += dt;

        _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);
        if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
            _imu._accel_filter[instance].reset();
        }

        _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);

        _imu._new_accel_data[instance] = true;
    }

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    if (!_imu.batchsampler.doing_post_filter_logging()) {
        log_accel_raw(instance, sample_us, accel);
    } else {
        log_accel_raw(instance, sample_us, _imu._accel_filtered[instance]);
    }
#else
    // assume we're doing pre-filter logging
    log_accel_raw(instance, sample_us, accel);
#endif
}


void AP_InertialSensor_Backend::_notify_new_accel_sensor_rate_sample(uint8_t instance, const Vector3f &_accel)
{
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    if (!_imu.batchsampler.doing_sensor_rate_logging()) {
        return;
    }

    // get batch sampling in correct orientation
    Vector3f accel = _accel;
    accel.rotate(_imu._accel_orientation[instance]);

    _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_ACCEL, AP_HAL::micros64(), accel);
#endif
}

void AP_InertialSensor_Backend::_notify_new_gyro_sensor_rate_sample(uint8_t instance, const Vector3f &_gyro)
{
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    if (!_imu.batchsampler.doing_sensor_rate_logging()) {
        return;
    }

    // get batch sampling in correct orientation
    Vector3f gyro = _gyro;
    gyro.rotate(_imu._gyro_orientation[instance]);

    _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_GYRO, AP_HAL::micros64(), gyro);
#endif
}

void AP_InertialSensor_Backend::log_accel_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &accel)
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        // should not have been called
        return;
    }
    if (should_log_imu_raw()) {
        Write_ACC(instance, sample_us, accel);
    } else {
#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
        if (!_imu.batchsampler.doing_sensor_rate_logging()) {
            _imu.batchsampler.sample(instance, AP_InertialSensor::IMU_SENSOR_TYPE_ACCEL, sample_us, accel);
        }
#endif
    }
#endif
}

void AP_InertialSensor_Backend::_set_accel_max_abs_offset(uint8_t instance,
                                                          float max_offset)
{
    _imu._accel_max_abs_offsets[instance] = max_offset;
}

// increment accelerometer error_count
void AP_InertialSensor_Backend::_inc_accel_error_count(uint8_t instance)
{
    _imu._accel_error_count[instance]++;
}

// increment gyro error_count
void AP_InertialSensor_Backend::_inc_gyro_error_count(uint8_t instance)
{
    _imu._gyro_error_count[instance]++;
}

/*
  publish a temperature value for an instance
 */
void AP_InertialSensor_Backend::_publish_temperature(uint8_t instance, float temperature) /* front end */
{
    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    _imu._temperature[instance] = temperature;

#if HAL_HAVE_IMU_HEATER
    /* give the temperature to the control loop in order to keep it constant*/
    if (instance == AP_HEATER_IMU_INSTANCE) {
        AP_BoardConfig *bc = AP::boardConfig();
        if (bc) {
            bc->set_imu_temp(temperature);
        }
    }
#endif
}

/*
  common gyro update function for all backends
 */
void AP_InertialSensor_Backend::update_gyro(uint8_t instance) /* front end */
{    
    WITH_SEMAPHORE(_sem);

    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    if (_imu._new_gyro_data[instance]) {
        _publish_gyro(instance, _imu._gyro_filtered[instance]);
#if HAL_GYROFFT_ENABLED
        // copy the gyro samples from the backend to the frontend window for FFTs sampling at less than IMU rate
        _imu._gyro_for_fft[instance] = _imu._last_gyro_for_fft[instance];
#endif
        _imu._new_gyro_data[instance] = false;
    }

    // possibly update filter frequency
    const float gyro_rate = _gyro_raw_sample_rate(instance);

    if (_last_gyro_filter_hz != _gyro_filter_cutoff() || sensors_converging()) {
        _imu._gyro_filter[instance].set_cutoff_frequency(gyro_rate, _gyro_filter_cutoff());
#if HAL_GYROFFT_ENABLED
        _imu._post_filter_gyro_filter[instance].set_cutoff_frequency(gyro_rate, _gyro_filter_cutoff());
#endif
        _last_gyro_filter_hz = _gyro_filter_cutoff();
    }

    for (auto &notch : _imu.harmonic_notches) {
        if (notch.params.enabled()) {
            notch.update_params(instance, sensors_converging(), gyro_rate);
        }
    }
}

/*
  common accel update function for all backends
 */
void AP_InertialSensor_Backend::update_accel(uint8_t instance) /* front end */
{    
    WITH_SEMAPHORE(_sem);

    if ((1U<<instance) & _imu.imu_kill_mask) {
        return;
    }
    if (_imu._new_accel_data[instance]) {
        _publish_accel(instance, _imu._accel_filtered[instance]);
        _imu._new_accel_data[instance] = false;
    }
    
    // possibly update filter frequency
    if (_last_accel_filter_hz != _accel_filter_cutoff()) {
        _imu._accel_filter[instance].set_cutoff_frequency(_accel_raw_sample_rate(instance), _accel_filter_cutoff());
        _last_accel_filter_hz = _accel_filter_cutoff();
    }
}

#if HAL_LOGGING_ENABLED
bool AP_InertialSensor_Backend::should_log_imu_raw() const
{
    if (_imu._log_raw_bit == (uint32_t)-1) {
        // tracker does not set a bit
        return false;
    }
    const AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return false;
    }
    if (!logger->should_log(_imu._log_raw_bit)) {
        return false;
    }
    return true;
}
#endif  // HAL_LOGGING_ENABLED

// log an unexpected change in a register for an IMU
void AP_InertialSensor_Backend::log_register_change(uint32_t bus_id, const AP_HAL::Device::checkreg &reg)
{
#if HAL_LOGGING_ENABLED
    AP::logger().Write("IREG", "TimeUS,DevID,Bank,Reg,Val", "QIBBB",
                       AP_HAL::micros64(),
                       bus_id,
                       reg.bank,
                       reg.regnum,
                       reg.value);
#endif
}
