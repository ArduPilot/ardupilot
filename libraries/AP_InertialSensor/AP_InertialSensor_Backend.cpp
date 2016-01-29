/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <DataFlash/DataFlash.h>

const extern AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu),
    _product_id(AP_PRODUCT_ID_NONE)
{}

void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) 
{
    /*
      accel calibration is always done in sensor frame with this
      version of the code. That means we apply the rotation after the
      offsets and scaling.
     */

    // apply offsets
    accel -= _imu._accel_offset[instance];

    // apply scaling
    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    accel.x *= accel_scale.x;
    accel.y *= accel_scale.y;
    accel.z *= accel_scale.z;

    // rotate to body frame
    accel.rotate(_imu._board_orientation);
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) 
{
    // gyro calibration is always assumed to have been done in sensor frame
    gyro -= _imu._gyro_offset[instance];
    gyro.rotate(_imu._board_orientation);
}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_publish_gyro(uint8_t instance, const Vector3f &gyro)
{
    _imu._gyro[instance] = gyro;
    _imu._gyro_healthy[instance] = true;

    if (_imu._gyro_raw_sample_rates[instance] <= 0) {
        return;
    }

    // publish delta angle
    _imu._delta_angle[instance] = _imu._delta_angle_acc[instance];
    _imu._delta_angle_dt[instance] = _imu._delta_angle_acc_dt[instance];
    _imu._delta_angle_valid[instance] = true;
}

void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(uint8_t instance,
                                                            const Vector3f &gyro,
                                                            uint64_t sample_us)
{
    float dt;

    if (_imu._gyro_raw_sample_rates[instance] <= 0) {
        return;
    }

    dt = 1.0f / _imu._gyro_raw_sample_rates[instance];

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

    // integrate delta angle accumulator
    // the angles and coning corrections are accumulated separately in the
    // referenced paper, but in simulation little difference was found between
    // integrating together and integrating separately (see examples/coning.py)
    _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
    _imu._delta_angle_acc_dt[instance] += dt;

    // save previous delta angle for coning correction
    _imu._last_delta_angle[instance] = delta_angle;
    _imu._last_raw_gyro[instance] = gyro;

    _imu._gyro_filtered[instance] = _imu._gyro_filter[instance].apply(gyro);
    if (_imu._gyro_filtered[instance].is_nan() || _imu._gyro_filtered[instance].is_inf()) {
        _imu._gyro_filter[instance].reset();
    }

    _imu._new_gyro_data[instance] = true;

    DataFlash_Class *dataflash = get_dataflash();
    if (dataflash != NULL) {
        uint64_t now = AP_HAL::micros64();
        struct log_GYRO pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            GyrX      : gyro.x,
            GyrY      : gyro.y,
            GyrZ      : gyro.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
    }
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_publish_accel(uint8_t instance, const Vector3f &accel)
{
    _imu._accel[instance] = accel;
    _imu._accel_healthy[instance] = true;

    if (_imu._accel_raw_sample_rates[instance] <= 0) {
        return;
    }

    // publish delta velocity
    _imu._delta_velocity[instance] = _imu._delta_velocity_acc[instance];
    _imu._delta_velocity_dt[instance] = _imu._delta_velocity_acc_dt[instance];
    _imu._delta_velocity_valid[instance] = true;


    if (_imu._accel_calibrator != NULL && _imu._accel_calibrator[instance].get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        Vector3f cal_sample = _imu._delta_velocity[instance];

        //remove rotation
        cal_sample.rotate_inverse(_imu._board_orientation);

        // remove scale factors
        const Vector3f &accel_scale = _imu._accel_scale[instance].get();
        cal_sample.x /= accel_scale.x;
        cal_sample.y /= accel_scale.y;
        cal_sample.z /= accel_scale.z;

        //remove offsets
        cal_sample += _imu._accel_offset[instance].get() * _imu._delta_velocity_dt[instance] ;

        _imu._accel_calibrator[instance].new_sample(cal_sample, _imu._delta_velocity_dt[instance]);
    }
}

void AP_InertialSensor_Backend::_notify_new_accel_raw_sample(uint8_t instance,
                                                             const Vector3f &accel,
                                                             uint64_t sample_us)
{
    float dt;

    if (_imu._accel_raw_sample_rates[instance] <= 0) {
        return;
    }

    dt = 1.0f / _imu._accel_raw_sample_rates[instance];

    _imu.calc_vibration_and_clipping(instance, accel, dt);

    // delta velocity
    _imu._delta_velocity_acc[instance] += accel * dt;
    _imu._delta_velocity_acc_dt[instance] += dt;

    _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);
    if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
        _imu._accel_filter[instance].reset();
    }

    _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);

    _imu._new_accel_data[instance] = true;

    DataFlash_Class *dataflash = get_dataflash();
    if (dataflash != NULL) {
        uint64_t now = AP_HAL::micros64();
        struct log_ACCEL pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACC1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            AccX      : accel.x,
            AccY      : accel.y,
            AccZ      : accel.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
    }
}

void AP_InertialSensor_Backend::_set_accel_max_abs_offset(uint8_t instance,
                                                          float max_offset)
{
    _imu._accel_max_abs_offsets[instance] = max_offset;
}

// set accelerometer error_count
void AP_InertialSensor_Backend::_set_accel_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._accel_error_count[instance] = error_count;
}

// set gyro error_count
void AP_InertialSensor_Backend::_set_gyro_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._gyro_error_count[instance] = error_count;
}

// return the requested sample rate in Hz
uint16_t AP_InertialSensor_Backend::get_sample_rate_hz(void) const
{
    // enum can be directly cast to Hz
    return (uint16_t)_imu._sample_rate;
}

/*
  publish a temperature value for an instance
 */
void AP_InertialSensor_Backend::_publish_temperature(uint8_t instance, float temperature)
{
    _imu._temperature[instance] = temperature;
}

/*
  common gyro update function for all backends
 */
void AP_InertialSensor_Backend::update_gyro(uint8_t instance)
{    
    hal.scheduler->suspend_timer_procs();

    if (_imu._new_gyro_data[instance]) {
        _publish_gyro(instance, _imu._gyro_filtered[instance]);
        _imu._new_gyro_data[instance] = false;
    }

    // possibly update filter frequency
    if (_last_gyro_filter_hz[instance] != _gyro_filter_cutoff()) {
        _imu._gyro_filter[instance].set_cutoff_frequency(_gyro_raw_sample_rate(instance), _gyro_filter_cutoff());
        _last_gyro_filter_hz[instance] = _gyro_filter_cutoff();
    }

    hal.scheduler->resume_timer_procs();
}

/*
  common accel update function for all backends
 */
void AP_InertialSensor_Backend::update_accel(uint8_t instance)
{    
    hal.scheduler->suspend_timer_procs();

    if (_imu._new_accel_data[instance]) {
        _publish_accel(instance, _imu._accel_filtered[instance]);
        _imu._new_accel_data[instance] = false;
    }
    
    // possibly update filter frequency
    if (_last_accel_filter_hz[instance] != _accel_filter_cutoff()) {
        _imu._accel_filter[instance].set_cutoff_frequency(_accel_raw_sample_rate(instance), _accel_filter_cutoff());
        _last_accel_filter_hz[instance] = _accel_filter_cutoff();
    }

    hal.scheduler->resume_timer_procs();
}
