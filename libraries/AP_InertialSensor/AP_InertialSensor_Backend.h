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
  IMU driver backend class. Each supported gyro/accel sensor type
  needs to have an object derived from this class.

  Note that drivers can implement just gyros or just accels, and can
  also provide multiple gyro/accel instances.
 */
#pragma once

#include <inttypes.h>

#include <AP_Math/AP_Math.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#include "AP_InertialSensor.h"

class AuxiliaryBus;
class AP_Logger;

class AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Backend(AP_InertialSensor &imu);
    AP_InertialSensor_Backend(const AP_InertialSensor_Backend &that) = delete;

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_InertialSensor_Backend(void) {}

    /*
     * Update the sensor data. Called by the frontend to transfer
     * accumulated sensor readings to the frontend structure via the
     * _publish_gyro() and _publish_accel() functions
     */
    virtual bool update() = 0; /* front end */

    /*
     * optional function to accumulate more samples. This is needed for drivers that don't use a timer to gather samples
     */
    virtual void accumulate() {}

    /*
     * Configure and start all sensors. The empty implementation allows
     * subclasses to already start the sensors when it's detected
     */
    virtual void start() { }

    /*
     * Return an AuxiliaryBus if backend has another bus it is able to export
     */
    virtual AuxiliaryBus *get_auxiliary_bus() { return nullptr; }

    /*
     * Return the unique identifier for this backend: it's the same for
     * several sensors if the backend registers more gyros/accels
     */
    int16_t get_id() const { return _id; }

    //Returns the Clip Limit
    float get_clip_limit() const { return _clip_limit; }

    // get a startup banner to output to the GCS
    virtual bool get_output_banner(char* banner, uint8_t banner_len) { return false; }

#if HAL_EXTERNAL_AHRS_ENABLED
    virtual void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt) {}
#endif

    /*
      device driver IDs. These are used to fill in the devtype field
      of the device ID, which shows up as INS*ID* parameters to
      users. The values are chosen for compatibility with existing PX4
      drivers.
      If a change is made to a driver that would make existing
      calibration values invalid then this number must be changed.
     */
    enum DevTypes {
        DEVTYPE_BMI160       = 0x09,
        DEVTYPE_L3G4200D     = 0x10,
        DEVTYPE_ACC_LSM303D  = 0x11,
        DEVTYPE_ACC_BMA180   = 0x12,
        DEVTYPE_ACC_MPU6000  = 0x13,
        DEVTYPE_ACC_MPU9250  = 0x16,
        DEVTYPE_ACC_IIS328DQ = 0x17,
        DEVTYPE_ACC_LSM9DS1  = 0x18,
        DEVTYPE_GYR_MPU6000  = 0x21,
        DEVTYPE_GYR_L3GD20   = 0x22,
        DEVTYPE_GYR_MPU9250  = 0x24,
        DEVTYPE_GYR_I3G4250D = 0x25,
        DEVTYPE_GYR_LSM9DS1  = 0x26,
        DEVTYPE_INS_ICM20789 = 0x27,
        DEVTYPE_INS_ICM20689 = 0x28,
        DEVTYPE_INS_BMI055   = 0x29,
        DEVTYPE_SITL         = 0x2A,
        DEVTYPE_INS_BMI088   = 0x2B,
        DEVTYPE_INS_ICM20948 = 0x2C,
        DEVTYPE_INS_ICM20648 = 0x2D,
        DEVTYPE_INS_ICM20649 = 0x2E,
        DEVTYPE_INS_ICM20602 = 0x2F,
        DEVTYPE_INS_ICM20601 = 0x30,
        DEVTYPE_INS_ADIS1647X = 0x31,
        DEVTYPE_SERIAL       = 0x32,
        DEVTYPE_INS_ICM40609 = 0x33,
        DEVTYPE_INS_ICM42688 = 0x34,
        DEVTYPE_INS_ICM42605 = 0x35,
        DEVTYPE_INS_ICM40605 = 0x36,
        DEVTYPE_INS_IIM42652 = 0x37,
        DEVTYPE_BMI270       = 0x38,
        DEVTYPE_INS_BMI085   = 0x39,
        DEVTYPE_INS_LSM6DS3  = 0x40,
    };

protected:
    // access to frontend
    AP_InertialSensor &_imu;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Default Clip Limit
    float _clip_limit = 15.5f * GRAVITY_MSS;

    void _rotate_and_correct_accel(uint8_t instance, Vector3f &accel) __RAMFUNC__;
    void _rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) __RAMFUNC__;

    // rotate gyro vector, offset and publish
    void _publish_gyro(uint8_t instance, const Vector3f &gyro) __RAMFUNC__; /* front end */

    // apply notch and lowpass gyro filters
    void apply_gyro_filters(const uint8_t instance, const Vector3f &gyro);

    // this should be called every time a new gyro raw sample is
    // available - be it published or not the sample is raw in the
    // sense that it's not filtered yet, but it must be rotated and
    // corrected (_rotate_and_correct_gyro)
    // The sample_us value must be provided for non-FIFO based
    // sensors, and should be set to zero for FIFO based sensors
    void _notify_new_gyro_raw_sample(uint8_t instance, const Vector3f &accel, uint64_t sample_us=0) __RAMFUNC__;

    // alternative interface using delta-angles. Rotation and correction is handled inside this function
    void _notify_new_delta_angle(uint8_t instance, const Vector3f &dangle);
    
    // rotate accel vector, scale, offset and publish
    void _publish_accel(uint8_t instance, const Vector3f &accel) __RAMFUNC__; /* front end */

    // this should be called every time a new accel raw sample is available -
    // be it published or not
    // the sample is raw in the sense that it's not filtered yet, but it must
    // be rotated and corrected (_rotate_and_correct_accel)
    // The sample_us value must be provided for non-FIFO based
    // sensors, and should be set to zero for FIFO based sensors
    void _notify_new_accel_raw_sample(uint8_t instance, const Vector3f &accel, uint64_t sample_us=0, bool fsync_set=false) __RAMFUNC__;

    // alternative interface using delta-velocities. Rotation and correction is handled inside this function
    void _notify_new_delta_velocity(uint8_t instance, const Vector3f &dvelocity);
    
    // set the amount of oversamping a accel is doing
    void _set_accel_oversampling(uint8_t instance, uint8_t n);

    // set the amount of oversamping a gyro is doing
    void _set_gyro_oversampling(uint8_t instance, uint8_t n);

    // indicate the backend is doing sensor-rate sampling for this accel
    void _set_accel_sensor_rate_sampling_enabled(uint8_t instance, bool value) {
        const uint8_t bit = (1<<instance);
        if (value) {
            _imu._accel_sensor_rate_sampling_enabled |= bit;
        } else {
            _imu._accel_sensor_rate_sampling_enabled &= ~bit;
        }
    }

    void _set_gyro_sensor_rate_sampling_enabled(uint8_t instance, bool value) {
        const uint8_t bit = (1<<instance);
        if (value) {
            _imu._gyro_sensor_rate_sampling_enabled |= bit;
        } else {
            _imu._gyro_sensor_rate_sampling_enabled &= ~bit;
        }
    }

    void _set_raw_sample_accel_multiplier(uint8_t instance, uint16_t mul) {
        _imu._accel_raw_sampling_multiplier[instance] = mul;
    }
    void _set_raw_sample_gyro_multiplier(uint8_t instance, uint16_t mul) {
        _imu._gyro_raw_sampling_multiplier[instance] = mul;
    }

    // update the sensor rate for FIFO sensors
    void _update_sensor_rate(uint16_t &count, uint32_t &start_us, float &rate_hz) const __RAMFUNC__;

    // return true if the sensors are still converging and sampling rates could change significantly
    bool sensors_converging() const { return AP_HAL::millis() < HAL_INS_CONVERGANCE_MS; }

    // set accelerometer max absolute offset for calibration
    void _set_accel_max_abs_offset(uint8_t instance, float offset);

    // get accelerometer raw sample rate.
    float _accel_raw_sample_rate(uint8_t instance) const {
        return _imu._accel_raw_sample_rates[instance];
    }

    // set accelerometer raw sample rate;  note that the storage type
    // is actually float!
    void _set_accel_raw_sample_rate(uint8_t instance, uint16_t rate_hz) {
        _imu._accel_raw_sample_rates[instance] = rate_hz;
    }
    
    // get gyroscope raw sample rate
    float _gyro_raw_sample_rate(uint8_t instance) const {
        return _imu._gyro_raw_sample_rates[instance];
    }

    // set gyro raw sample rate; note that the storage type is
    // actually float!
    void _set_gyro_raw_sample_rate(uint8_t instance, uint16_t rate_hz) {
        _imu._gyro_raw_sample_rates[instance] = rate_hz;
    }
    
    // publish a temperature value
    void _publish_temperature(uint8_t instance, float temperature); /* front end */

    // increment accelerometer error_count
    void _inc_accel_error_count(uint8_t instance) __RAMFUNC__;

    // increment gyro error_count
    void _inc_gyro_error_count(uint8_t instance) __RAMFUNC__;
    
    // backend unique identifier or -1 if backend doesn't identify itself
    int16_t _id = -1;

    // return the default filter frequency in Hz for the sample rate
    uint16_t _accel_filter_cutoff(void) const { return _imu._accel_filter_cutoff; }

    // return the default filter frequency in Hz for the sample rate
    uint16_t _gyro_filter_cutoff(void) const { return _imu._gyro_filter_cutoff; }

    // return the requested loop rate at which samples will be made available in Hz
    uint16_t get_loop_rate_hz(void) const {
        // enum can be directly cast to Hz
        return (uint16_t)_imu._loop_rate;
    }

    // common gyro update function for all backends
    void update_gyro(uint8_t instance) __RAMFUNC__; /* front end */

    // common accel update function for all backends
    void update_accel(uint8_t instance) __RAMFUNC__; /* front end */

    // support for updating filter at runtime
    uint16_t _last_accel_filter_hz;
    uint16_t _last_gyro_filter_hz;

    void set_gyro_orientation(uint8_t instance, enum Rotation rotation) {
        _imu._gyro_orientation[instance] = rotation;
    }

    void set_accel_orientation(uint8_t instance, enum Rotation rotation) {
        _imu._accel_orientation[instance] = rotation;
    }

    // increment clipping counted. Used by drivers that do decimation before supplying
    // samples to the frontend
    void increment_clip_count(uint8_t instance) {
        _imu._accel_clip_count[instance]++;
    }

    // should fast sampling be enabled on this IMU?
    bool enable_fast_sampling(uint8_t instance) {
        return (_imu._fast_sampling_mask & (1U<<instance)) != 0;
    }

    // if fast sampling is enabled, the rate to use in kHz
    uint8_t get_fast_sampling_rate() {
        return (1 << uint8_t(_imu._fast_sampling_rate));
    }

    // called by subclass when data is received from the sensor, thus
    // at the 'sensor rate'
    void _notify_new_accel_sensor_rate_sample(uint8_t instance, const Vector3f &accel) __RAMFUNC__;
    void _notify_new_gyro_sensor_rate_sample(uint8_t instance, const Vector3f &gyro) __RAMFUNC__;

    /*
      notify of a FIFO reset so we don't use bad data to update observed sensor rate
    */
    void notify_accel_fifo_reset(uint8_t instance) __RAMFUNC__;
    void notify_gyro_fifo_reset(uint8_t instance) __RAMFUNC__;
    
    // log an unexpected change in a register for an IMU
    void log_register_change(uint32_t bus_id, const AP_HAL::Device::checkreg &reg) __RAMFUNC__;

    // note that each backend is also expected to have a static detect()
    // function which instantiates an instance of the backend sensor
    // driver if the sensor is available

private:

    bool should_log_imu_raw() const ;
    void log_accel_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &accel) __RAMFUNC__;
    void log_gyro_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &gryo) __RAMFUNC__;

    // logging
    void Write_ACC(const uint8_t instance, const uint64_t sample_us, const Vector3f &accel) const __RAMFUNC__; // Write ACC data packet: raw accel data
    void Write_GYR(const uint8_t instance, const uint64_t sample_us, const Vector3f &gyro) const __RAMFUNC__;  // Write GYR data packet: raw gyro data

};
