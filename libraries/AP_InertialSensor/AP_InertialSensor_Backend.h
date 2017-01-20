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

#include "AP_InertialSensor.h"

class AuxiliaryBus;
class DataFlash_Class;

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
    virtual bool update() = 0;

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
        DEVTYPE_GYR_MPU6000  = 0x21,
        DEVTYPE_GYR_L3GD20   = 0x22,
        DEVTYPE_GYR_MPU9250  = 0x24
    };
        
protected:
    // access to frontend
    AP_InertialSensor &_imu;

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;

    void _rotate_and_correct_accel(uint8_t instance, Vector3f &accel);
    void _rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro);

    // rotate gyro vector, offset and publish
    void _publish_gyro(uint8_t instance, const Vector3f &gyro);

    // this should be called every time a new gyro raw sample is available -
    // be it published or not
    // the sample is raw in the sense that it's not filtered yet, but it must
    // be rotated and corrected (_rotate_and_correct_gyro)
    void _notify_new_gyro_raw_sample(uint8_t instance, const Vector3f &accel, uint64_t sample_us=0);

    // rotate accel vector, scale, offset and publish
    void _publish_accel(uint8_t instance, const Vector3f &accel);

    // this should be called every time a new accel raw sample is available -
    // be it published or not
    // the sample is raw in the sense that it's not filtered yet, but it must
    // be rotated and corrected (_rotate_and_correct_accel)
    void _notify_new_accel_raw_sample(uint8_t instance, const Vector3f &accel, uint64_t sample_us=0, bool fsync_set=false);

    // set accelerometer max absolute offset for calibration
    void _set_accel_max_abs_offset(uint8_t instance, float offset);

    // get accelerometer raw sample rate
    uint32_t _accel_raw_sample_rate(uint8_t instance) const {
        return _imu._accel_raw_sample_rates[instance];
    }

    // get gyroscope raw sample rate
    uint32_t _gyro_raw_sample_rate(uint8_t instance) const {
        return _imu._gyro_raw_sample_rates[instance];
    }

    // publish a temperature value
    void _publish_temperature(uint8_t instance, float temperature);

    // set accelerometer error_count
    void _set_accel_error_count(uint8_t instance, uint32_t error_count);

    // set gyro error_count
    void _set_gyro_error_count(uint8_t instance, uint32_t error_count);

    // increment accelerometer error_count
    void _inc_accel_error_count(uint8_t instance);

    // increment gyro error_count
    void _inc_gyro_error_count(uint8_t instance);
    
    // backend unique identifier or -1 if backend doesn't identify itself
    int16_t _id = -1;

    // return the default filter frequency in Hz for the sample rate
    uint8_t _accel_filter_cutoff(void) const { return _imu._accel_filter_cutoff; }

    // return the default filter frequency in Hz for the sample rate
    uint8_t _gyro_filter_cutoff(void) const { return _imu._gyro_filter_cutoff; }

    // return the requested sample rate in Hz
    uint16_t get_sample_rate_hz(void) const;

    // access to frontend dataflash
    DataFlash_Class *get_dataflash(void) const {
        return _imu._log_raw_data? _imu._dataflash : nullptr;
    }

    // common gyro update function for all backends
    void update_gyro(uint8_t instance);

    // common accel update function for all backends
    void update_accel(uint8_t instance);

    // support for updating filter at runtime
    int8_t _last_accel_filter_hz[INS_MAX_INSTANCES];
    int8_t _last_gyro_filter_hz[INS_MAX_INSTANCES];

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
    
    // note that each backend is also expected to have a static detect()
    // function which instantiates an instance of the backend sensor
    // driver if the sensor is available
};
