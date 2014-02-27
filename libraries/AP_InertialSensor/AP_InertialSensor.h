/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define INS_MAX_INSTANCES 2
#else
#define INS_MAX_INSTANCES 1
#endif

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_InertialSensor_UserInteract.h"
/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_InertialSensor
{
public:
    AP_InertialSensor();

    enum Start_style {
        COLD_START = 0,
        WARM_START
    };

    // the rate that updates will be available to the application
    enum Sample_rate {
        RATE_50HZ,
        RATE_100HZ,
        RATE_200HZ,
        RATE_400HZ
    };

    /// Perform startup initialisation.
    ///
    /// Called to initialise the state of the IMU.
    ///
    /// For COLD_START, implementations using real sensors can assume
    /// that the airframe is stationary and nominally oriented.
    ///
    /// For WARM_START, no assumptions should be made about the
    /// orientation or motion of the airframe.  Calibration should be
    /// as for the previous COLD_START call.
    ///
    /// @param style	The initialisation startup style.
    ///
    virtual void init( Start_style style,
                       Sample_rate sample_rate);

    /// Perform cold startup initialisation for just the accelerometers.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work.
    ///
    virtual void init_accel();

#if !defined( __AVR_ATmega1280__ )
    // perform accelerometer calibration including providing user instructions
    // and feedback
    virtual bool calibrate_accel(AP_InertialSensor_UserInteract *interact,
                                 float& trim_roll,
                                 float& trim_pitch);
#endif

    /// calibrated - returns true if the accelerometers have been calibrated
    ///
    /// @note this should not be called while flying because it reads from the eeprom which can be slow
    ///
    bool calibrated();

    /// Perform cold-start initialisation for just the gyros.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work
    ///
    virtual void init_gyro(void);

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro(void) const { return get_gyro(_get_primary_gyro()); }
    virtual void       set_gyro(uint8_t instance, const Vector3f &gyro) {}

    // set gyro offsets in radians/sec
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset[i]; }
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_get_primary_gyro()); }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(get_primary_accel()); }
    virtual void       set_accel(uint8_t instance, const Vector3f &accel) {}

    // multi-device interface
    virtual bool get_gyro_health(uint8_t instance) const { return true; }
    bool get_gyro_health(void) const { return get_gyro_health(_get_primary_gyro()); }
    virtual uint8_t get_gyro_count(void) const { return 1; };

    virtual bool get_accel_health(uint8_t instance) const { return true; }
    bool get_accel_health(void) const { return get_accel_health(get_primary_accel()); }
    virtual uint8_t get_accel_count(void) const { return 1; };

    // get accel offsets in m/s/s
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset[i]; }
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(get_primary_accel()); }

    // get accel scale
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale[i]; }
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(get_primary_accel()); }

    /* Update the sensor data, so that getters are nonblocking.
     * Returns a bool of whether data was updated or not.
     */
    virtual bool update() = 0;

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    virtual float get_delta_time() const = 0;

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    virtual float get_gyro_drift_rate(void) = 0;

    // wait for a sample to be available, with timeout in milliseconds
    virtual bool wait_for_sample(uint16_t timeout_ms) = 0;

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    // override default filter frequency
    void set_default_filter(float filter_hz) {
        if (!_mpu6000_filter.load()) {
            _mpu6000_filter.set(filter_hz);
        }
    }

    virtual uint16_t error_count(void) const { return 0; }
    virtual bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    virtual uint8_t get_primary_accel(void) const { return 0; }

protected:

    virtual uint8_t _get_primary_gyro(void) const { return 0; }

    // sensor specific init to be overwritten by descendant classes
    virtual uint16_t        _init_sensor( Sample_rate sample_rate ) = 0;

    // no-save implementations of accel and gyro initialisation routines
    virtual void  _init_accel();

    virtual void _init_gyro();

#if !defined( __AVR_ATmega1280__ )
    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    // _calibrate_accel - perform low level accel calibration
    virtual bool            _calibrate_accel(Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale);
    virtual void            _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    virtual void            _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    virtual void            _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
    virtual void            _calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch);
#endif

    // save parameters to eeprom
    void  _save_parameters();

    // Most recent accelerometer reading obtained by ::update
    Vector3f _accel[INS_MAX_INSTANCES];

    // previous accelerometer reading obtained by ::update
    Vector3f _previous_accel[INS_MAX_INSTANCES];

    // Most recent gyro reading obtained by ::update
    Vector3f _gyro[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f             _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f             _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f             _gyro_offset[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8                 _mpu6000_filter;

    // board orientation from AHRS
    enum Rotation			_board_orientation;
};

#include "AP_InertialSensor_Oilpan.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_HIL.h"
#include "AP_InertialSensor_PX4.h"
#include "AP_InertialSensor_UserInteract_Stream.h"
#include "AP_InertialSensor_UserInteract_MAVLink.h"
#include "AP_InertialSensor_Flymaple.h"
#include "AP_InertialSensor_L3G4200D.h"

#endif // __AP_INERTIAL_SENSOR_H__
