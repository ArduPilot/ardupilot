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
#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
#define INS_MAX_INSTANCES 3
#else
#define INS_MAX_INSTANCES 1
#endif

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_InertialSensor_UserInteract.h"

class AP_InertialSensor_Backend;

/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_InertialSensor
{
    friend class AP_InertialSensor_Backend;

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
    void init( Start_style style,
               Sample_rate sample_rate);

    /// Perform cold startup initialisation for just the accelerometers.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work.
    ///
    void init_accel();


    /// Register a new gyro/accel driver, allocating an instance
    /// number
    uint8_t register_gyro(void);
    uint8_t register_accel(void);

#if !defined( __AVR_ATmega1280__ )
    // perform accelerometer calibration including providing user instructions
    // and feedback
    bool calibrate_accel(AP_InertialSensor_UserInteract *interact,
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
    void init_gyro(void);

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro(void) const { return get_gyro(_primary_gyro); }
    void               set_gyro(uint8_t instance, const Vector3f &gyro);

    // set gyro offsets in radians/sec
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset[i]; }
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_primary_gyro); }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(_primary_accel); }
    void               set_accel(uint8_t instance, const Vector3f &accel);

    // multi-device interface
    bool get_gyro_health(uint8_t instance) const { return true; }
    bool get_gyro_health(void) const { return get_gyro_health(_primary_gyro); }
    bool get_gyro_health_all(void) const;
    uint8_t get_gyro_count(void) const { return _gyro_count; }
    bool gyro_calibrated_ok(uint8_t instance) const { return _gyro_cal_ok[instance]; }
    bool gyro_calibrated_ok_all() const;

    bool get_accel_health(uint8_t instance) const { return true; }
    bool get_accel_health(void) const { return get_accel_health(_primary_accel); }
    bool get_accel_health_all(void) const;
    uint8_t get_accel_count(void) const { return _accel_count; };

    // get accel offsets in m/s/s
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset[i]; }
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(_primary_accel); }

    // get accel scale
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale[i]; }
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(_primary_accel); }

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    float get_delta_time() const { return _delta_time; }

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    float get_gyro_drift_rate(void) const { return ToRad(0.5f/60); }

    // update gyro and accel values from accumulated samples
    void update(void);

    // wait for a sample to be available
    void wait_for_sample(void);

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

    // get_filter - return filter in hz
    uint8_t get_filter() const { return _mpu6000_filter.get(); }

    uint16_t error_count(void) const { return 0; }
    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_primary_accel(void) const { return 0; }

private:

    // load backend drivers
    void _detect_backends(Sample_rate sample_rate);

    // accel and gyro initialisation
    void _init_accel();
    void _init_gyro();

#if !defined( __AVR_ATmega1280__ )
    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    // _calibrate_accel - perform low level accel calibration
    bool _calibrate_accel(Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale);
    void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    void _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
    void _calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch);
#endif

    // save parameters to eeprom
    void  _save_parameters();

    // backend objects
    AP_InertialSensor_Backend *_backends[INS_MAX_INSTANCES];

    // number of gyros and accel drivers. Note that most backends
    // provide both accel and gyro data, so will increment both
    // counters on initialisation
    uint8_t _gyro_count;
    uint8_t _accel_count;

    // Most recent accelerometer reading
    Vector3f _accel[INS_MAX_INSTANCES];

    // Most recent gyro reading
    Vector3f _gyro[INS_MAX_INSTANCES];

    // timestamp of latest gyro and accel readings
    uint32_t _last_gyro_sample_time_usec[INS_MAX_INSTANCES];
    uint32_t _last_accel_sample_time_usec[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f _gyro_offset[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8     _mpu6000_filter;

    // board orientation from AHRS
    enum Rotation _board_orientation;

    // calibrated_ok flags
    bool _gyro_cal_ok[INS_MAX_INSTANCES];

    // primary accel and gyro
    uint8_t _primary_gyro;
    uint8_t _primary_accel;

    // time between samples
    float _delta_time;
};

#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_HIL.h"
#include "AP_InertialSensor_UserInteract_Stream.h"
#include "AP_InertialSensor_UserInteract_MAVLink.h"

#endif // __AP_INERTIAL_SENSOR_H__
