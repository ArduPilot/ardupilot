/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

#include "../AP_Math/AP_Math.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"

#define GRAVITY 9.80665
// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0

/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 */
class AP_InertialSensor
{
public:
    AP_InertialSensor();

    enum Start_style {
        COLD_START = 0,
        WARM_START
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
    virtual void        init( Start_style           style,
                              void                  (*delay_cb)(unsigned long t),
                              void                  (*flash_leds_cb)(bool on),
                              AP_PeriodicProcess *  scheduler );

    /// Perform cold startup initialisation for just the accelerometers.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work.
    ///
    virtual void        init_accel(void (*callback)(unsigned long t), void (*flash_leds_cb)(bool on));

    // perform accelerometer calibration including providing user instructions and feedback
    virtual void        calibrate_accel(void (*callback)(unsigned long t), void (*flash_leds_cb)(bool on));

    /// Perform cold-start initialisation for just the gyros.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work
    ///
    virtual void        init_gyro(void (*callback)(unsigned long t), void (*flash_leds_cb)(bool on));

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    Vector3f            get_gyro(void) { return _gyro; }

    // set gyro offsets in radians/sec
    Vector3f            get_gyro_offsets(void) { return _gyro_offset; }
    void                set_gyro_offsets(Vector3f offsets) { _gyro_offset.set(offsets); }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    Vector3f            get_accel(void) { return _accel; }

    // get accel offsets in m/s/s
    Vector3f            get_accel_offsets() { return _accel_offset; }
    void                set_accel_offsets(Vector3f offsets) { _accel_offset.set(offsets); }

    // get accel scale
    Vector3f            get_accel_scale() { return _accel_scale; }

    // sensor specific init to be overwritten by descendant classes
    // To-Do: should be combined with init above?
    virtual uint16_t        _init( AP_PeriodicProcess * scheduler ) = 0;

    /* Update the sensor data, so that getters are nonblocking.
     * Returns a bool of whether data was updated or not.
     */
    virtual bool            update() = 0;

    // check if the sensors have new data
    virtual bool            new_data_available(void) = 0;

    /* Getters for individual gyro axes.
     * Gyros have correct coordinate frame and units (degrees per second).
     */
    virtual float           gx() = 0;
    virtual float           gy() = 0;
    virtual float           gz() = 0;

    /* Getters for individual accel axes.
     * Accels have correct coordinate frame ( flat level ax, ay = 0; az = -9.81)
     * and units (meters per second squared).
     */
    virtual float           ax() = 0;
    virtual float           ay() = 0;
    virtual float           az() = 0;

    /* Temperature, in degrees celsius, of the gyro. */
    virtual float           temperature() = 0;

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    virtual float           get_delta_time() { return (float)get_delta_time_micros() * 1.0e-6; }
    virtual uint32_t        get_delta_time_micros() = 0;

    // get_last_sample_time_micros returns the time in microseconds that the last sample was taken
    //virtual uint32_t        get_last_sample_time_micros() = 0;

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    virtual float           get_gyro_drift_rate(void) = 0;

    // get number of samples read from the sensors
    virtual uint16_t        num_samples_available() = 0;

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // no-save implementations of accel and gyro initialisation routines
    virtual void            _init_accel(void    (*delay_cb)(unsigned long t),
                                        void    (*flash_leds_cb)(bool on) = NULL);
    virtual void            _init_gyro(void (*delay_cb)(unsigned long t),
                                       void (*flash_leds_cb)(bool on) = NULL);

    // _calibrate_accel - perform low level accel calibration
    virtual bool            _calibrate_accel(Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale );
    virtual void            _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    virtual void            _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    virtual void            _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);

    // load parameters from eeprom
    void                    load_parameters();

    // save parameters to eeprom
    void                    save_parameters();

    // Most recent accelerometer reading obtained by ::update
    Vector3f                _accel;

    // Most recent gyro reading obtained by ::update
    Vector3f                _gyro;

    // product id
    AP_Int16                _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f             _accel_scale;
    AP_Vector3f             _accel_offset;
    AP_Vector3f             _gyro_offset;
};

#include "AP_InertialSensor_Oilpan.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_Stub.h"

#endif // __AP_INERTIAL_SENSOR_H__
