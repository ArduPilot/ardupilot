// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	IMU.h
/// @brief	Abstract class defining the interface to a real or virtual
///         Inertial Measurement Unit.

#ifndef IMU_h
#define IMU_h

#include "../AP_Math/AP_Math.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include <inttypes.h>

class IMU
{

public:
    /// Constructor
    IMU();

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

    /// Perform cold-start initialisation for just the gyros.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work
    ///
    virtual void        init_gyro(void (*callback)(unsigned long t), void (*flash_leds_cb)(bool on));

    /// Give the IMU some cycles to perform/fetch an update from its
    /// sensors.
    ///
    /// @returns	True if some state was updated.
    ///
    virtual bool        update(void);

    // true if new data is available from the sensors
    virtual bool        new_data_available(void);

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    Vector3f        get_gyro(void) {
        return _gyro;
    }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    Vector3f        get_accel(void) {
        return _accel;
    }


    /// return the number of seconds that the last update represents
    ///
    /// @returns	number of seconds
    ///
    float        get_delta_time(void) {
        return _sample_time * 1.0e-6;
    }

    /// return the maximum gyro drift rate in radians/s/s. This
    /// depends on what gyro chips are being used
    virtual float        get_gyro_drift_rate(void);

    /// A count of bad sensor readings
    ///
    /// @todo This should be renamed, as there's no guarantee that sensors
    ///       are using ADCs, etc.
    ///
    uint8_t                 adc_constraints;

    virtual float           gx(void);
    virtual float           gy(void);
    virtual float           gz(void);
    virtual float           ax(void);
    virtual float           ay(void);
    virtual float           az(void);
    virtual void            ax(const float v);
    virtual void            ay(const float v);
    virtual void            az(const float v);

    static const struct AP_Param::GroupInfo        var_info[];

protected:

    AP_Vector6f         _sensor_cal;    ///< Calibrated sensor offsets

    /// Most recent accelerometer reading obtained by ::update
    Vector3f            _accel;

    /// Most recent gyro reading obtained by ::update
    Vector3f            _gyro;

    /// number of microseconds that the accel and gyro values
    /// were sampled over
    uint32_t        _sample_time;

    AP_Int16        _product_id;        // this is the product id returned from the INS init
};

#endif
