/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
#define AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS        (15.5f*GRAVITY_MSS) // accelerometer values over 15.5G are recorded as a clipping error
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ     5.0f    // accel vibration floor filter hz
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ           2.0f    // accel vibration filter hz

/**
   maximum number of INS instances available on this platform. If more
   than 1 then redundent sensors may be available
 */
#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
#define INS_MAX_INSTANCES 3
#define INS_MAX_BACKENDS  6
#define INS_VIBRATION_CHECK 1
#define INS_VIBRATION_CHECK_INSTANCES 2
#else
#define INS_MAX_INSTANCES 1
#define INS_MAX_BACKENDS  1
#define INS_VIBRATION_CHECK 0
#endif


#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_InertialSensor_UserInteract.h"
#include <Filter/LowPassFilter.h>

class AP_InertialSensor_Backend;
class AuxiliaryBus;

/*
  forward declare DataFlash class. We can't include DataFlash.h
  because of mutual dependencies
 */
class DataFlash_Class;

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
    static AP_InertialSensor *get_instance();

    // the rate that updates will be available to the application
    enum Sample_rate {
        RATE_50HZ  = 50,
        RATE_100HZ = 100,
        RATE_200HZ = 200,
        RATE_400HZ = 400
    };

    enum Gyro_Calibration_Timing {
        GYRO_CAL_NEVER = 0,
        GYRO_CAL_STARTUP_ONLY = 1,
        GYRO_CAL_STARTUP_AND_FIRST_BOOT = 2
    };

    /// Perform startup initialisation.
    ///
    /// Called to initialise the state of the IMU.
    ///
    /// Gyros will be calibrated unless INS_GYRO_CAL is zero
    ///
    /// @param style	The initialisation startup style.
    ///
    void init(Sample_rate sample_rate);

    /// Register a new gyro/accel driver, allocating an instance
    /// number
    uint8_t register_gyro(void);
    uint8_t register_accel(void);

    // perform accelerometer calibration including providing user instructions
    // and feedback
    bool calibrate_accel(AP_InertialSensor_UserInteract *interact,
                         float& trim_roll,
                         float& trim_pitch);
    bool calibrate_trim(float &trim_roll, float &trim_pitch);

    /// calibrating - returns true if the gyros or accels are currently being calibrated
    bool calibrating() const { return _calibrating; }

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

    // set gyro offsets in radians/sec
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset[i]; }
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_primary_gyro); }

    //get delta angle if available
    bool get_delta_angle(uint8_t i, Vector3f &delta_angle) const;
    bool get_delta_angle(Vector3f &delta_angle) const { return get_delta_angle(_primary_gyro, delta_angle); }

    //get delta velocity if available
    bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity) const;
    bool get_delta_velocity(Vector3f &delta_velocity) const { return get_delta_velocity(_primary_accel, delta_velocity); }

    float get_delta_velocity_dt(uint8_t i) const;
    float get_delta_velocity_dt() const { return get_delta_velocity_dt(_primary_accel); }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(_primary_accel); }

    uint32_t get_gyro_error_count(uint8_t i) const { return _gyro_error_count[i]; }
    uint32_t get_accel_error_count(uint8_t i) const { return _accel_error_count[i]; }

    // multi-device interface
    bool get_gyro_health(uint8_t instance) const { return (instance<_gyro_count) ? _gyro_healthy[instance] : false; }
    bool get_gyro_health(void) const { return get_gyro_health(_primary_gyro); }
    bool get_gyro_health_all(void) const;
    uint8_t get_gyro_count(void) const { return _gyro_count; }
    bool gyro_calibrated_ok(uint8_t instance) const { return _gyro_cal_ok[instance]; }
    bool gyro_calibrated_ok_all() const;
    bool use_gyro(uint8_t instance) const;
    Gyro_Calibration_Timing gyro_calibration_timing() { return (Gyro_Calibration_Timing)_gyro_cal_timing.get(); }

    bool get_accel_health(uint8_t instance) const { return (instance<_accel_count) ? _accel_healthy[instance] : false; }
    bool get_accel_health(void) const { return get_accel_health(_primary_accel); }
    bool get_accel_health_all(void) const;
    uint8_t get_accel_count(void) const { return _accel_count; };
    bool accel_calibrated_ok_all() const;
    bool use_accel(uint8_t instance) const;

    // get accel offsets in m/s/s
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset[i]; }
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(_primary_accel); }

    // get accel scale
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale[i]; }
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(_primary_accel); }

    // return the temperature if supported. Zero is returned if no
    // temperature is available
    float get_temperature(uint8_t instance) const { return _temperature[instance]; }

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

    // return the selected sample rate
    Sample_rate get_sample_rate(void) const { return _sample_rate; }

    uint16_t error_count(void) const { return 0; }
    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_primary_accel(void) const { return _primary_accel; }
    uint8_t get_primary_gyro(void) const { return _primary_gyro; }

    // enable HIL mode
    void set_hil_mode(void) { _hil_mode = true; }

    // get the gyro filter rate in Hz
    uint8_t get_gyro_filter_hz(void) const { return _gyro_filter_cutoff; }

    // get the accel filter rate in Hz
    uint8_t get_accel_filter_hz(void) const { return _accel_filter_cutoff; }

    // pass in a pointer to DataFlash for raw data logging
    void set_dataflash(DataFlash_Class *dataflash) { _dataflash = dataflash; }

    // enable/disable raw gyro/accel logging
    void set_raw_logging(bool enable) { _log_raw_data = enable; }

#if INS_VIBRATION_CHECK
    // calculate vibration levels and check for accelerometer clipping (called by a backends)
    void calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt);

    // retrieve latest calculated vibration levels
    Vector3f get_vibration_levels() const { return get_vibration_levels(_primary_accel); }
    Vector3f get_vibration_levels(uint8_t instance) const;

    // retrieve and clear accelerometer clipping count
    uint32_t get_accel_clip_count(uint8_t instance) const;
#endif

    // check for vibration movement. True when all axis show nearly zero movement
    bool is_still();

    /*
      HIL set functions. The minimum for HIL is set_accel() and
      set_gyro(). The others are option for higher fidelity log
      playback
     */
    void set_accel(uint8_t instance, const Vector3f &accel);
    void set_gyro(uint8_t instance, const Vector3f &gyro);
    void set_delta_time(float delta_time);
    void set_delta_velocity(uint8_t instance, float deltavt, const Vector3f &deltav);
    void set_delta_angle(uint8_t instance, const Vector3f &deltaa);

    AuxiliaryBus *get_auxiliar_bus(int16_t backend_id);

private:

    // load backend drivers
    void _add_backend(AP_InertialSensor_Backend *backend);
    void _detect_backends(void);
    void _start_backends();
    AP_InertialSensor_Backend *_find_backend(int16_t backend_id);

    // gyro initialisation
    void _init_gyro();

    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    // _calibrate_accel - perform low level accel calibration
    bool _calibrate_accel(const Vector3f accel_sample[6],
                          Vector3f& accel_offsets,
                          Vector3f& accel_scale,
                          float max_abs_offsets,
                          enum Rotation rotation);
    bool _check_sample_range(const Vector3f accel_sample[6], enum Rotation rotation, 
                             AP_InertialSensor_UserInteract* interact);
    void _calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
    void _calibrate_reset_matrices(float dS[6], float JS[6][6]);
    void _calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
    bool _calculate_trim(const Vector3f &accel_sample, float& trim_roll, float& trim_pitch);

    // save parameters to eeprom
    void  _save_parameters();

    // backend objects
    AP_InertialSensor_Backend *_backends[INS_MAX_BACKENDS];

    // number of gyros and accel drivers. Note that most backends
    // provide both accel and gyro data, so will increment both
    // counters on initialisation
    uint8_t _gyro_count;
    uint8_t _accel_count;
    uint8_t _backend_count;

    // the selected sample rate
    Sample_rate _sample_rate;
    
    // Most recent accelerometer reading
    Vector3f _accel[INS_MAX_INSTANCES];
    Vector3f _delta_velocity[INS_MAX_INSTANCES];
    float _delta_velocity_dt[INS_MAX_INSTANCES];
    bool _delta_velocity_valid[INS_MAX_INSTANCES];

    // Most recent gyro reading
    Vector3f _gyro[INS_MAX_INSTANCES];
    Vector3f _delta_angle[INS_MAX_INSTANCES];
    bool _delta_angle_valid[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f _gyro_offset[INS_MAX_INSTANCES];

    // accelerometer max absolute offsets to be used for calibration
    float _accel_max_abs_offsets[INS_MAX_INSTANCES];

    // accelerometer sample rate in units of Hz
    uint32_t _accel_sample_rates[INS_MAX_INSTANCES];

    // temperatures for an instance if available
    float _temperature[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8     _accel_filter_cutoff;
    AP_Int8     _gyro_filter_cutoff;
    AP_Int8     _gyro_cal_timing;

    // use for attitude, velocity, position estimates
    AP_Int8     _use[INS_MAX_INSTANCES];

    // board orientation from AHRS
    enum Rotation _board_orientation;

    // calibrated_ok flags
    bool _gyro_cal_ok[INS_MAX_INSTANCES];

    // primary accel and gyro
    uint8_t _primary_gyro;
    uint8_t _primary_accel;

    // has wait_for_sample() found a sample?
    bool _have_sample:1;

    // are we in HIL mode?
    bool _hil_mode:1;

    // are gyros or accels currently being calibrated
    bool _calibrating:1;

    // should we log raw accel/gyro data?
    bool _log_raw_data:1;

    bool _backends_detected:1;

    // the delta time in seconds for the last sample
    float _delta_time;

    // last time a wait_for_sample() returned a sample
    uint32_t _last_sample_usec;

    // target time for next wait_for_sample() return
    uint32_t _next_sample_usec;
    
    // time between samples in microseconds
    uint32_t _sample_period_usec;

    // health of gyros and accels
    bool _gyro_healthy[INS_MAX_INSTANCES];
    bool _accel_healthy[INS_MAX_INSTANCES];

    uint32_t _accel_error_count[INS_MAX_INSTANCES];
    uint32_t _gyro_error_count[INS_MAX_INSTANCES];

#if INS_VIBRATION_CHECK
    // vibration and clipping
    uint32_t _accel_clip_count[INS_MAX_INSTANCES];
    LowPassFilterVector3f _accel_vibe_floor_filter[INS_VIBRATION_CHECK_INSTANCES];
    LowPassFilterVector3f _accel_vibe_filter[INS_VIBRATION_CHECK_INSTANCES];

    // threshold for detecting stillness
    AP_Float _still_threshold;
#endif

    /*
      state for HIL support
     */
    struct {
        float delta_time;
    } _hil {};

    DataFlash_Class *_dataflash;

    static AP_InertialSensor *_s_instance;
};

#include "AP_InertialSensor_Backend.h"
#include "AP_InertialSensor_MPU6000.h"
#include "AP_InertialSensor_PX4.h"
#include "AP_InertialSensor_Oilpan.h"
#include "AP_InertialSensor_MPU9250.h"
#include "AP_InertialSensor_L3G4200D.h"
#include "AP_InertialSensor_Flymaple.h"
#include "AP_InertialSensor_MPU9150.h"
#include "AP_InertialSensor_LSM9DS0.h"
#include "AP_InertialSensor_HIL.h"
#include "AP_InertialSensor_UserInteract_Stream.h"
#include "AP_InertialSensor_UserInteract_MAVLink.h"

#endif // __AP_INERTIAL_SENSOR_H__
