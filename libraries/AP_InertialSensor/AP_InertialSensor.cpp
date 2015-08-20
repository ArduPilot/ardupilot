/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Progmem/AP_Progmem.h>
#include "AP_InertialSensor.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>

/*
  enable TIMING_DEBUG to track down scheduling issues with the main
  loop. Output is on the debug console
 */
#define TIMING_DEBUG 0

#if TIMING_DEBUG
#include <stdio.h>
#define timing_printf(fmt, args...)      do { printf("[timing] " fmt, ##args); } while(0)
#else
#define timing_printf(fmt, args...)
#endif

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#elif APM_BUILD_TYPE(APM_BUILD_APMrover2)
#define DEFAULT_GYRO_FILTER  10
#define DEFAULT_ACCEL_FILTER 10
#else
#define DEFAULT_GYRO_FILTER  20
#define DEFAULT_ACCEL_FILTER 20
#endif

#define SAMPLE_UNIT 1

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::var_info[] PROGMEM = {
    // @Param: PRODUCT_ID
    // @DisplayName: IMU Product ID
    // @Description: Which type of IMU is installed (read-only). 
    // @User: Advanced
    // @Values: 0:Unknown,1:APM1-1280,2:APM1-2560,88:APM2,3:SITL,4:PX4v1,5:PX4v2,256:Flymaple,257:Linux
    AP_GROUPINFO("PRODUCT_ID",  0, AP_InertialSensor, _product_id,   0),

    /*
      The following parameter indexes and reserved from previous use
      as accel offsets and scaling from before the 16g change in the
      PX4 backend:

      ACCSCAL : 1
      ACCOFFS : 2
      MPU6K_FILTER: 4
      ACC2SCAL : 5
      ACC2OFFS : 6
      ACC3SCAL : 8
      ACC3OFFS : 9
      CALSENSFRAME : 11
     */

    // @Param: GYROFFS_X
    // @DisplayName: Gyro offsets of X axis
    // @Description: Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYROFFS_Y
    // @DisplayName: Gyro offsets of Y axis
    // @Description: Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYROFFS_Z
    // @DisplayName: Gyro offsets of Z axis
    // @Description: Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("GYROFFS",     3, AP_InertialSensor, _gyro_offset[0],  0),

#if INS_MAX_INSTANCES > 1
    // @Param: GYR2OFFS_X
    // @DisplayName: Gyro2 offsets of X axis
    // @Description: Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR2OFFS_Y
    // @DisplayName: Gyro2 offsets of Y axis
    // @Description: Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR2OFFS_Z
    // @DisplayName: Gyro2 offsets of Z axis
    // @Description: Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("GYR2OFFS",    7, AP_InertialSensor, _gyro_offset[1],   0),
#endif

#if INS_MAX_INSTANCES > 2
    // @Param: GYR3OFFS_X
    // @DisplayName: Gyro3 offsets of X axis
    // @Description: Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR3OFFS_Y
    // @DisplayName: Gyro3 offsets of Y axis
    // @Description: Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR3OFFS_Z
    // @DisplayName: Gyro3 offsets of Z axis
    // @Description: Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("GYR3OFFS",   10, AP_InertialSensor, _gyro_offset[2],   0),
#endif

    // @Param: ACCSCAL_X
    // @DisplayName: Accelerometer scaling of X axis
    // @Description: Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACCSCAL_Y
    // @DisplayName: Accelerometer scaling of Y axis
    // @Description: Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACCSCAL_Z
    // @DisplayName: Accelerometer scaling of Z axis
    // @Description: Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ACCSCAL",     12, AP_InertialSensor, _accel_scale[0],  0),

    // @Param: ACCOFFS_X
    // @DisplayName: Accelerometer offsets of X axis
    // @Description: Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACCOFFS_Y
    // @DisplayName: Accelerometer offsets of Y axis
    // @Description: Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACCOFFS_Z
    // @DisplayName: Accelerometer offsets of Z axis
    // @Description: Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    AP_GROUPINFO("ACCOFFS",     13, AP_InertialSensor, _accel_offset[0], 0),

#if INS_MAX_INSTANCES > 1
    // @Param: ACC2SCAL_X
    // @DisplayName: Accelerometer2 scaling of X axis
    // @Description: Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC2SCAL_Y
    // @DisplayName: Accelerometer2 scaling of Y axis
    // @Description: Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC2SCAL_Z
    // @DisplayName: Accelerometer2 scaling of Z axis
    // @Description: Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ACC2SCAL",    14, AP_InertialSensor, _accel_scale[1],   0),

    // @Param: ACC2OFFS_X
    // @DisplayName: Accelerometer2 offsets of X axis
    // @Description: Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC2OFFS_Y
    // @DisplayName: Accelerometer2 offsets of Y axis
    // @Description: Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC2OFFS_Z
    // @DisplayName: Accelerometer2 offsets of Z axis
    // @Description: Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    AP_GROUPINFO("ACC2OFFS",    15, AP_InertialSensor, _accel_offset[1],  0),
#endif

#if INS_MAX_INSTANCES > 2
    // @Param: ACC3SCAL_X
    // @DisplayName: Accelerometer3 scaling of X axis
    // @Description: Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC3SCAL_Y
    // @DisplayName: Accelerometer3 scaling of Y axis
    // @Description: Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced

    // @Param: ACC3SCAL_Z
    // @DisplayName: Accelerometer3 scaling of Z axis
    // @Description: Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine
    // @Range: 0.8 1.2
    // @User: Advanced
    AP_GROUPINFO("ACC3SCAL",    16, AP_InertialSensor, _accel_scale[2],   0),

    // @Param: ACC3OFFS_X
    // @DisplayName: Accelerometer3 offsets of X axis
    // @Description: Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC3OFFS_Y
    // @DisplayName: Accelerometer3 offsets of Y axis
    // @Description: Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced

    // @Param: ACC3OFFS_Z
    // @DisplayName: Accelerometer3 offsets of Z axis
    // @Description: Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -3.5 3.5
    // @User: Advanced
    AP_GROUPINFO("ACC3OFFS",    17, AP_InertialSensor, _accel_offset[2],  0),
#endif

    // @Param: GYRO_FILTER
    // @DisplayName: Gyro filter cutoff frequency
    // @Description: Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
    // @Units: Hz
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("GYRO_FILTER", 18, AP_InertialSensor, _gyro_filter_cutoff,  DEFAULT_GYRO_FILTER),

    // @Param: ACCEL_FILTER
    // @DisplayName: Accel filter cutoff frequency
    // @Description: Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
    // @Units: Hz
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("ACCEL_FILTER", 19, AP_InertialSensor, _accel_filter_cutoff,  DEFAULT_ACCEL_FILTER),

    // @Param: USE
    // @DisplayName: Use first IMU for attitude, velocity and position estimates
    // @Description: Use first IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE", 20, AP_InertialSensor, _use[0],  1),

#if INS_MAX_INSTANCES > 1
    // @Param: USE2
    // @DisplayName: Use second IMU for attitude, velocity and position estimates
    // @Description: Use second IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2", 21, AP_InertialSensor, _use[1],  1),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: USE3
    // @DisplayName: Use third IMU for attitude, velocity and position estimates
    // @Description: Use third IMU for attitude, velocity and position estimates
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3", 22, AP_InertialSensor, _use[2],  0),
#endif

    /*
      NOTE: parameter indexes have gaps above. When adding new
      parameters check for conflicts carefully
     */

    AP_GROUPEND
};

AP_InertialSensor::AP_InertialSensor() :
    _gyro_count(0),
    _accel_count(0),
    _backend_count(0),
    _accel(),
    _gyro(),
    _board_orientation(ROTATION_NONE),
    _primary_gyro(0),
    _primary_accel(0),
    _hil_mode(false),
    _calibrating(false),
    _log_raw_data(false),
    _dataflash(NULL)
{
    AP_Param::setup_object_defaults(this, var_info);        
    for (uint8_t i=0; i<INS_MAX_BACKENDS; i++) {
        _backends[i] = NULL;
    }
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_error_count[i] = 0;
        _gyro_error_count[i] = 0;
#if INS_VIBRATION_CHECK
        _accel_clip_count[i] = 0;
#endif

        _accel_max_abs_offsets[i] = 3.5f;
    }
#if INS_VIBRATION_CHECK
    for (uint8_t i=0; i<INS_VIBRATION_CHECK_INSTANCES; i++) {
        _accel_vibe_floor_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ);
        _accel_vibe_filter[i].set_cutoff_frequency(AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ);
    }
#endif
    memset(_delta_velocity_valid,0,sizeof(_delta_velocity_valid));
    memset(_delta_angle_valid,0,sizeof(_delta_angle_valid));
}


/*
  register a new gyro instance
 */
uint8_t AP_InertialSensor::register_gyro(void)
{
    if (_gyro_count == INS_MAX_INSTANCES) {
        hal.scheduler->panic(PSTR("Too many gyros"));
    }
    return _gyro_count++;
}

/*
  register a new accel instance
 */
uint8_t AP_InertialSensor::register_accel(void)
{
    if (_accel_count == INS_MAX_INSTANCES) {
        hal.scheduler->panic(PSTR("Too many accels"));
    }
    return _accel_count++;
}

void
AP_InertialSensor::init( Start_style style,
                         Sample_rate sample_rate)
{
    // remember the sample rate
    _sample_rate = sample_rate;

    if (_gyro_count == 0 && _accel_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }

    // initialise accel scale if need be. This is needed as we can't
    // give non-zero default values for vectors in AP_Param
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (_accel_scale[i].get().is_zero()) {
            _accel_scale[i].set(Vector3f(1,1,1));
        }
    }

    if (WARM_START != style) {
        // do cold-start calibration for gyro only
        _init_gyro();
    }

    switch (sample_rate) {
    case RATE_50HZ:
        _sample_period_usec = 20000;
        break;
    case RATE_100HZ:
        _sample_period_usec = 10000;
        break;
    case RATE_200HZ:
        _sample_period_usec = 5000;
        break;
    case RATE_400HZ:
    default:
        _sample_period_usec = 2500;
        break;
    }

    // establish the baseline time between samples
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;
}

void AP_InertialSensor::_add_backend(AP_InertialSensor_Backend *backend)
{
    if (!backend)
        return;
    if (_backend_count == INS_MAX_BACKENDS)
        hal.scheduler->panic(PSTR("Too many INS backends"));
    _backends[_backend_count++] = backend;
}

/*
  detect available backends for this board
 */
void 
AP_InertialSensor::_detect_backends(void)
{
    if (_hil_mode) {
        _add_backend(AP_InertialSensor_HIL::detect(*this));
        return;
    }
#if HAL_INS_DEFAULT == HAL_INS_HIL
    _add_backend(AP_InertialSensor_HIL::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_MPU60XX_SPI
    _add_backend(AP_InertialSensor_MPU6000::detect_spi(*this));
#elif HAL_INS_DEFAULT == HAL_INS_MPU60XX_I2C && HAL_INS_MPU60XX_I2C_BUS == 2
    _add_backend(AP_InertialSensor_MPU6000::detect_i2c(*this, hal.i2c2, HAL_INS_MPU60XX_I2C_ADDR));
#elif HAL_INS_DEFAULT == HAL_INS_PX4 || HAL_INS_DEFAULT == HAL_INS_VRBRAIN
    _add_backend(AP_InertialSensor_PX4::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_OILPAN
    _add_backend(AP_InertialSensor_Oilpan::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_MPU9250
    _add_backend(AP_InertialSensor_MPU9250::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_FLYMAPLE
    _add_backend(AP_InertialSensor_Flymaple::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_LSM9DS0
    _add_backend(AP_InertialSensor_LSM9DS0::detect(*this));
#elif HAL_INS_DEFAULT == HAL_INS_L3G4200D
    _add_backend(AP_InertialSensor_L3G4200D::detect(*this));
#else
    #error Unrecognised HAL_INS_TYPE setting
#endif

    if (_backend_count == 0 ||
        _gyro_count == 0 ||
        _accel_count == 0) {
        hal.scheduler->panic(PSTR("No INS backends available"));
    }

    // set the product ID to the ID of the first backend
    _product_id.set(_backends[0]->product_id());
}

/*
  _calculate_trim - calculates the x and y trim angles. The
  accel_sample must be correctly scaled, offset and oriented for the
  board 
*/
bool AP_InertialSensor::_calculate_trim(const Vector3f &accel_sample, float& trim_roll, float& trim_pitch)
{
    trim_pitch = atan2f(accel_sample.x, pythagorous2(accel_sample.y, accel_sample.z));
    trim_roll = atan2f(-accel_sample.y, -accel_sample.z);
    if (fabsf(trim_roll) > radians(10) || 
        fabsf(trim_pitch) > radians(10)) {
        hal.console->println_P(PSTR("trim over maximum of 10 degrees"));
        return false;
    }
    hal.console->printf_P(PSTR("Trim OK: roll=%.2f pitch=%.2f\n"),
                          (double)degrees(trim_roll),
                          (double)degrees(trim_pitch));
    return true;
}

// calibrate_accel - perform accelerometer calibration including providing user
// instructions and feedback Gauss-Newton accel calibration routines borrowed
// from Rolfe Schmidt blog post describing the method:
// http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
// original sketch available at
// http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
bool AP_InertialSensor::calibrate_accel(AP_InertialSensor_UserInteract* interact,
                                        float &trim_roll,
                                        float &trim_pitch)
{
    uint8_t num_accels = min(get_accel_count(), INS_MAX_INSTANCES);
    Vector3f samples[INS_MAX_INSTANCES][6];
    Vector3f new_offsets[INS_MAX_INSTANCES];
    Vector3f new_scaling[INS_MAX_INSTANCES];
    Vector3f orig_offset[INS_MAX_INSTANCES];
    Vector3f orig_scale[INS_MAX_INSTANCES];
    uint8_t num_ok = 0;

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return false;
    }

    _calibrating = true;

    /*
      we do the accel calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    for (uint8_t k=0; k<num_accels; k++) {
        // backup original offsets and scaling
        orig_offset[k] = _accel_offset[k].get();
        orig_scale[k]  = _accel_scale[k].get();

        // clear accelerometer offsets and scaling
        _accel_offset[k] = Vector3f(0,0,0);
        _accel_scale[k] = Vector3f(1,1,1);
    }

    memset(samples, 0, sizeof(samples));

    // capture data from 6 positions
    for (uint8_t i=0; i<6; i++) {
        const prog_char_t *msg;

        // display message to user
        switch ( i ) {
            case 0:
                msg = PSTR("level");
                break;
            case 1:
                msg = PSTR("on its LEFT side");
                break;
            case 2:
                msg = PSTR("on its RIGHT side");
                break;
            case 3:
                msg = PSTR("nose DOWN");
                break;
            case 4:
                msg = PSTR("nose UP");
                break;
            default:    // default added to avoid compiler warning
            case 5:
                msg = PSTR("on its BACK");
                break;
        }
        interact->printf_P(
                PSTR("Place vehicle %S and press any key.\n"), msg);

        // wait for user input
        if (!interact->blocking_read()) {
            //No need to use interact->printf_P for an error, blocking_read does this when it fails
            goto failed;
        }

        const uint8_t update_dt_milliseconds = (uint8_t)(1000.0f/get_sample_rate()+0.5f);

        // wait 100ms for ins filter to rise
        for (uint8_t k=0; k<100/update_dt_milliseconds; k++) {
            wait_for_sample();
            update();
            hal.scheduler->delay(update_dt_milliseconds);
        }

        uint32_t num_samples = 0;
        while (num_samples < 400/update_dt_milliseconds) {
            wait_for_sample();
            // read samples from ins
            update();
            // capture sample
            for (uint8_t k=0; k<num_accels; k++) {
                Vector3f samp;
                if(get_delta_velocity(k,samp) && _delta_velocity_dt[k] > 0) {
                    samp /= _delta_velocity_dt[k];
                } else {
                    samp = get_accel(k);
                }
                samples[k][i] += samp;
                if (!get_accel_health(k)) {
                    interact->printf_P(PSTR("accel[%u] not healthy"), (unsigned)k);
                    goto failed;
                }
            }
            hal.scheduler->delay(update_dt_milliseconds);
            num_samples++;
        }
        for (uint8_t k=0; k<num_accels; k++) {
            samples[k][i] /= num_samples;
        }
    }

    // run the calibration routine
    for (uint8_t k=0; k<num_accels; k++) {
        if (!_check_sample_range(samples[k], saved_orientation, interact)) {
            interact->printf_P(PSTR("Insufficient accel range"));
            continue;
        }

        bool success = _calibrate_accel(samples[k],
                                        new_offsets[k],
                                        new_scaling[k],
                                        _accel_max_abs_offsets[k],
                                        saved_orientation);

        interact->printf_P(PSTR("Offsets[%u]: %.2f %.2f %.2f\n"),
                            (unsigned)k,
                           (double)new_offsets[k].x, (double)new_offsets[k].y, (double)new_offsets[k].z);
        interact->printf_P(PSTR("Scaling[%u]: %.2f %.2f %.2f\n"),
                           (unsigned)k,
                           (double)new_scaling[k].x, (double)new_scaling[k].y, (double)new_scaling[k].z);
        if (success) num_ok++;
    }

    if (num_ok == num_accels) {
        interact->printf_P(PSTR("Calibration successful\n"));

        for (uint8_t k=0; k<num_accels; k++) {
            // set and save calibration
            _accel_offset[k].set(new_offsets[k]);
            _accel_scale[k].set(new_scaling[k]);
        }
        for (uint8_t k=num_accels; k<INS_MAX_INSTANCES; k++) {
            // clear unused accelerometer's scaling and offsets
            _accel_offset[k] = Vector3f(0,0,0);
            _accel_scale[k] = Vector3f(0,0,0);
        }
        _save_parameters();

        /*
          calculate the trims as well from primary accels 
          We use the original board rotation for this sample
        */
        Vector3f level_sample = samples[0][0];
        level_sample.x *= new_scaling[0].x;
        level_sample.y *= new_scaling[0].y;
        level_sample.z *= new_scaling[0].z;
        level_sample -= new_offsets[0];
        level_sample.rotate(saved_orientation);

        if (!_calculate_trim(level_sample, trim_roll, trim_pitch)) {
            goto failed;
        }

        _board_orientation = saved_orientation;

        _calibrating = false;
        return true;
    }

failed:
    interact->printf_P(PSTR("Calibration FAILED\n"));
    // restore original scaling and offsets
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k].set(orig_offset[k]);
        _accel_scale[k].set(orig_scale[k]);
    }
    _board_orientation = saved_orientation;
    _calibrating = false;
    return false;
}

void
AP_InertialSensor::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_parameters();
}

#if INS_VIBRATION_CHECK
// accelerometer clipping reporting
uint32_t AP_InertialSensor::get_accel_clip_count(uint8_t instance) const
{
    if (instance >= get_accel_count()) {
        return 0;
    }
    return _accel_clip_count[instance];
}
#endif

// get_gyro_health_all - return true if all gyros are healthy
bool AP_InertialSensor::get_gyro_health_all(void) const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!get_gyro_health(i)) {
            return false;
        }
    }
    // return true if we have at least one gyro
    return (get_gyro_count() > 0);
}

// gyro_calibration_ok_all - returns true if all gyros were calibrated successfully
bool AP_InertialSensor::gyro_calibrated_ok_all() const
{
    for (uint8_t i=0; i<get_gyro_count(); i++) {
        if (!gyro_calibrated_ok(i)) {
            return false;
        }
    }
    return (get_gyro_count() > 0);
}

// return true if gyro instance should be used (must be healthy and have it's use parameter set to 1)
bool AP_InertialSensor::use_gyro(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }

    return (get_gyro_health(instance) && _use[instance]);
}

// get_accel_health_all - return true if all accels are healthy
bool AP_InertialSensor::get_accel_health_all(void) const
{
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!get_accel_health(i)) {
            return false;
        }
    }
    // return true if we have at least one accel
    return (get_accel_count() > 0);
}


/*
  calculate the trim_roll and trim_pitch. This is used for redoing the
  trim without needing a full accel cal
 */
bool AP_InertialSensor::calibrate_trim(float &trim_roll, float &trim_pitch)
{
    Vector3f level_sample;

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return false;
    }

    _calibrating = true;

    const uint8_t update_dt_milliseconds = (uint8_t)(1000.0f/get_sample_rate()+0.5f);

    // wait 100ms for ins filter to rise
    for (uint8_t k=0; k<100/update_dt_milliseconds; k++) {
        wait_for_sample();
        update();
        hal.scheduler->delay(update_dt_milliseconds);
    }

    uint32_t num_samples = 0;
    while (num_samples < 400/update_dt_milliseconds) {
        wait_for_sample();
        // read samples from ins
        update();
        // capture sample
        Vector3f samp;
        samp = get_accel(0);
        level_sample += samp;
        if (!get_accel_health(0)) {
            goto failed;
        }
        hal.scheduler->delay(update_dt_milliseconds);
        num_samples++;
    }
    level_sample /= num_samples;

    if (!_calculate_trim(level_sample, trim_roll, trim_pitch)) {
        goto failed;
    }

    _calibrating = false;
    return true;

failed:
    _calibrating = false;
    return false;    
}

/*
  check if the accelerometers are calibrated in 3D and that current number of accels matched number when calibrated
 */
bool AP_InertialSensor::accel_calibrated_ok_all() const
{
    // check each accelerometer has offsets saved
    for (uint8_t i=0; i<get_accel_count(); i++) {
        // exactly 0.0 offset is extremely unlikely
        if (_accel_offset[i].get().is_zero()) {
            return false;
        }
        // exactly 1.0 scaling is extremely unlikely
        const Vector3f &scaling = _accel_scale[i].get();
        if (is_equal(scaling.x,1.0f) && is_equal(scaling.y,1.0f) && is_equal(scaling.z,1.0f)) {
            return false;
        }
        // zero scaling also indicates not calibrated
        if (_accel_scale[i].get().is_zero()) {
            return false;
        }
    }

    // check calibrated accels matches number of accels (no unused accels should have offsets or scaling)
    if (get_accel_count() < INS_MAX_INSTANCES) {
        for (uint8_t i=get_accel_count(); i<INS_MAX_INSTANCES; i++) {
            const Vector3f &scaling = _accel_scale[i].get();
            bool have_scaling = (!is_zero(scaling.x) && !is_equal(scaling.x,1.0f)) || (!is_zero(scaling.y) && !is_equal(scaling.y,1.0f)) || (!is_zero(scaling.z) && !is_equal(scaling.z,1.0f));
            bool have_offsets = !_accel_offset[i].get().is_zero();
            if (have_scaling || have_offsets) {
                return false;
            }
        }
    }

    // if we got this far the accelerometers must have been calibrated
    return true;
}

// return true if accel instance should be used (must be healthy and have it's use parameter set to 1)
bool AP_InertialSensor::use_accel(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }

    return (get_accel_health(instance) && _use[instance]);
}

void
AP_InertialSensor::_init_gyro()
{
    uint8_t num_gyros = min(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES];
    Vector3f new_gyro_offset[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];

    // exit immediately if calibration is already in progress
    if (_calibrating) {
        return;
    }

    // record we are calibrating
    _calibrating = true;

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // cold start
    hal.console->print_P(PSTR("Init Gyro"));

    /*
      we do the gyro calibration with no board rotation. This avoids
      having to rotate readings during the calibration
    */
    enum Rotation saved_orientation = _board_orientation;
    _board_orientation = ROTATION_NONE;

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        _gyro_offset[k].set(Vector3f());
        new_gyro_offset[k].zero();
        best_diff[k] = 0;
        last_average[k].zero();
        converged[k] = false;
    }

    for(int8_t c = 0; c < 5; c++) {
        hal.scheduler->delay(5);
        update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 30 seconds
    // if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 30*4 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum[INS_MAX_INSTANCES], gyro_avg[INS_MAX_INSTANCES], gyro_diff[INS_MAX_INSTANCES];
        Vector3f accel_start;
        float diff_norm[INS_MAX_INSTANCES];
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

        hal.console->print_P(PSTR("*"));

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_sum[k].zero();
        }
        accel_start = get_accel(0);
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_gyros; k++) {
                gyro_sum[k] += get_gyro(k);
            }
            hal.scheduler->delay(5);
        }

        Vector3f accel_diff = get_accel(0) - accel_start;
        if (accel_diff.length() > 0.2f) {
            // the accelerometers changed during the gyro sum. Skip
            // this sample. This copes with doing gyro cal on a
            // steadily moving platform. The value 0.2 corresponds
            // with around 5 degrees/second of rotation.
            continue;
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_avg[k] = gyro_sum[k] / i;
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            if (j == 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];
            } else if (gyro_diff[k].length() < ToRad(0.1f)) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                if (!converged[k] || last_average[k].length() < new_gyro_offset[k].length()) {
                    new_gyro_offset[k] = last_average[k];
                }
                if (!converged[k]) {
                    converged[k] = true;
                    num_converged++;
                }
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    hal.console->println();
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            hal.console->printf_P(PSTR("gyro[%u] did not converge: diff=%f dps\n"),
                                  (unsigned)k, (double)ToDeg(best_diff[k]));
            _gyro_offset[k] = best_avg[k];
            // flag calibration as failed for this gyro
            _gyro_cal_ok[k] = false;
        } else {
            _gyro_cal_ok[k] = true;
            _gyro_offset[k] = new_gyro_offset[k];
        }
    }

    // restore orientation
    _board_orientation = saved_orientation;

    // record calibration complete
    _calibrating = false;

    // stop flashing leds
    AP_Notify::flags.initialising = false;
}

/*
  check that the samples used for accel calibration have a sufficient
  range on each axis. The sphere fit in _calibrate_accel() can produce
  bad offsets and scaling factors if the range of input data is
  insufficient.

  We rotate each sample in the check to body frame to cope with 45
  board orientations which could result in smaller ranges. The sample
  inputs are in sensor frame
 */
bool AP_InertialSensor::_check_sample_range(const Vector3f accel_sample[6], enum Rotation rotation, 
                                            AP_InertialSensor_UserInteract* interact)
{
    // we want at least 12 m/s/s range on all axes. This should be
    // very easy to achieve, and guarantees the accels have been
    // exposed to a good range of data
    const float min_range = 12.0f;

    Vector3f min_sample, max_sample;

    // start with first sample
    min_sample = accel_sample[0];
    min_sample.rotate(rotation);
    max_sample = min_sample;

    for (uint8_t s=1; s<6; s++) {
        Vector3f sample = accel_sample[s];
        sample.rotate(rotation);
        for (uint8_t i=0; i<3; i++) {
            if (sample[i] < min_sample[i]) {
                min_sample[i] = sample[i];
            }
            if (sample[i] > max_sample[i]) {
                max_sample[i] = sample[i];
            }
        }
    }
    Vector3f range = max_sample - min_sample;
    interact->printf_P(PSTR("AccelRange: %.1f %.1f %.1f"),
            (double)range.x, (double)range.y, (double)range.z);
    bool ok = (range.x >= min_range && 
               range.y >= min_range && 
               range.z >= min_range);
    return ok;
}


// _calibrate_model - perform low level accel calibration
// accel_sample are accelerometer samples collected in 6 different positions
// accel_offsets are output from the calibration routine
// accel_scale are output from the calibration routine
// returns true if successful
bool AP_InertialSensor::_calibrate_accel(const Vector3f accel_sample[6],
                                         Vector3f& accel_offsets,
                                         Vector3f& accel_scale,
                                         float max_abs_offsets,
                                         enum Rotation rotation)
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001f;
    float change = 100.0f;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];
    bool success = true;

    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
    
    while( num_iterations < 20 && change > eps ) {
        num_iterations++;

        _calibrate_reset_matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;
            _calibrate_update_matrices(ds, JS, beta, data);
        }

        _calibrate_find_delta(ds, JS, delta);

        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);

        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }

    // copy results out
    accel_scale.x = beta[3] * GRAVITY_MSS;
    accel_scale.y = beta[4] * GRAVITY_MSS;
    accel_scale.z = beta[5] * GRAVITY_MSS;
    accel_offsets.x = beta[0] * accel_scale.x;
    accel_offsets.y = beta[1] * accel_scale.y;
    accel_offsets.z = beta[2] * accel_scale.z;

    // sanity check scale
    if( accel_scale.is_nan() || fabsf(accel_scale.x-1.0f) > 0.1f || fabsf(accel_scale.y-1.0f) > 0.1f || fabsf(accel_scale.z-1.0f) > 0.1f ) {
        success = false;
    }

    if (accel_offsets.is_nan() ||
        fabsf(accel_offsets.x) > max_abs_offsets ||
        fabsf(accel_offsets.y) > max_abs_offsets ||
        fabsf(accel_offsets.z) > max_abs_offsets) {
        success = false;
    }

    // return success or failure
    return success;
}

void AP_InertialSensor::_calibrate_update_matrices(float dS[6], float JS[6][6],
                                    float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0f;
    float jacobian[6];
    
    for( j=0; j<3; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    
    for( j=0; j<6; j++ ) {
        dS[j] += jacobian[j]*residual;
        for( k=0; k<6; k++ ) {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}


// _calibrate_reset_matrices - clears matrices
void AP_InertialSensor::_calibrate_reset_matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ) {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ) {
            JS[j][k] = 0.0f;
        }
    }
}

void AP_InertialSensor::_calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( !is_zero(mu) ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;
        
        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}

// save parameters to eeprom
void AP_InertialSensor::_save_parameters()
{
    _product_id.save();
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_scale[i].save();
        _accel_offset[i].save();
        _gyro_offset[i].save();
    }
}


/*
  update gyro and accel values from backends
 */
void AP_InertialSensor::update(void)
{
    // during initialisation update() may be called without
    // wait_for_sample(), and a wait is implied
    wait_for_sample();

    if (!_hil_mode) {
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            // mark sensors unhealthy and let update() in each backend
            // mark them healthy via _publish_gyro() and
            // _publish_accel()
            _gyro_healthy[i] = false;
            _accel_healthy[i] = false;
            _delta_velocity_valid[i] = false;
            _delta_angle_valid[i] = false;
        }
        for (uint8_t i=0; i<_backend_count; i++) {
            _backends[i]->update();
        }

        // adjust health status if a sensor has a non-zero error count
        // but another sensor doesn't. 
        bool have_zero_accel_error_count = false;
        bool have_zero_gyro_error_count = false;
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i] && _accel_error_count[i] == 0) {
                have_zero_accel_error_count = true;
            }
            if (_gyro_healthy[i] && _gyro_error_count[i] == 0) {
                have_zero_gyro_error_count = true;
            }
        }

        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_gyro_healthy[i] && _gyro_error_count[i] != 0 && have_zero_gyro_error_count) {
                // we prefer not to use a gyro that has had errors
                _gyro_healthy[i] = false;
            }
            if (_accel_healthy[i] && _accel_error_count[i] != 0 && have_zero_accel_error_count) {
                // we prefer not to use a accel that has had errors
                _accel_healthy[i] = false;
            }
        }

        // set primary to first healthy accel and gyro
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_gyro_healthy[i] && _use[i]) {
                _primary_gyro = i;
                break;
            }
        }
        for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
            if (_accel_healthy[i] && _use[i]) {
                _primary_accel = i;
                break;
            }
        }
    }

    _have_sample = false;
}

/*
  wait for a sample to be available. This is the function that
  determines the timing of the main loop in ardupilot. 

  Ideally this function would return at exactly the rate given by the
  sample_rate argument given to AP_InertialSensor::init(). 

  The key output of this function is _delta_time, which is the time
  over which the gyro and accel integration will happen for this
  sample. We want that to be a constant time if possible, but if
  delays occur we need to cope with them. The long term sum of
  _delta_time should be exactly equal to the wall clock elapsed time
 */
void AP_InertialSensor::wait_for_sample(void)
{
    if (_have_sample) {
        // the user has called wait_for_sample() again without
        // consuming the sample with update()
        return;
    }

    uint32_t now = hal.scheduler->micros();

    if (_next_sample_usec == 0 && _delta_time <= 0) {
        // this is the first call to wait_for_sample()
        _last_sample_usec = now - _sample_period_usec;
        _next_sample_usec = now + _sample_period_usec;
        goto check_sample;
    }

    // see how long it is till the next sample is due
    if (_next_sample_usec - now <=_sample_period_usec) {
        // we're ahead on time, schedule next sample at expected period
        uint32_t wait_usec = _next_sample_usec - now;
        hal.scheduler->delay_microseconds_boost(wait_usec);
        uint32_t now2 = hal.scheduler->micros();
        if (now2+100 < _next_sample_usec) {
            timing_printf("shortsleep %u\n", (unsigned)(_next_sample_usec-now2));
        }
        if (now2 > _next_sample_usec+400) {
            timing_printf("longsleep %u wait_usec=%u\n", 
                          (unsigned)(now2-_next_sample_usec),
                          (unsigned)wait_usec);
        }
        _next_sample_usec += _sample_period_usec;
    } else if (now - _next_sample_usec < _sample_period_usec/8) {
        // we've overshot, but only by a small amount, keep on
        // schedule with no delay
        timing_printf("overshoot1 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec += _sample_period_usec;
    } else {
        // we've overshot by a larger amount, re-zero scheduling with
        // no delay
        timing_printf("overshoot2 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec = now + _sample_period_usec;
    }

check_sample:
    if (!_hil_mode) {
        // we also wait for at least one backend to have a sample of both
        // accel and gyro. This normally completes immediately.
        bool gyro_available = false;
        bool accel_available = false;
        while (!gyro_available || !accel_available) {
            for (uint8_t i=0; i<_backend_count; i++) {
                gyro_available |= _backends[i]->gyro_sample_available();
                accel_available |= _backends[i]->accel_sample_available();
            }
            if (!gyro_available || !accel_available) {
                hal.scheduler->delay_microseconds(100);
            }
        }
    }

    now = hal.scheduler->micros();
    if (_hil_mode && _hil.delta_time > 0) {
        _delta_time = _hil.delta_time;
        _hil.delta_time = 0;
    } else {
        _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    }
    _last_sample_usec = now;

#if 0
    {
        static uint64_t delta_time_sum;
        static uint16_t counter;
        if (delta_time_sum == 0) {
            delta_time_sum = _sample_period_usec;
        }
        delta_time_sum += _delta_time * 1.0e6f;
        if (counter++ == 400) {
            counter = 0;
            hal.console->printf("now=%lu _delta_time_sum=%lu diff=%ld\n",
                                (unsigned long)now, 
                                (unsigned long)delta_time_sum,
                                (long)(now - delta_time_sum));
        }
    }
#endif

    _have_sample = true;
}


/*
  get delta angles
 */
bool AP_InertialSensor::get_delta_angle(uint8_t i, Vector3f &delta_angle) const 
{
    if (_delta_angle_valid[i]) {
        delta_angle = _delta_angle[i];
        return true;
    } else if (get_gyro_health(i)) {
        // provide delta angle from raw gyro, so we use the same code
        // at higher level
        delta_angle = get_gyro(i) * get_delta_time();
        return true;
    }
    return false;
}

/*
  get delta velocity if available
*/
bool AP_InertialSensor::get_delta_velocity(uint8_t i, Vector3f &delta_velocity) const
{
    if (_delta_velocity_valid[i]) {
        delta_velocity = _delta_velocity[i];
        return true;
    } else if (get_accel_health(i)) {
        delta_velocity = get_accel(i) * get_delta_time();
        return true;
    }
    return false;
}

/*
  return delta_time for the delta_velocity
 */
float AP_InertialSensor::get_delta_velocity_dt(uint8_t i) const
{
    if (_delta_velocity_valid[i]) {
        return _delta_velocity_dt[i];
    }
    return get_delta_time();
}


/*
  support for setting accel and gyro vectors, for use by HIL
 */
void AP_InertialSensor::set_accel(uint8_t instance, const Vector3f &accel)
{
    if (_accel_count == 0) {
        // we haven't initialised yet
        return;
    }
    if (instance < INS_MAX_INSTANCES) {
        _accel[instance] = accel;
        _accel_healthy[instance] = true;
        if (_accel_count <= instance) {
            _accel_count = instance+1;
        }
        if (!_accel_healthy[_primary_accel]) {
            _primary_accel = instance;
        }
    }
}

void AP_InertialSensor::set_gyro(uint8_t instance, const Vector3f &gyro)
{
    if (_gyro_count == 0) {
        // we haven't initialised yet
        return;
    }
    if (instance < INS_MAX_INSTANCES) {
        _gyro[instance] = gyro;
        _gyro_healthy[instance] = true;
        if (_gyro_count <= instance) {
            _gyro_count = instance+1;
            _gyro_cal_ok[instance] = true;
        }
        if (!_accel_healthy[_primary_accel]) {
            _primary_accel = instance;
        }
    }
}

/*
  set delta time for next ins.update()
 */
void AP_InertialSensor::set_delta_time(float delta_time)
{
    _hil.delta_time = delta_time;
}

/*
  set delta velocity for next update
 */
void AP_InertialSensor::set_delta_velocity(uint8_t instance, float deltavt, const Vector3f &deltav)
{
    if (instance < INS_MAX_INSTANCES) {
        _delta_velocity_valid[instance] = true;
        _delta_velocity[instance] = deltav;
        _delta_velocity_dt[instance] = deltavt;
    }
}

/*
  set delta angle for next update
 */
void AP_InertialSensor::set_delta_angle(uint8_t instance, const Vector3f &deltaa)
{
    if (instance < INS_MAX_INSTANCES) {
        _delta_angle_valid[instance] = true;
        _delta_angle[instance] = deltaa;
    }
}

#if INS_VIBRATION_CHECK
// calculate vibration levels and check for accelerometer clipping (called by a backends)
void AP_InertialSensor::calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt)
{
    // check for clipping
    if (fabsf(accel.x) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS ||
        fabsf(accel.y) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS ||
        fabsf(accel.z) > AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS) {
        _accel_clip_count[instance]++;
    }

    // calculate vibration levels
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        // filter accel at 5hz
        Vector3f accel_filt = _accel_vibe_floor_filter[instance].apply(accel, dt);

        // calc difference from this sample and 5hz filtered value, square and filter at 2hz
        Vector3f accel_diff = (accel - accel_filt);
        accel_diff.x *= accel_diff.x;
        accel_diff.y *= accel_diff.y;
        accel_diff.z *= accel_diff.z;
        _accel_vibe_filter[instance].apply(accel_diff, dt);
    }
}

// retrieve latest calculated vibration levels
Vector3f AP_InertialSensor::get_vibration_levels(uint8_t instance) const
{
    Vector3f vibe;
    if (instance < INS_VIBRATION_CHECK_INSTANCES) {
        vibe = _accel_vibe_filter[instance].get();
        vibe.x = safe_sqrt(vibe.x);
        vibe.y = safe_sqrt(vibe.y);
        vibe.z = safe_sqrt(vibe.z);
    }
    return vibe;
}
#endif
