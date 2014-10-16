/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Notify.h>

extern const AP_HAL::HAL& hal;


#define SAMPLE_UNIT 1

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::var_info[] PROGMEM = {
    // @Param: PRODUCT_ID
    // @DisplayName: IMU Product ID
    // @Description: Which type of IMU is installed (read-only). 
    // @User: Advanced
    // @Values: 0:Unknown,1:APM1-1280,2:APM1-2560,88:APM2,3:SITL,4:PX4v1,5:PX4v2,256:Flymaple,257:Linux
    AP_GROUPINFO("PRODUCT_ID",  0, AP_InertialSensor, _product_id,   0),

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
    AP_GROUPINFO("ACCSCAL",     1, AP_InertialSensor, _accel_scale[0],  0),

    // @Param: ACCOFFS_X
    // @DisplayName: Accelerometer offsets of X axis
    // @Description: Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced

    // @Param: ACCOFFS_Y
    // @DisplayName: Accelerometer offsets of Y axis
    // @Description: Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced

    // @Param: ACCOFFS_Z
    // @DisplayName: Accelerometer offsets of Z axis
    // @Description: Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced
    AP_GROUPINFO("ACCOFFS",     2, AP_InertialSensor, _accel_offset[0], 0),

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

    // @Param: MPU6K_FILTER
    // @DisplayName: MPU6000 filter frequency
    // @Description: Filter frequency to ask the MPU6000 to apply to samples. This can be set to a lower value to try to cope with very high vibration levels in aircraft. The default value on ArduPlane, APMrover2 and ArduCopter is 20Hz. This option takes effect on the next reboot or gyro initialisation
    // @Units: Hz
    // @Values: 0:Default,5:5Hz,10:10Hz,20:20Hz,42:42Hz,98:98Hz
    // @User: Advanced
    AP_GROUPINFO("MPU6K_FILTER", 4, AP_InertialSensor, _mpu6000_filter,  0),

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
    AP_GROUPINFO("ACC2SCAL",    5, AP_InertialSensor, _accel_scale[1],   0),

    // @Param: ACC2OFFS_X
    // @DisplayName: Accelerometer2 offsets of X axis
    // @Description: Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced

    // @Param: ACC2OFFS_Y
    // @DisplayName: Accelerometer2 offsets of Y axis
    // @Description: Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced

    // @Param: ACC2OFFS_Z
    // @DisplayName: Accelerometer2 offsets of Z axis
    // @Description: Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced
    AP_GROUPINFO("ACC2OFFS",    6, AP_InertialSensor, _accel_offset[1],  0),

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
    AP_GROUPINFO("ACC3SCAL",    8, AP_InertialSensor, _accel_scale[2],   0),

    // @Param: ACC3OFFS_X
    // @DisplayName: Accelerometer3 offsets of X axis
    // @Description: Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced

    // @Param: ACC3OFFS_Y
    // @DisplayName: Accelerometer3 offsets of Y axis
    // @Description: Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced

    // @Param: ACC3OFFS_Z
    // @DisplayName: Accelerometer3 offsets of Z axis
    // @Description: Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
    // @Units: m/s/s
    // @Range: -300 300
    // @User: Advanced
    AP_GROUPINFO("ACC3OFFS",    9, AP_InertialSensor, _accel_offset[2],  0),

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

    AP_GROUPEND
};

AP_InertialSensor::AP_InertialSensor() :
    _gyro_count(0),
    _accel_count(0),
    _accel(),
    _gyro(),
    _board_orientation(ROTATION_NONE),
    _hil_mode(false)
{
    AP_Param::setup_object_defaults(this, var_info);        
    for (uint8_t i=0; i<INS_MAX_BACKENDS; i++) {
        _backends[i] = NULL;
    }
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
    if (_gyro_count == 0 && _accel_count == 0) {
        // detect available backends. Only called once
        _detect_backends(sample_rate);
    }

    _product_id = 0; // FIX

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
    _base_delta_time = 1.0e-6f * _sample_period_usec;
    _delta_time = 0;
    _have_sample = false;
}


/*
  detect available backends for this board
 */
void 
AP_InertialSensor::_detect_backends(Sample_rate sample_rate)
{
#if HAL_INS_DEFAULT == HAL_INS_HIL
    _backends[0] = AP_InertialSensor_HIL::detect(*this, sample_rate);
#elif HAL_INS_DEFAULT == HAL_INS_MPU6000
    _backends[0] = AP_InertialSensor_MPU6000::detect(*this, sample_rate);
#elif HAL_INS_DEFAULT == HAL_INS_PX4 || HAL_INS_DEFAULT == HAL_INS_VRBRAIN
    _backends[0] = AP_InertialSensor_PX4::detect(*this, sample_rate);
#elif HAL_INS_DEFAULT == HAL_INS_OILPAN
    _backends[0] = AP_InertialSensor_Oilpan::detect(*this, sample_rate);
#elif HAL_INS_DEFAULT == HAL_INS_MPU9250
    _backends[0] = AP_InertialSensor_MPU9250::detect(*this, sample_rate);
#elif HAL_INS_DEFAULT == HAL_INS_FLYMAPLE
    _backends[0] = AP_InertialSensor_Flymaple::detect(*this, sample_rate);
#else
    #error Unrecognised HAL_INS_TYPE setting
#endif
    if (_backends[0] == NULL ||
        _gyro_count == 0 ||
        _accel_count == 0) {
        hal.scheduler->panic(PSTR("No INS backends available"));
    }

    // set the product ID to the ID of the first backend
    _product_id.set(_backends[0]->product_id());
}

void
AP_InertialSensor::init_accel()
{
    _init_accel();

    // save calibration
    _save_parameters();
}

#if !defined( __AVR_ATmega1280__ )
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

    for (uint8_t k=0; k<num_accels; k++) {
        // backup original offsets and scaling
        orig_offset[k] = _accel_offset[k].get();
        orig_scale[k]  = _accel_scale[k].get();

        // clear accelerometer offsets and scaling
        _accel_offset[k] = Vector3f(0,0,0);
        _accel_scale[k] = Vector3f(1,1,1);
    }

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

        // clear out any existing samples from ins
        update();

        // average 32 samples
        for (uint8_t k=0; k<num_accels; k++) {
            samples[k][i] = Vector3f();
        }
        uint8_t num_samples = 0;
        while (num_samples < 32) {
            wait_for_sample();
            // read samples from ins
            update();
            // capture sample
            for (uint8_t k=0; k<num_accels; k++) {
                samples[k][i] += get_accel(k);
            }
            hal.scheduler->delay(10);
            num_samples++;
        }
        for (uint8_t k=0; k<num_accels; k++) {
            samples[k][i] /= num_samples;
        }
    }

    // run the calibration routine
    for (uint8_t k=0; k<num_accels; k++) {
        bool success = _calibrate_accel(samples[k], new_offsets[k], new_scaling[k]);

        interact->printf_P(PSTR("Offsets[%u]: %.2f %.2f %.2f\n"),
                           (unsigned)k,
                           new_offsets[k].x, new_offsets[k].y, new_offsets[k].z);
        interact->printf_P(PSTR("Scaling[%u]: %.2f %.2f %.2f\n"),
                           (unsigned)k,
                           new_scaling[k].x, new_scaling[k].y, new_scaling[k].z);
        if (success) num_ok++;
    }

    if (num_ok == num_accels) {
        interact->printf_P(PSTR("Calibration successful\n"));

        for (uint8_t k=0; k<num_accels; k++) {
            // set and save calibration
            _accel_offset[k].set(new_offsets[k]);
            _accel_scale[k].set(new_scaling[k]);
        }
        _save_parameters();

        // calculate the trims as well from primary accels and pass back to caller
        _calculate_trim(samples[0][0], trim_roll, trim_pitch);

        return true;
    }

failed:
    interact->printf_P(PSTR("Calibration FAILED\n"));
    // restore original scaling and offsets
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k].set(orig_offset[k]);
        _accel_scale[k].set(orig_scale[k]);
    }
    return false;
}
#endif

/// calibrated - returns true if the accelerometers have been calibrated
/// @note this should not be called while flying because it reads from the eeprom which can be slow
bool AP_InertialSensor::calibrated()
{
    // check each accelerometer has offsets saved
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!_accel_offset[i].load()) {
            return false;
        }
    }
    // if we got this far the accelerometers must have been calibrated
    return true;
}

void
AP_InertialSensor::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_parameters();
}

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

void
AP_InertialSensor::_init_accel()
{
    uint8_t num_accels = min(get_accel_count(), INS_MAX_INSTANCES);
    uint8_t flashcount = 0;
    Vector3f prev[INS_MAX_INSTANCES];
    Vector3f accel_offset[INS_MAX_INSTANCES];
    float total_change[INS_MAX_INSTANCES];
    float max_offset[INS_MAX_INSTANCES];

    memset(max_offset, 0, sizeof(max_offset));
    memset(total_change, 0, sizeof(total_change));

    // cold start
    hal.scheduler->delay(100);

    hal.console->print_P(PSTR("Init Accel"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // clear accelerometer offsets and scaling
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k] = Vector3f(0,0,0);
        _accel_scale[k] = Vector3f(1,1,1);

        // initialise accel offsets to a large value the first time
        // this will force us to calibrate accels at least twice
        accel_offset[k] = Vector3f(500, 500, 500);
    }

    // loop until we calculate acceptable offsets
    while (true) {
        // get latest accelerometer values
        update();

        for (uint8_t k=0; k<num_accels; k++) {
            // store old offsets
            prev[k] = accel_offset[k];

            // get new offsets
            accel_offset[k] = get_accel(k);
        }

        // We take some readings...
        for(int8_t i = 0; i < 50; i++) {

            hal.scheduler->delay(20);
            update();

            // low pass filter the offsets
            for (uint8_t k=0; k<num_accels; k++) {
                accel_offset[k] = accel_offset[k] * 0.9f + get_accel(k) * 0.1f;
            }

            // display some output to the user
            if(flashcount >= 10) {
                hal.console->print_P(PSTR("*"));
                flashcount = 0;
            }
            flashcount++;
        }

        for (uint8_t k=0; k<num_accels; k++) {
            // null gravity from the Z accel
            accel_offset[k].z += GRAVITY_MSS;

            total_change[k] = 
                fabsf(prev[k].x - accel_offset[k].x) + 
                fabsf(prev[k].y - accel_offset[k].y) + 
                fabsf(prev[k].z - accel_offset[k].z);
            max_offset[k] = (accel_offset[k].x > accel_offset[k].y) ? accel_offset[k].x : accel_offset[k].y;
            max_offset[k] = (max_offset[k] > accel_offset[k].z) ? max_offset[k] : accel_offset[k].z;
        }

        uint8_t num_converged = 0;
        for (uint8_t k=0; k<num_accels; k++) {
            if (total_change[k] <= AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE && 
                max_offset[k] <= AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET) {
                num_converged++;
            }
        }

        if (num_converged == num_accels) break;

        hal.scheduler->delay(500);
    }

    // set the global accel offsets
    for (uint8_t k=0; k<num_accels; k++) {
        _accel_offset[k] = accel_offset[k];
    }

    // stop flashing the leds
    AP_Notify::flags.initialising = false;

    hal.console->print_P(PSTR(" "));

}

void
AP_InertialSensor::_init_gyro()
{
    uint8_t num_gyros = min(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[INS_MAX_INSTANCES], best_avg[INS_MAX_INSTANCES];
    float best_diff[INS_MAX_INSTANCES];
    bool converged[INS_MAX_INSTANCES];

    // cold start
    hal.console->print_P(PSTR("Init Gyro"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // remove existing gyro offsets
    for (uint8_t k=0; k<num_gyros; k++) {
        _gyro_offset[k] = Vector3f(0,0,0);
        best_diff[k] = 0;
        last_average[k].zero();
        converged[k] = false;
        _gyro_cal_ok[k] = true; // default calibration ok flag to true
    }

    for(int8_t c = 0; c < 5; c++) {
        hal.scheduler->delay(5);
        update();
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    uint8_t num_converged = 0;

    // we try to get a good calibration estimate for up to 10 seconds
    // if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 20 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum[INS_MAX_INSTANCES], gyro_avg[INS_MAX_INSTANCES], gyro_diff[INS_MAX_INSTANCES];
        float diff_norm[INS_MAX_INSTANCES];
        uint8_t i;

        memset(diff_norm, 0, sizeof(diff_norm));

        hal.console->print_P(PSTR("*"));

        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_sum[k].zero();
        }
        for (i=0; i<50; i++) {
            update();
            for (uint8_t k=0; k<num_gyros; k++) {
                gyro_sum[k] += get_gyro(k);
            }
            hal.scheduler->delay(5);
        }
        for (uint8_t k=0; k<num_gyros; k++) {
            gyro_avg[k] = gyro_sum[k] / i;
            gyro_diff[k] = last_average[k] - gyro_avg[k];
            diff_norm[k] = gyro_diff[k].length();
        }

        for (uint8_t k=0; k<num_gyros; k++) {
            if (converged[k]) continue;
            if (j == 0) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = gyro_avg[k];
            } else if (gyro_diff[k].length() < ToRad(0.1f)) {
                // we want the average to be within 0.1 bit, which is 0.04 degrees/s
                last_average[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
                _gyro_offset[k] = last_average[k];
                converged[k] = true;
                num_converged++;
            } else if (diff_norm[k] < best_diff[k]) {
                best_diff[k] = diff_norm[k];
                best_avg[k] = (gyro_avg[k] * 0.5f) + (last_average[k] * 0.5f);
            }
            last_average[k] = gyro_avg[k];
        }
    }

    // stop flashing leds
    AP_Notify::flags.initialising = false;

    if (num_converged == num_gyros) {
        // all OK
        return;
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    hal.console->println();
    for (uint8_t k=0; k<num_gyros; k++) {
        if (!converged[k]) {
            hal.console->printf_P(PSTR("gyro[%u] did not converge: diff=%f dps\n"),
                                  (unsigned)k, ToDeg(best_diff[k]));
            _gyro_offset[k] = best_avg[k];
            // flag calibration as failed for this gyro
            _gyro_cal_ok[k] = false;
        }
    }
}

#if !defined( __AVR_ATmega1280__ )
// _calibrate_model - perform low level accel calibration
// accel_sample are accelerometer samples collected in 6 different positions
// accel_offsets are output from the calibration routine
// accel_scale are output from the calibration routine
// returns true if successful
bool AP_InertialSensor::_calibrate_accel( Vector3f accel_sample[6],
                                          Vector3f& accel_offsets, Vector3f& accel_scale )
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
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
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if( accel_offsets.is_nan() || fabsf(accel_offsets.x) > 3.5f || fabsf(accel_offsets.y) > 3.5f || fabsf(accel_offsets.z) > 3.5f ) {
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
    float residual = 1.0;
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
            if( mu != 0.0f ) {
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

// _calculate_trim  - calculates the x and y trim angles (in radians) given a raw accel sample (i.e. no scaling or offsets applied) taken when the vehicle was level
void AP_InertialSensor::_calculate_trim(Vector3f accel_sample, float& trim_roll, float& trim_pitch)
{
    // scale sample and apply offsets
    Vector3f accel_scale = _accel_scale[0].get();
    Vector3f accel_offsets = _accel_offset[0].get();
    Vector3f scaled_accels_x( accel_sample.x * accel_scale.x - accel_offsets.x,
                              0,
                              accel_sample.z * accel_scale.z - accel_offsets.z );
    Vector3f scaled_accels_y( 0,
                              accel_sample.y * accel_scale.y - accel_offsets.y,
                              accel_sample.z * accel_scale.z - accel_offsets.z );

    // calculate x and y axis angle (i.e. roll and pitch angles)
    Vector3f vertical = Vector3f(0,0,-1);
    trim_roll = scaled_accels_y.angle(vertical);
    trim_pitch = scaled_accels_x.angle(vertical);

    // angle call doesn't return the sign so take care of it here
    if( scaled_accels_y.y > 0 ) {
        trim_roll = -trim_roll;
    }
    if( scaled_accels_x.x < 0 ) {
        trim_pitch = -trim_pitch;
    }
}

#endif // __AVR_ATmega1280__

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
        for (int8_t i=0; i<INS_MAX_BACKENDS; i++) {
            if (_backends[i] != NULL) {
                _backends[i]->update();
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

    if (_last_sample_usec == 0 && _delta_time <= 0) {
        // this is the first call to wait_for_sample()
        _last_sample_usec = now;
        _delta_time = _base_delta_time;
    }

    // see how many samples worth of time have elapsed
    uint16_t sample_count = 0;
    while (now - _last_sample_usec > _sample_period_usec) {
        _last_sample_usec += _sample_period_usec;
        sample_count++;
        if (sample_count == 1000) {
            // we've gone a very long time between samples, and gyro
            // integration times beyond this don't really make
            // sense. Just reset the timing
            _last_sample_usec = now;
            _delta_time = sample_count * _base_delta_time;
            goto check_sample;
        }
    }

    if (sample_count == 0) {
        // this is the normal case, and it means we took less than
        // _sample_period_usec to process the last sample. We're
        // on-time. So we can just sleep for the right number of
        // microseconds to get us to the next sample.
        _last_sample_usec += _sample_period_usec;
        uint32_t wait_usec = _last_sample_usec - now;
        if (wait_usec > 100 && wait_usec <= _sample_period_usec) {
            // only delay if the time to wait is significant.
            hal.scheduler->delay_microseconds(wait_usec);
        }

        // we set the _delta_time to the base time, ignoring small
        // timing fluctuations. 
        _delta_time = _base_delta_time;
    } else {
        // we have had some sort of delay and have missed our time
        // slot. We want to run this sample immediately, and need to
        // adjust _delta_time to reflect the delay we've had
        _delta_time = _base_delta_time*sample_count + 1.0e-6f*(now - _last_sample_usec);

        // reset the time base to the current time
        _last_sample_usec = now;
    }

check_sample:
    if (!_hil_mode) {
        // but also wait for at least one backend to have a sample of both
        // accel and gyro. This normally completes immediately.
        bool gyro_available = false;
        bool accel_available = false;
        while (!gyro_available || !accel_available) {
            for (uint8_t i=0; i<INS_MAX_BACKENDS; i++) {
                if (_backends[i] != NULL) {
                    gyro_available |= _backends[i]->gyro_sample_available();
                    accel_available |= _backends[i]->accel_sample_available();
                }
            }
            if (!gyro_available || !accel_available) {
                hal.scheduler->delay_microseconds(100);
            }
        }
    }

    _have_sample = true;
}

/*
  support for setting accel and gyro vectors, for use by HIL
 */
void AP_InertialSensor::set_accel(uint8_t instance, const Vector3f &accel)
{
    if (instance < INS_MAX_INSTANCES) {
        _accel[instance] = accel;
    }
}

void AP_InertialSensor::set_gyro(uint8_t instance, const Vector3f &gyro)
{
    if (instance < INS_MAX_INSTANCES) {
        _gyro[instance] = gyro;
    }
}

