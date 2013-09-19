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
    // @Description: Which type of IMU is installed (read-only)
    // @User: Standard
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
    AP_GROUPINFO("ACCSCAL",     1, AP_InertialSensor, _accel_scale,  0),

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
    AP_GROUPINFO("ACCOFFS",     2, AP_InertialSensor, _accel_offset, 0),

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
    AP_GROUPINFO("GYROFFS",     3, AP_InertialSensor, _gyro_offset,  0),

    // @Param: MPU6K_FILTER
    // @DisplayName: MPU6000 filter frequency
    // @Description: Filter frequency to ask the MPU6000 to apply to samples. This can be set to a lower value to try to cope with very high vibration levels in aircraft. The default value on ArduPlane, APMrover2 and ArduCopter is 20Hz. This option takes effect on the next reboot or gyro initialisation
    // @Units: Hz
    // @Values: 0:Default,5:5Hz,10:10Hz,20:20Hz,42:42Hz,98:98Hz
    // @User: Advanced
    AP_GROUPINFO("MPU6K_FILTER", 4, AP_InertialSensor, _mpu6000_filter,  0),

    AP_GROUPEND
};

AP_InertialSensor::AP_InertialSensor() :
    _accel(),
    _gyro()
{
    AP_Param::setup_object_defaults(this, var_info);        
}

void
AP_InertialSensor::init( Start_style style,
                         Sample_rate sample_rate)
{
    _product_id = _init_sensor(sample_rate);

    // check scaling
    Vector3f accel_scale = _accel_scale.get();
    if( accel_scale.x == 0 && accel_scale.y == 0 && accel_scale.z == 0 ) {
        accel_scale.x = accel_scale.y = accel_scale.z = 1.0;
        _accel_scale.set(accel_scale);
    }

    if (WARM_START != style) {
        // do cold-start calibration for gyro only
        _init_gyro();
    }
}

// save parameters to eeprom
void AP_InertialSensor::_save_parameters()
{
    _product_id.save();
    _accel_scale.save();
    _accel_offset.save();
    _gyro_offset.save();
}

void
AP_InertialSensor::init_gyro()
{
    _init_gyro();

    // save calibration
    _save_parameters();
}

void
AP_InertialSensor::_init_gyro()
{
    Vector3f last_average, best_avg;
    Vector3f ins_gyro;
    float best_diff = 0;

    // cold start
    hal.scheduler->delay(100);
    hal.console->printf_P(PSTR("Init Gyro"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // remove existing gyro offsets
    _gyro_offset = Vector3f(0,0,0);

    for(int8_t c = 0; c < 25; c++) {
        hal.scheduler->delay(20);

        update();
        ins_gyro = get_gyro();
    }

    // the strategy is to average 200 points over 1 second, then do it
    // again and see if the 2nd average is within a small margin of
    // the first

    last_average.zero();

    // we try to get a good calibration estimate for up to 10 seconds
    // if the gyros are stable, we should get it in 2 seconds
    for (int16_t j = 0; j <= 10; j++) {
        Vector3f gyro_sum, gyro_avg, gyro_diff;
        float diff_norm;
        uint8_t i;

        hal.console->printf_P(PSTR("*"));

        gyro_sum.zero();
        for (i=0; i<200; i++) {
            update();
            ins_gyro = get_gyro();
            gyro_sum += ins_gyro;
            hal.scheduler->delay(5);
        }
        gyro_avg = gyro_sum / i;

        gyro_diff = last_average - gyro_avg;
        diff_norm = gyro_diff.length();

        if (j == 0) {
            best_diff = diff_norm;
            best_avg = gyro_avg;
        } else if (gyro_diff.length() < ToRad(0.04)) {
            // we want the average to be within 0.1 bit, which is 0.04 degrees/s
            last_average = (gyro_avg * 0.5) + (last_average * 0.5);
            _gyro_offset = last_average;
            // stop flashing leds
            AP_Notify::flags.initialising = false;
            // all done
            return;
        } else if (diff_norm < best_diff) {
            best_diff = diff_norm;
            best_avg = (gyro_avg * 0.5) + (last_average * 0.5);
        }
        last_average = gyro_avg;
    }

    // stop flashing leds
    AP_Notify::flags.initialising = false;

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    hal.console->printf_P(PSTR("\ngyro did not converge: diff=%f dps\n"), ToDeg(best_diff));

    _gyro_offset = best_avg;
}


void
AP_InertialSensor::init_accel()
{
    _init_accel();

    // save calibration
    _save_parameters();
}

void
AP_InertialSensor::_init_accel()
{
    int8_t flashcount = 0;
    Vector3f ins_accel;
    Vector3f prev;
    Vector3f accel_offset;
    float total_change;
    float max_offset;

    // cold start
    hal.scheduler->delay(100);

    hal.console->printf_P(PSTR("Init Accel"));

    // flash leds to tell user to keep the IMU still
    AP_Notify::flags.initialising = true;

    // clear accelerometer offsets and scaling
    _accel_offset = Vector3f(0,0,0);
    _accel_scale = Vector3f(1,1,1);

    // initialise accel offsets to a large value the first time
    // this will force us to calibrate accels at least twice
    accel_offset = Vector3f(500, 500, 500);

    // loop until we calculate acceptable offsets
    do {
        // get latest accelerometer values
        update();
        ins_accel = get_accel();

        // store old offsets
        prev = accel_offset;

        // get new offsets
        accel_offset = ins_accel;

        // We take some readings...
        for(int8_t i = 0; i < 50; i++) {

            hal.scheduler->delay(20);
            update();
            ins_accel = get_accel();

            // low pass filter the offsets
            accel_offset = accel_offset * 0.9 + ins_accel * 0.1;

            // display some output to the user
            if(flashcount >= 10) {
                hal.console->printf_P(PSTR("*"));
                flashcount = 0;
            }
            flashcount++;
        }

        // null gravity from the Z accel
        accel_offset.z += GRAVITY_MSS;

        total_change = fabsf(prev.x - accel_offset.x) + fabsf(prev.y - accel_offset.y) + fabsf(prev.z - accel_offset.z);
        max_offset = (accel_offset.x > accel_offset.y) ? accel_offset.x : accel_offset.y;
        max_offset = (max_offset > accel_offset.z) ? max_offset : accel_offset.z;

        hal.scheduler->delay(500);
    } while (  total_change > AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE || max_offset > AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET);

    // set the global accel offsets
    _accel_offset = accel_offset;

    // stop flashing the leds
    AP_Notify::flags.initialising = false;

    hal.console->printf_P(PSTR(" "));

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
    Vector3f samples[6];
    Vector3f new_offsets;
    Vector3f new_scaling;
    Vector3f orig_offset;
    Vector3f orig_scale;

    // backup original offsets and scaling
    orig_offset = _accel_offset.get();
    orig_scale = _accel_scale.get();

    // clear accelerometer offsets and scaling
    _accel_offset = Vector3f(0,0,0);
    _accel_scale = Vector3f(1,1,1);

    // capture data from 6 positions
    for (int8_t i=0; i<6; i++) {
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
                PSTR("Place APM %S and press any key.\n"), msg);

        // wait for user input
        interact->blocking_read();

        // clear out any existing samples from ins
        update();

        // wait until we have 32 samples
        while( num_samples_available() < 32 * SAMPLE_UNIT ) {
            hal.scheduler->delay(10);
        }

        // read samples from ins
        update();

        // capture sample
        samples[i] = get_accel();
    }

    // run the calibration routine
    bool success = _calibrate_accel(samples, new_offsets, new_scaling);

    interact->printf_P(PSTR("Offsets: %.2f %.2f %.2f\n"),
                       new_offsets.x, new_offsets.y, new_offsets.z);
    interact->printf_P(PSTR("Scaling: %.2f %.2f %.2f\n"),
                       new_scaling.x, new_scaling.y, new_scaling.z);

    if (success) {
        interact->printf_P(PSTR("Calibration successful\n"));

        // set and save calibration
        _accel_offset.set(new_offsets);
        _accel_scale.set(new_scaling);
        _save_parameters();

        // calculate the trims as well and pass back to caller
        _calculate_trim(samples[0], trim_roll, trim_pitch);

        return true;
    }

    interact->printf_P(PSTR("Calibration FAILED\n"));
    // restore original scaling and offsets
    _accel_offset.set(orig_offset);
    _accel_scale.set(orig_scale);
    return false;
}

/// calibrated - returns true if the accelerometers have been calibrated
/// @note this should not be called while flying because it reads from the eeprom which can be slow
bool AP_InertialSensor::calibrated()
{
    return _accel_offset.load();
}

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
    Vector3f accel_scale = _accel_scale.get();
    Vector3f accel_offsets = _accel_offset.get();
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

