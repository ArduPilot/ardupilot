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
    AP_GROUPINFO("ACC2SCAL",    5, AP_InertialSensor, _accel_scale[1],   0),
    AP_GROUPINFO("ACC2OFFS",    6, AP_InertialSensor, _accel_offset[1],  0),
    AP_GROUPINFO("GYR2OFFS",    7, AP_InertialSensor, _gyro_offset[1],   0),
#endif

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
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (_accel_scale[i].get().is_zero()) {
            _accel_scale[i].set(Vector3f(1,1,1));
        }
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
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        _accel_scale[i].save();
        _accel_offset[i].save();
        _gyro_offset[i].save();
    }
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
    uint8_t num_gyros = min(get_gyro_count(), INS_MAX_INSTANCES);
    Vector3f last_average[num_gyros], best_avg[num_gyros];
    float best_diff[num_gyros];
    bool converged[num_gyros];

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
        Vector3f gyro_sum[num_gyros], gyro_avg[num_gyros], gyro_diff[num_gyros];
        float diff_norm[num_gyros];
        uint8_t i;

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
        }
    }
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
    uint8_t num_accels = min(get_accel_count(), INS_MAX_INSTANCES);
    uint8_t flashcount = 0;
    Vector3f prev[num_accels];
    Vector3f accel_offset[num_accels];
    float total_change[num_accels];
    float max_offset[num_accels];

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
    Vector3f samples[num_accels][6];
    Vector3f new_offsets[num_accels];
    Vector3f new_scaling[num_accels];
    Vector3f orig_offset[num_accels];
    Vector3f orig_scale[num_accels];
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
        interact->blocking_read();

        // clear out any existing samples from ins
        update();

        // average 32 samples
        for (uint8_t k=0; k<num_accels; k++) {
            samples[k][i] = Vector3f();
        }
        uint8_t num_samples = 0;
        while (num_samples < 32) {
            if (!wait_for_sample(1000)) {
                interact->printf_P(PSTR("Failed to get INS sample\n"));
                goto failed;
            }
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

/// calibrated - returns true if the accelerometers have been calibrated
/// @note this should not be called while flying because it reads from the eeprom which can be slow
bool AP_InertialSensor::calibrated()
{
    return _accel_offset[0].load();
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

