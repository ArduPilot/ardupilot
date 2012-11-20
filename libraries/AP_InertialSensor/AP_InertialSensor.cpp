/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <FastSerial.h>

#include "AP_InertialSensor.h"

#include <SPI.h>
#include <AP_Common.h>

#define FLASH_LEDS(on) do { if (flash_leds_cb != NULL) flash_leds_cb(on); } while (0)

#define SAMPLE_UNIT 1

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::var_info[] PROGMEM = {
    AP_GROUPINFO("PRODUCT_ID",  0, AP_InertialSensor, _product_id,   0),
    AP_GROUPINFO("ACCSCAL",     1, AP_InertialSensor, _accel_scale,  0),
    AP_GROUPINFO("ACCOFFS",     2, AP_InertialSensor, _accel_offset, 0),
    AP_GROUPINFO("GYROFFS",     3, AP_InertialSensor, _gyro_offset,  0),
    AP_GROUPEND
};

AP_InertialSensor::AP_InertialSensor()
{
}

void
AP_InertialSensor::init( Start_style style,
                         void (*delay_cb)(unsigned long t),
                         void (*flash_leds_cb)(bool on),
                         AP_PeriodicProcess *  scheduler )
{
    _product_id = _init_sensor(scheduler);

    // check scaling
    Vector3f accel_scale = _accel_scale.get();
    if( accel_scale.x == 0 && accel_scale.y == 0 && accel_scale.z == 0 ) {
        accel_scale.x = accel_scale.y = accel_scale.z = 1.0;
        _accel_scale.set(accel_scale);
    }

    if (WARM_START != style) {
        // do cold-start calibration for gyro only
        _init_gyro(delay_cb, flash_leds_cb);
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
AP_InertialSensor::init_gyro(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
    _init_gyro(delay_cb, flash_leds_cb);

    // save calibration
    _save_parameters();
}

void
AP_InertialSensor::_init_gyro(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
    Vector3f last_average, best_avg;
    Vector3f ins_gyro;
    float best_diff = 0;

    // cold start
    delay_cb(100);
    Serial.printf_P(PSTR("Init Gyro"));

    // remove existing gyro offsets
    _gyro_offset = Vector3f(0,0,0);

    for(int16_t c = 0; c < 25; c++) {
        // Mostly we are just flashing the LED's here
        // to tell the user to keep the IMU still
        FLASH_LEDS(true);
        delay_cb(20);

        update();
        ins_gyro = get_gyro();

        FLASH_LEDS(false);
        delay_cb(20);
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

        Serial.printf_P(PSTR("*"));

        gyro_sum.zero();
        for (i=0; i<200; i++) {
            update();
            ins_gyro = get_gyro();
            gyro_sum += ins_gyro;
            if (i % 40 == 20) {
                FLASH_LEDS(true);
            } else if (i % 40 == 0) {
                FLASH_LEDS(false);
            }
            delay_cb(5);
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

            // all done
            return;
        } else if (diff_norm < best_diff) {
            best_diff = diff_norm;
            best_avg = (gyro_avg * 0.5) + (last_average * 0.5);
        }
        last_average = gyro_avg;
    }

    // we've kept the user waiting long enough - use the best pair we
    // found so far
    Serial.printf_P(PSTR("\ngyro did not converge: diff=%f dps\n"), ToDeg(best_diff));

    _gyro_offset = best_avg;
}


void
AP_InertialSensor::init_accel(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
    _init_accel(delay_cb, flash_leds_cb);

    // save calibration
    _save_parameters();
}

void
AP_InertialSensor::_init_accel(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
    int8_t flashcount = 0;
    Vector3f ins_accel;
    Vector3f prev;
    Vector3f accel_offset;
    float total_change;
    float max_offset;

    // cold start
    delay_cb(100);

    Serial.printf_P(PSTR("Init Accel"));

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

            delay_cb(20);
            update();
            ins_accel = get_accel();

            // low pass filter the offsets
            accel_offset = accel_offset * 0.9 + ins_accel * 0.1;

            // display some output to the user
            if(flashcount == 5) {
                Serial.printf_P(PSTR("*"));
                FLASH_LEDS(true);
            }

            if(flashcount >= 10) {
                flashcount = 0;
                FLASH_LEDS(false);
            }
            flashcount++;
        }

        // null gravity from the Z accel
        // TO-DO: replace with gravity #define form location.cpp
        accel_offset.z += GRAVITY;

        total_change = fabs(prev.x - accel_offset.x) + fabs(prev.y - accel_offset.y) + fabs(prev.z - accel_offset.z);
        max_offset = (accel_offset.x > accel_offset.y) ? accel_offset.x : accel_offset.y;
        max_offset = (max_offset > accel_offset.z) ? max_offset : accel_offset.z;

        delay_cb(500);
    } while (  total_change > AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE || max_offset > AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET);

    // set the global accel offsets
    _accel_offset = accel_offset;

    Serial.printf_P(PSTR(" "));

}

#if !defined( __AVR_ATmega1280__ )
// perform accelerometer calibration including providing user instructions and feedback
bool AP_InertialSensor::calibrate_accel(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on), void (*send_msg)(const prog_char_t *, ...))
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
                msg = PSTR("on it's left side");
                break;
            case 2:
                msg = PSTR("on it's right side");
                break;
            case 3:
                msg = PSTR("nose down");
                break;
            case 4:
                msg = PSTR("nose up");
                break;
            case 5:
                msg = PSTR("on it's back");
                break;
        }
        if (send_msg == NULL) {
            Serial.printf_P(PSTR("USER: Place APM %S and press any key.\n"), msg);
        }else{
            send_msg(PSTR("USER: Place APM %S and press any key.\n"), msg);
        }

        // wait for user input
        while( !Serial.available() ) {
            delay_cb(20);
        }
        // clear unput buffer
        while( Serial.available() ) {
            Serial.read();
        }

        // clear out any existing samples from ins
        update();

        // wait until we have 32 samples
        while( num_samples_available() < 32 * SAMPLE_UNIT ) {
            delay_cb(1);
        }

        // read samples from ins
        update();

        // capture sample
        samples[i] = get_accel();
    }

    // run the calibration routine
    if( _calibrate_accel(samples, new_offsets, new_scaling) ) {
        if (send_msg == NULL) {
            Serial.printf_P(PSTR("Calibration successful\n"));
        }else{
            send_msg(PSTR("Calibration successful\n"));
        }

        // set and save calibration
        _accel_offset.set(new_offsets);
        _accel_scale.set(new_scaling);
        _save_parameters();
        return true;
    }

    if (send_msg == NULL) {
        Serial.printf_P(PSTR("Calibration failed\n"));
    } else {
        send_msg(PSTR("Calibration failed\n"));
    }
    // restore original scaling and offsets
    _accel_offset.set(orig_offset);
    _accel_scale.set(orig_scale);
    return false;
}

// _calibrate_model - perform low level accel calibration
// accel_sample are accelerometer samples collected in 6 different positions
// accel_offsets are output from the calibration routine
// accel_scale are output from the calibration routine
// returns true if successful
bool AP_InertialSensor::_calibrate_accel( Vector3f accel_sample[6], Vector3f& accel_offsets, Vector3f& accel_scale )
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
    beta[3] = beta[4] = beta[5] = 1.0/GRAVITY;
    
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
    accel_scale.x = beta[3] * GRAVITY;
    accel_scale.y = beta[4] * GRAVITY;
    accel_scale.z = beta[5] * GRAVITY;
    accel_offsets.x = beta[0] * accel_scale.x;
    accel_offsets.y = beta[1] * accel_scale.y;
    accel_offsets.z = beta[2] * accel_scale.z;

    // sanity check scale
    if( accel_scale.is_nan() || fabs(accel_scale.x-1.0) > 0.1 || fabs(accel_scale.y-1.0) > 1.1 || fabs(accel_scale.z-1.0) > 1.1 ) {
        success = false;
    }
    // sanity check offsets (2.0 is roughly 2/10th of a G, 5.0 is roughly half a G)
    if( accel_offsets.is_nan() || fabs(accel_offsets.x) > 2.0 || fabs(accel_offsets.y) > 2.0 || fabs(accel_offsets.z) > 5.0 ) {
        success = false;
    }

    // return success or failure
    return success;
}

void AP_InertialSensor::_calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    
    for( j=0; j<3; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0*b*b*dx;
        jacobian[3+j] = -2.0*b*dx*dx;
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
        dS[j] = 0.0;
        for( k=0; k<6; k++ ) {
            JS[j][k] = 0.0;
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
            if( mu != 0.0 ) {
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
        JS[i][i] = 1.0;
        
        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0;
        }
    }

    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}

#endif // __AVR_ATmega1280__

