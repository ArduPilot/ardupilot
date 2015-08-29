/*
  SITL handling

  This emulates the ADS7844 ADC

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"

#include <AP_Math/AP_Math.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <SITL/SITL.h>
#include "Scheduler.h"
#include <AP_Math/AP_Math.h>
#include <AP_ADC/AP_ADC.h>
#include "SITL_State.h"
#include <fenv.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

/*
  convert airspeed in m/s to an airspeed sensor value
 */
uint16_t SITL_State::_airspeed_sensor(float airspeed)
{
    const float airspeed_ratio = 1.9936f;
    const float airspeed_offset = 2013;
    float airspeed_pressure, airspeed_raw;

    airspeed_pressure = (airspeed*airspeed) / airspeed_ratio;
    airspeed_raw = airspeed_pressure + airspeed_offset;
    if (airspeed_raw/4 > 0xFFFF) {
        return 0xFFFF;
    }
    // add delay
    uint32_t now = hal.scheduler->millis();
    uint32_t best_time_delta_wind = 200; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index_wind = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time_wind >= 10) { // store data every 10 ms.
        last_store_time_wind = now;
        if (store_index_wind > wind_buffer_length-1) { // reset buffer index if index greater than size of buffer
            store_index_wind = 0;
        }
        buffer_wind[store_index_wind].data = airspeed_raw; // add data to current index
        buffer_wind[store_index_wind].time = last_store_time_wind; // add time to current index
        store_index_wind = store_index_wind + 1; // increment index
    }

    // return delayed measurement
    delayed_time_wind = now - _sitl->wind_delay; // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=wind_buffer_length-1; i++) {
        time_delta_wind = abs(delayed_time_wind - buffer_wind[i].time); // find difference between delayed time and time stamp in buffer
        // if this difference is smaller than last delta, store this time
        if (time_delta_wind < best_time_delta_wind) {
            best_index_wind = i;
            best_time_delta_wind = time_delta_wind;
        }
    }
    if (best_time_delta_wind < 200) { // only output stored state if < 200 msec retrieval error
        airspeed_raw = buffer_wind[best_index_wind].data;
    }

    return airspeed_raw/4;
}


float SITL_State::_gyro_drift(void)
{
    if (_sitl->drift_speed == 0.0f ||
            _sitl->drift_time == 0.0f) {
        return 0;
    }
    double period  = _sitl->drift_time * 2;
    double minutes = fmod(_scheduler->_micros64() / 60.0e6, period);
    if (minutes < period/2) {
        return minutes * ToRad(_sitl->drift_speed);
    }
    return (period - minutes) * ToRad(_sitl->drift_speed);

}

/*
  emulate an analog rangefinder
 */
uint16_t SITL_State::_ground_sonar(void)
{
    float altitude = height_agl();

    float voltage = 5.0f;
    if (fabsf(_sitl->state.rollDeg) < 90 &&
            fabsf(_sitl->state.pitchDeg) < 90) {
        // adjust for apparent altitude with roll
        altitude /= cosf(radians(_sitl->state.rollDeg)) * cosf(radians(_sitl->state.pitchDeg));

        altitude += _sitl->sonar_noise * _rand_float();

        // Altitude in in m, scaler in meters/volt
        voltage = altitude / _sitl->sonar_scale;
        voltage = constrain_float(voltage, 0, 5.0f);

        if (_sitl->sonar_glitch >= (_rand_float() + 1.0f)/2.0f) {
            voltage = 5.0f;
        }
    }

    return 1023*(voltage / 5.0f);
}

/*
  setup the INS input channels with new input

  Note that this uses roll, pitch and yaw only as inputs. The
  simulator rollrates are instantaneous, whereas we need to use
  average rates to cope with slow update rates.

  inputs are in degrees

	phi - roll
	theta - pitch
	psi - true heading
	alpha - angle of attack
	beta - side slip
	phidot - roll rate
	thetadot - pitch rate
	psidot - yaw rate
	v_north - north velocity in local/body frame
	v_east - east velocity in local/body frame
	v_down - down velocity in local/body frame
	A_X_pilot - X accel in body frame
	A_Y_pilot - Y accel in body frame
	A_Z_pilot - Z accel in body frame

  Note: doubles on high prec. stuff are preserved until the last moment

 */
void SITL_State::_update_ins(float roll, 	float pitch, 	float yaw,		// Relative to earth
                             double rollRate, 	double pitchRate,double yawRate,	// Local to plane
                             double xAccel, 	double yAccel, 	double zAccel,		// Local to plane
                             float airspeed,	float altitude)
{
    if (_ins == NULL) {
        // no inertial sensor in this sketch
        return;
    }

    // minimum noise levels are 2 bits, but averaged over many
    // samples, giving around 0.01 m/s/s
    float accel_noise = 0.01f;
    float accel2_noise = 0.01f;
    // minimum gyro noise is also less than 1 bit
    float gyro_noise = ToRad(0.04f);
    if (_motors_on) {
        // add extra noise when the motors are on
        accel_noise += _sitl->accel_noise;
        accel2_noise += _sitl->accel2_noise;
        gyro_noise += ToRad(_sitl->gyro_noise);
    }
    // get accel bias (add only to first accelerometer)
    Vector3f accel_bias = _sitl->accel_bias.get();
    float xAccel1 = xAccel + accel_noise * _rand_float() + accel_bias.x;
    float yAccel1 = yAccel + accel_noise * _rand_float() + accel_bias.y;
    float zAccel1 = zAccel + accel_noise * _rand_float() + accel_bias.z;

    float xAccel2 = xAccel + accel2_noise * _rand_float();
    float yAccel2 = yAccel + accel2_noise * _rand_float();
    float zAccel2 = zAccel + accel2_noise * _rand_float();

    if (fabsf(_sitl->accel_fail) > 1.0e-6f) {
        xAccel1 = _sitl->accel_fail;
        yAccel1 = _sitl->accel_fail;
        zAccel1 = _sitl->accel_fail;
    }

    Vector3f accel0 = Vector3f(xAccel1, yAccel1, zAccel1) + _ins->get_accel_offsets(0);
    Vector3f accel1 = Vector3f(xAccel2, yAccel2, zAccel2) + _ins->get_accel_offsets(1);
    _ins->set_accel(0, accel0);
    _ins->set_accel(1, accel1);

    // check noise
    _ins->calc_vibration_and_clipping(0, accel0, 0.0025f);
    _ins->calc_vibration_and_clipping(1, accel1, 0.0025f);

    float p = radians(rollRate) + _gyro_drift();
    float q = radians(pitchRate) + _gyro_drift();
    float r = radians(yawRate) + _gyro_drift();

    float p1 = p + gyro_noise * _rand_float();
    float q1 = q + gyro_noise * _rand_float();
    float r1 = r + gyro_noise * _rand_float();

    float p2 = p + gyro_noise * _rand_float();
    float q2 = q + gyro_noise * _rand_float();
    float r2 = r + gyro_noise * _rand_float();

    _ins->set_gyro(0, Vector3f(p1, q1, r1) + _ins->get_gyro_offsets(0));
    _ins->set_gyro(1, Vector3f(p2, q2, r2) + _ins->get_gyro_offsets(1));


    sonar_pin_value    = _ground_sonar();
    float airspeed_simulated = (fabsf(_sitl->aspd_fail) > 1.0e-6f) ? _sitl->aspd_fail : airspeed;
    airspeed_pin_value = _airspeed_sensor(airspeed_simulated + (_sitl->aspd_noise * _rand_float()));
}

#endif
