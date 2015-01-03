/*
  SITL handling

  This emulates the ADS7844 ADC

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

#include <AP_Math.h>
#include "../AP_Compass/AP_Compass.h"
#include "../AP_Declination/AP_Declination.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"
#include "../SITL/SITL.h"
#include "Scheduler.h"
#include <AP_Math.h>
#include "../AP_ADC/AP_ADC.h"
#include <SITL_State.h>
#include <fenv.h>


using namespace AVR_SITL;

/*
  convert airspeed in m/s to an airspeed sensor value
 */
uint16_t SITL_State::_airspeed_sensor(float airspeed)
{
	const float airspeed_ratio = 1.9936;
	const float airspeed_offset = 2013;
	float airspeed_pressure, airspeed_raw;

	airspeed_pressure = (airspeed*airspeed) / airspeed_ratio;
	airspeed_raw = airspeed_pressure + airspeed_offset;
        if (airspeed_raw/4 > 0xFFFF) {
            return 0xFFFF;
        }
	return airspeed_raw/4;
}


float SITL_State::_gyro_drift(void)
{
	if (_sitl->drift_speed == 0.0) {
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
        altitude /= cos(radians(_sitl->state.rollDeg)) * cos(radians(_sitl->state.pitchDeg));
        
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
	double p, q, r;

	if (_ins == NULL) {
		// no inertial sensor in this sketch
		return;
	}

	SITL::convert_body_frame(roll, pitch,
				 rollRate, pitchRate, yawRate,
				 &p, &q, &r);

	// minimum noise levels are 2 bits, but averaged over many
	// samples, giving around 0.01 m/s/s
	float accel_noise = 0.01;
        // minimum gyro noise is also less than 1 bit
	float gyro_noise = ToRad(0.04);
	if (_motors_on) {
		// add extra noise when the motors are on
		accel_noise += _sitl->accel_noise;
		gyro_noise += ToRad(_sitl->gyro_noise);
	}
	// get accel bias (add only to first accelerometer)
	Vector3f accel_bias = _sitl->accel_bias.get();
	float xAccel1 = xAccel + accel_noise * _rand_float() + accel_bias.x;
	float yAccel1 = yAccel + accel_noise * _rand_float() + accel_bias.y;
	float zAccel1 = zAccel + accel_noise * _rand_float() + accel_bias.z;

	float xAccel2 = xAccel + accel_noise * _rand_float();
	float yAccel2 = yAccel + accel_noise * _rand_float();
	float zAccel2 = zAccel + accel_noise * _rand_float();

        if (fabs(_sitl->accel_fail) > 1.0e-6) {
            xAccel1 = _sitl->accel_fail;
            yAccel1 = _sitl->accel_fail;
            zAccel1 = _sitl->accel_fail;
        }

	_ins->set_accel(0, Vector3f(xAccel1, yAccel1, zAccel1) + _ins->get_accel_offsets(0));
	_ins->set_accel(1, Vector3f(xAccel2, yAccel2, zAccel2) + _ins->get_accel_offsets(1));

	p += _gyro_drift();
	q += _gyro_drift();
	r += _gyro_drift();

	float p1 = p + gyro_noise * _rand_float();
	float q1 = q + gyro_noise * _rand_float();
	float r1 = r + gyro_noise * _rand_float();

	float p2 = p + gyro_noise * _rand_float();
	float q2 = q + gyro_noise * _rand_float();
	float r2 = r + gyro_noise * _rand_float();

	_ins->set_gyro(0, Vector3f(p1, q1, r1) + _ins->get_gyro_offsets(0));
	_ins->set_gyro(1, Vector3f(p2, q2, r2) + _ins->get_gyro_offsets(1));


        sonar_pin_value    = _ground_sonar();
        airspeed_pin_value = _airspeed_sensor(airspeed + (_sitl->aspd_noise * _rand_float()));
}

#endif
