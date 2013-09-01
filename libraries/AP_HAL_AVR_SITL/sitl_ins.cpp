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
#include "../SITL/SITL.h"
#include "Scheduler.h"
#include <AP_Math.h>
#include "../AP_ADC/AP_ADC.h"
#include <SITL_State.h>

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
	return airspeed_raw/4;
}


float SITL_State::_gyro_drift(void)
{
	if (_sitl->drift_speed == 0.0) {
		return 0;
	}
	double period  = _sitl->drift_time * 2;
	double minutes = fmod(_scheduler->_micros() / 60.0e6, period);
	if (minutes < period/2) {
		return minutes * ToRad(_sitl->drift_speed);
	}
	return (period - minutes) * ToRad(_sitl->drift_speed);

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
			     float airspeed)
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
	xAccel += accel_noise * _rand_float();
	yAccel += accel_noise * _rand_float();
	zAccel += accel_noise * _rand_float();

	p += gyro_noise * _rand_float();
	q += gyro_noise * _rand_float();
	r += gyro_noise * _rand_float();

	p += _gyro_drift();
	q += _gyro_drift();
	r += _gyro_drift();

	_ins->set_gyro(Vector3f(p, q, r) + _ins->get_gyro_offsets());
	_ins->set_accel(Vector3f(xAccel, yAccel, zAccel) + _ins->get_accel_offsets());

	airspeed_pin_value = _airspeed_sensor(airspeed);
}

#endif
