/*
  SITL handling

  This emulates the ADS7844 ADC

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <math.h>
#include <AP_ADC.h>
#include <SITL.h>
#include <avr/interrupt.h>
#include "wiring.h"
#include "sitl_adc.h"
#include "desktop.h"
#include "util.h"

extern SITL sitl;

/*
  convert airspeed in m/s to an airspeed sensor value
 */
static uint16_t airspeed_sensor(float airspeed)
{
	const float airspeed_ratio = 1.9936;
	const float airspeed_offset = 2820;
	float airspeed_pressure, airspeed_raw;

	airspeed_pressure = sqr(airspeed) / airspeed_ratio;
	airspeed_raw = airspeed_pressure + airspeed_offset;
	return airspeed_raw;
}


static float gyro_drift(void)
{
	if (sitl.drift_speed == 0.0) {
		return 0;
	}
	double period  = sitl.drift_time * 2;
	double minutes = fmod(micros() / 60.0e6, period);
	if (minutes < period/2) {
		return minutes * ToRad(sitl.drift_speed);
	}
	return (period - minutes) * ToRad(sitl.drift_speed);

}

/*
  setup the ADC channels with new input

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
void sitl_update_adc(float roll, 	float pitch, 	float yaw,		// Relative to earth
		     double rollRate, 	double pitchRate,double yawRate,	// Local to plane
		     double xAccel, 	double yAccel, 	double zAccel,		// Local to plane
		     float airspeed)
{
	static const uint8_t sensor_map[6] = { 1, 2, 0, 4, 5, 6 };
	static const float _sensor_signs[6] = { 1, -1, -1, 1, -1, -1 };
	const float accel_offset = 2041;
	const float gyro_offset = 1658;
	const float _gyro_gain_x = ToRad(0.4);
	const float _gyro_gain_y = ToRad(0.41);
	const float _gyro_gain_z = ToRad(0.41);
	const float _accel_scale = 9.80665 / 423.8;
	double adc[7];
	double p, q, r;
	extern float sitl_motor_speed[4];
	bool motors_on = false;

	SITL::convert_body_frame(roll, pitch,
				 rollRate, pitchRate, yawRate,
				 &p, &q, &r);

	for (uint8_t i=0; i<4; i++) {
		if (sitl_motor_speed[i] > 0.0) {
			motors_on = true;
		}
	}

	// minimum noise levels are 2 bits
	float accel_noise = _accel_scale*2;
	float gyro_noise = _gyro_gain_y*2;
	if (motors_on) {
		// add extra noise when the motors are on
		accel_noise += sitl.accel_noise;
		gyro_noise += ToRad(sitl.gyro_noise);
	}
	xAccel += accel_noise * rand_float();
	yAccel += accel_noise * rand_float();
	zAccel += accel_noise * rand_float();

	p += gyro_noise * rand_float();
	q += gyro_noise * rand_float();
	r += gyro_noise * rand_float();

	p += gyro_drift();
	q += gyro_drift();
	r += gyro_drift();

	/* work out the ADC channel values */
	adc[0] =  (p   / (_gyro_gain_x * _sensor_signs[0])) + gyro_offset;
	adc[1] =  (q   / (_gyro_gain_y * _sensor_signs[1])) + gyro_offset;
	adc[2] =  (r   / (_gyro_gain_z * _sensor_signs[2])) + gyro_offset;

	adc[3] =  (xAccel / (_accel_scale * _sensor_signs[3])) + accel_offset;
	adc[4] =  (yAccel / (_accel_scale * _sensor_signs[4])) + accel_offset;
	adc[5] =  (zAccel / (_accel_scale * _sensor_signs[5])) + accel_offset;

	/* tell the UDR2 register emulation what values to give to the driver */
	for (uint8_t i=0; i<6; i++) {
		UDR2.set(sensor_map[i], adc[i]);
	}
	// set the airspeed sensor input
	UDR2.set(7, airspeed_sensor(airspeed));

	/* FIX: rubbish value for temperature for now */
	UDR2.set(3, 2000);

	runInterrupt(6);
}


/*
  setup ADC emulation
 */
void sitl_setup_adc(void)
{
	// mark it always ready. This is the register
	// the ADC driver uses to tell if there is new data pending
	UCSR2A = (1 << RXC2);
}
