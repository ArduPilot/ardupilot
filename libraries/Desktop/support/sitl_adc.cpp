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
#include <AP_DCM.h>
#include <AP_ADC.h>
#include "wiring.h"
#include "sitl_adc.h"
#include "desktop.h"
#include "util.h"

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
#define ToRad(x) (x*0.01745329252)  // *pi/180
	const float _gyro_gain_x = ToRad(0.4);
	const float _gyro_gain_y = ToRad(0.41);
	const float _gyro_gain_z = ToRad(0.41);
	const float _accel_scale = 9.80665 / 423.8;
	double adc[7];
	double phi, theta, phiDot, thetaDot, psiDot;
	double p, q, r;

	/* convert the angular velocities from earth frame to
	   body frame. Thanks to James Goppert for the formula
	*/
	phi = ToRad(roll);
	theta = ToRad(pitch);
	phiDot = ToRad(rollRate);
	thetaDot = ToRad(pitchRate);
	psiDot = ToRad(yawRate);

	p = phiDot - psiDot*sin(theta);
	q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta);
	r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot;

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

	runInterrupt(6);

	// set the airspeed sensor input
	UDR2.set(7, airspeed_sensor(airspeed));

	/* FIX: rubbish value for temperature for now */
	UDR2.set(3, 2000);

#if 0
	extern AP_DCM_HIL dcm;
	dcm.setHil(ToRad(roll), ToRad(pitch), ToRad(yaw),
		   ToRad(rollRate), ToRad(pitchRate), ToRad(yawRate));

#endif

	static uint32_t last_report;
	uint32_t tnow = millis();
	extern AP_DCM dcm;
	Vector3f omega = dcm.get_gyro();
	// report roll/pitch discrepancies
	if (tnow - last_report > 5000 ||
	    (tnow - last_report > 1000 &&
	     (fabs(roll - dcm.roll_sensor/100.0) > 5.0 ||
	      fabs(pitch - dcm.pitch_sensor/100.0) > 5.0))) {
		last_report = tnow;
		/*printf("roll=%5.1f / %5.1f  pitch=%5.1f / %5.1f  rr=%5.2f / %5.2f  pr=%5.2f / %5.2f\n",
		       roll, dcm.roll_sensor/100.0,
		       pitch, dcm.pitch_sensor/100.0,
		       rollRate, ToDeg(omega.x),
		       pitchRate, ToDeg(omega.y));
		*/
	}
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
