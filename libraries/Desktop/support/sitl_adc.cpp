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
 */
void sitl_update_adc(float roll, float pitch, float yaw, float airspeed)
{
	static const uint8_t sensor_map[6] = { 1, 2, 0, 4, 5, 6 };
	static const float _sensor_signs[6] = { 1, -1, -1, 1, -1, -1 };
	const float accel_offset = 2041;
	const float gyro_offset = 1658;
#define ToRad(x) (x*0.01745329252)  // *pi/180
	const float _gyro_gain_x = 0.4;
	const float _gyro_gain_y = 0.41;
	const float _gyro_gain_z = 0.41;
	const float _accel_scale = 9.80665 / 423.8;
	float adc[7];
	float xAccel, yAccel, zAccel, scale;
	float rollRate, pitchRate, yawRate;
	static uint32_t last_update;
	static float last_roll, last_pitch, last_yaw;
	unsigned long delta_t;

	// 200Hz max
	if (micros() - last_update < 5000) {
		return;
	}

	/* map roll/pitch/yaw to values the accelerometer would see */
	xAccel = sin(ToRad(pitch)) * cos(ToRad(roll));
	yAccel = -sin(ToRad(roll)) * cos(ToRad(pitch));
	zAccel = -cos(ToRad(roll)) * cos(ToRad(pitch));
	scale = 9.81 / sqrt((xAccel*xAccel)+(yAccel*yAccel)+(zAccel*zAccel));
	xAccel *= scale;
	yAccel *= scale;
	zAccel *= scale;

	/* map roll/pitch/yaw to values the gyro would see */
	if (last_update == 0) {
		rollRate = 0;
		pitchRate = 0;
		yawRate = 0;
		delta_t = micros();
	} else {
		delta_t = micros() - last_update;
		float rollChange, pitchChange, yawChange;
		rollChange  = normalise180(roll - last_roll);
		pitchChange = normalise180(pitch - last_pitch);
		yawChange   = normalise180(yaw - last_yaw);
		rollRate  = 1.0e6 * rollChange  / delta_t;
		pitchRate = 1.0e6 * pitchChange / delta_t;
		yawRate   = 1.0e6 * yawChange   / delta_t;
	}
	last_update += delta_t;

	last_roll = roll;
	last_pitch = pitch;
	last_yaw = yaw;

	/* work out the ADC channel values */
	adc[0] =  (rollRate  / (_gyro_gain_x * _sensor_signs[0])) + gyro_offset;
	adc[1] =  (pitchRate / (_gyro_gain_y * _sensor_signs[1])) + gyro_offset;
	adc[2] =  (yawRate   / (_gyro_gain_z * _sensor_signs[2])) + gyro_offset;

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

#if 0
	extern AP_DCM_HIL dcm;
	dcm.setHil(ToRad(roll), ToRad(pitch), ToRad(yaw),
		   ToRad(rollRate), ToRad(pitchRate), ToRad(yawRate));

#endif

#if 0
	extern AP_DCM dcm;
	Vector3f omega = dcm.get_gyro();
	printf("dt=%5u adc[2]=%6.1f roll=%6.1f / %6.1f yaw=%6.1f / %6.1f  yawRate=%6.3f / %6.3f\n",
	       (unsigned)delta_t,
	       adc[2],
	       roll, dcm.roll_sensor/100.0, yaw, dcm.yaw_sensor/100.0, yawRate, ToDeg(omega.z));
#endif
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
