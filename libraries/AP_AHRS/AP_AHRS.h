#ifndef AP_AHRS_H
#define AP_AHRS_H
/*
  AHRS (Attitude Heading Reference System) interface for ArduPilot

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#include <AP_Math.h>
#include <inttypes.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_IMU.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class AP_AHRS
{
public:
	// Constructor
	AP_AHRS(IMU *imu, GPS *&gps):
		_imu(imu),
		_gps(gps)
	{
		// base the ki values by the sensors maximum drift
		// rate. The APM2 has gyros which are much less drift
		// prone than the APM1, so we should have a lower ki,
		// which will make us less prone to increasing omegaI
		// incorrectly due to sensor noise
		_gyro_drift_limit = imu->get_gyro_drift_rate();
	}

	// Accessors
	void		set_centripetal(bool b) { _centripetal = b; }
	bool		get_centripetal(void) { return _centripetal; }
	void		set_compass(Compass *compass) { _compass = compass; }

	// Methods
	virtual void update(void) = 0;

	// Euler angles (radians)
	float		roll;
	float		pitch;
	float		yaw;

	// integer Euler angles (Degrees * 100)
	int32_t		roll_sensor;
	int32_t		pitch_sensor;
	int32_t		yaw_sensor;

	// return a smoothed and corrected gyro vector
	virtual Vector3f get_gyro(void) = 0;

	// return the current estimate of the gyro drift
	virtual Vector3f get_gyro_drift(void) = 0;

	// reset the current attitude, used on new IMU calibration
	virtual void reset(bool recover_eulers=false) = 0;

	// how often our attitude representation has gone out of range
	uint8_t renorm_range_count;

	// how often our attitude representation has blown up completely
	uint8_t renorm_blowup_count;

	// return the average size of the roll/pitch error estimate
	// since last call
	virtual float get_error_rp(void) = 0;

	// return the average size of the yaw error estimate
	// since last call
	virtual float get_error_yaw(void) = 0;

	// return a DCM rotation matrix representing our current
	// attitude
	virtual Matrix3f get_dcm_matrix(void) = 0;

protected:
	// pointer to compass object, if enabled
	Compass 	* _compass;

	// time in microseconds of last compass update
	uint32_t        _compass_last_update;

	// note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
	//       IMU under us without our noticing.
	GPS 		*&_gps;
	IMU 		*_imu;

	// true if we are doing centripetal acceleration correction
	bool		_centripetal;

	// the limit of the gyro drift claimed by the sensors, in
	// radians/s/s
	float           _gyro_drift_limit;
};

#include <AP_AHRS_DCM.h>
#include <AP_AHRS_Quaternion.h>
#include <AP_AHRS_HIL.h>

#endif // AP_AHRS_H
