/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	SITL.cpp - software in the loop state

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public License
    as published by the Free Software Foundation; either version 2.1
    of the License, or (at your option) any later version.
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <SITL.h>

// table of user settable parameters
const AP_Param::GroupInfo SITL::var_info[] PROGMEM = {
    AP_GROUPINFO("BARO_RND",   0, SITL,  baro_noise,  3),
    AP_GROUPINFO("GYR_RND",    1, SITL,  gyro_noise, 30),
    AP_GROUPINFO("ACC_RND",    2, SITL,  accel_noise, 3),
    AP_GROUPINFO("MAG_RND",    3, SITL,  mag_noise,  10),
    AP_GROUPINFO("GPS_DISABLE",4, SITL,  gps_disable, 0),
    AP_GROUPINFO("DRIFT_SPEED",5, SITL,  drift_speed, 0.2),
    AP_GROUPINFO("DRIFT_TIME", 6, SITL,  drift_time,  5),
    AP_GROUPINFO("GPS_DELAY",  7, SITL,  gps_delay,   4),
    AP_GROUPINFO("ENGINE_MUL", 8, SITL,  engine_mul,  1),
    AP_GROUPINFO("WIND_SPD",   9, SITL,  wind_speed,  5),
    AP_GROUPINFO("WIND_DIR",  10, SITL,  wind_direction,  180),
    AP_GROUPINFO("WIND_TURB", 11, SITL,  wind_turbulance,  0.2),
    AP_GROUPEND
};



/* report SITL state via MAVLink */
void SITL::simstate_send(mavlink_channel_t chan)
{
	double p, q, r;
	float yaw;

	// we want the gyro values to be directly comparable to the
	// raw_imu message, which is in body frame
	convert_body_frame(state.rollDeg, state.pitchDeg,
                       state.rollRate, state.pitchRate, state.yawRate,
                       &p, &q, &r);

	// convert to same conventions as DCM
	yaw = state.yawDeg;
	if (yaw > 180) {
		yaw -= 360;
	}

    mavlink_msg_simstate_send(chan,
                              ToRad(state.rollDeg),
                              ToRad(state.pitchDeg),
                              ToRad(yaw),
                              state.xAccel,
                              state.yAccel,
                              state.zAccel,
                              p, q, r,
                              state.latitude,
                              state.longitude);
}

// convert a set of roll rates from earth frame to body frame
void SITL::convert_body_frame(double rollDeg, double pitchDeg,
                              double rollRate, double pitchRate, double yawRate,
                              double *p, double *q, double *r)
{
	double phi, theta, phiDot, thetaDot, psiDot;

	phi = ToRad(rollDeg);
	theta = ToRad(pitchDeg);
	phiDot = ToRad(rollRate);
	thetaDot = ToRad(pitchRate);
	psiDot = ToRad(yawRate);

	*p = phiDot - psiDot*sin(theta);
	*q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta);
	*r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot;    
}

