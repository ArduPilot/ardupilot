/*
	APM_DCM_HIL.cpp - DCM AHRS Library, for Ardupilot Mega, Hardware in the Loop Model
		Code by James Goppert. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

        Methods:
				update_DCM()	: Updates the AHRS by integrating the rotation matrix over time using the IMU object data
				get_gyro()			: Returns gyro vector corrected for bias
				get_accel()		: Returns accelerometer vector
				get_dcm_matrix()	: Returns dcm matrix

*/
#include <AP_DCM_HIL.h>

#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

/**************************************************/
void
AP_DCM_HIL::setHil(float _roll, float _pitch, float _yaw,
		float _rollRate, float _pitchRate, float _yawRate)
{
	roll = _roll;
	pitch = _pitch;
	yaw = _yaw;

	_omega_integ_corr.x = _rollRate;
	_omega_integ_corr.y = _pitchRate;
	_omega_integ_corr.z = _yawRate;

	roll_sensor =ToDeg(roll)*100;
	pitch_sensor =ToDeg(pitch)*100;
	yaw_sensor =ToDeg(yaw)*100;

	// Need the standard C_body<-nav dcm from navigation frame to body frame
	// Strapdown Inertial Navigation Technology / Titterton/ pg. 41
	float sRoll = sin(roll), cRoll = cos(roll);
	float sPitch = sin(pitch), cPitch = cos(pitch);
	float sYaw = sin(yaw), cYaw = cos(yaw);
	_dcm_matrix.a.x =  cPitch*cYaw;
	_dcm_matrix.a.y =  -cRoll*sYaw+sRoll*sPitch*cYaw;
	_dcm_matrix.a.z =  sRoll*sYaw+cRoll*sPitch*cYaw;
	_dcm_matrix.b.x =  cPitch*sYaw;
	_dcm_matrix.b.y =  cRoll*cYaw+sRoll*sPitch*sYaw;
	_dcm_matrix.b.z =  -sRoll*cYaw+cRoll*sPitch*sYaw;
	_dcm_matrix.c.x =  -sPitch;
	_dcm_matrix.c.y =  sRoll*cPitch;
	_dcm_matrix.c.z =  cRoll*cPitch;
}
