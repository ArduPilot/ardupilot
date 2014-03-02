/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
	SITL.cpp - software in the loop state

*/

#include <AP_Common.h>
#include <AP_HAL.h>
#include <GCS_MAVLink.h>
#include <SITL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo SITL::var_info[] PROGMEM = {
    AP_GROUPINFO("BARO_RND",   0, SITL,  baro_noise,  3),
    AP_GROUPINFO("GYR_RND",    1, SITL,  gyro_noise, 30),
    AP_GROUPINFO("ACC_RND",    2, SITL,  accel_noise, 3),
    AP_GROUPINFO("MAG_RND",    3, SITL,  mag_noise,  10),
    AP_GROUPINFO("GPS_DISABLE",4, SITL,  gps_disable, 0),
    AP_GROUPINFO("DRIFT_SPEED",5, SITL,  drift_speed, 0.2),
    AP_GROUPINFO("DRIFT_TIME", 6, SITL,  drift_time,  5),
    AP_GROUPINFO("GPS_DELAY",  7, SITL,  gps_delay,   2),
    AP_GROUPINFO("ENGINE_MUL", 8, SITL,  engine_mul,  1),
    AP_GROUPINFO("WIND_SPD",   9, SITL,  wind_speed,  0),
    AP_GROUPINFO("WIND_DIR",  10, SITL,  wind_direction,  180),
    AP_GROUPINFO("WIND_TURB", 11, SITL,  wind_turbulance,  0.2),
    AP_GROUPINFO("GPS_TYPE",  12, SITL,  gps_type,  SITL::GPS_TYPE_UBLOX),
    AP_GROUPINFO("GPS_BYTELOSS",  13, SITL,  gps_byteloss,  0),
    AP_GROUPINFO("GPS_NUMSATS",   14, SITL,  gps_numsats,   10),
    AP_GROUPINFO("MAG_ERROR",     15, SITL,  mag_error,  0),
    AP_GROUPINFO("SERVO_RATE",    16, SITL,  servo_rate,  0),
    AP_GROUPINFO("GPS_GLITCH",    17, SITL,  gps_glitch,  0),
    AP_GROUPINFO("GPS_HZ",        18, SITL,  gps_hertz,  5),
    AP_GROUPINFO("BATT_VOLTAGE",  19, SITL,  batt_voltage,  12.6),
    AP_GROUPINFO("ASPD_RND",      20, SITL,  aspd_noise,  0.5),
    AP_GROUPINFO("ACCEL_FAIL",    21, SITL,  accel_fail,  0),
    AP_GROUPINFO("BARO_DRIFT",    22, SITL,  baro_drift,  0),
    AP_GROUPINFO("SONAR_GLITCH",  23, SITL,  sonar_glitch, 0),
    AP_GROUPINFO("SONAR_RND",     24, SITL,  sonar_noise, 0),
    AP_GROUPINFO("RC_FAIL",       25, SITL,  rc_fail, 0),
    AP_GROUPINFO("GPS2_ENABLE",   26, SITL,  gps2_enable, 0),
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
                              state.latitude*1.0e7,
                              state.longitude*1.0e7);
}

/* report SITL state to DataFlash */
void SITL::Log_Write_SIMSTATE(DataFlash_Class &DataFlash)
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

    struct log_AHRS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SIMSTATE_MSG),
        time_ms : hal.scheduler->millis(),
        roll  : (int16_t)(state.rollDeg*100),
        pitch : (int16_t)(state.pitchDeg*100),
        yaw   : (uint16_t)(wrap_360_cd(yaw*100)),
        alt   : (float)state.altitude,
        lat   : (int32_t)(state.latitude*1.0e7),
        lng   : (int32_t)(state.longitude*1.0e7)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
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

	*p = phiDot - psiDot*sinf(theta);
	*q = cosf(phi)*thetaDot + sinf(phi)*psiDot*cosf(theta);
	*r = cosf(phi)*psiDot*cosf(theta) - sinf(phi)*thetaDot;    
}

