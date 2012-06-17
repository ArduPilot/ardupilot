// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/************************************************************
* AP_mount -- library to control a 2 or 3 axis mount.		*
*															*
* Author:  Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
*		   Gregory Fletcher;								*
*															*
* Purpose:  Move a 2 or 3 axis mount attached to vehicle,	*
*			Used for mount to track targets or stabilise	*
*			camera plus	other modes.						*
*															*
* Usage:	Use in main code to control	mounts attached to	*
*			vehicle.										*
*															*
*Comments:  All angles in degrees * 100, distances in meters*
*			unless otherwise stated.						*
 ************************************************************/
#ifndef AP_Mount_H
#define AP_Mount_H

#include <FastSerial.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <GCS_MAVLink.h>
#include <../RC_Channel/RC_Channel_aux.h>

class AP_Mount
{
public:
	//Constructor
	AP_Mount(const struct Location *current_loc, GPS *&gps, AP_AHRS *ahrs);

	//enums
	enum MountType{
		k_pan_tilt = 0,			///< yaw-pitch
		k_tilt_roll = 1,		///< pitch-roll
		k_pan_tilt_roll = 2,	///< yaw-pitch-roll
	};

	// MAVLink methods
	void configure_msg(mavlink_message_t* msg);
	void control_msg(mavlink_message_t* msg);
	void status_msg(mavlink_message_t* msg);
	void set_roi_cmd();
	void configure_cmd();
	void control_cmd();

	// should be called periodically
	void update_mount_position();
	void update_mount_type();		///< Auto-detect the mount gimbal type depending on the functions assigned to the servos
	void debug_output();			///< For testing and development. Called in the medium loop.
	// Accessors
	enum MountType get_mount_type();

private:

	//methods
	void set_mode(enum MAV_MOUNT_MODE mode);

	void set_retract_angles(int roll, int pitch, int yaw);		///< set mount retracted position
	void set_neutral_angles(int roll, int pitch, int yaw);
	void set_mavlink_angles(int roll, int pitch, int yaw);
	void set_GPS_target_location(Location targetGPSLocation);	///< used to tell the mount to track GPS location

	// internal methods
	void calc_GPS_target_angle(struct Location *target);
	void calculate();
	long rc_map(RC_Channel_aux* rc_ch);

	//members
	AP_AHRS *_ahrs;          ///< Rotation matrix from earth to plane.
	GPS     *&_gps;
	const struct Location *_current_loc;
	static const float t7 = 10000000.0;
	float _roll_control_angle;
	float _pitch_control_angle;
	float _yaw_control_angle;

	int16_t _roll_angle;	 ///< degrees*100
	int16_t _pitch_angle;	 ///< degrees*100
	int16_t _yaw_angle;		 ///< degrees*100

	uint8_t _stab_roll;  ///< (1 = yes, 0 = no)
	uint8_t _stab_pitch; ///< (1 = yes, 0 = no)
	uint8_t _stab_yaw;   ///< (1 = yes, 0 = no)

	enum MAV_MOUNT_MODE _mount_mode;
	MountType _mount_type;

	struct Location _target_GPS_location;

	Vector3i _retract_angles;		///< retracted position for mount, vector.x = roll vector.y = pitch, vector.z=yaw
	Vector3i _neutral_angles;		///< neutral position for mount, vector.x = roll vector.y = pitch, vector.z=yaw
	Vector3i _mavlink_angles;		///< mavlink position for mount, vector.x = roll vector.y = pitch, vector.z=yaw
};
#endif
