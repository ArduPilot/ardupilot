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

// #defines to control function of RC Channel used to manually provide angular offset to AP_Mount when we can't use RC_Channel_aux (which is the case for ArduCopter).
#define AP_MOUNT_MANUAL_RC_FUNCTION_DISABLED	0
#define AP_MOUNT_MANUAL_RC_FUNCTION_ROLL		1
#define AP_MOUNT_MANUAL_RC_FUNCTION_PITCH		2
#define AP_MOUNT_MANUAL_RC_FUNCTION_YAW			3

class AP_Mount
{
public:
	//Constructor
	AP_Mount(const struct Location *current_loc, GPS *&gps, AP_AHRS *ahrs);

	// MAVLink methods
	void configure_msg(mavlink_message_t* msg);
	void control_msg(mavlink_message_t* msg);
	void status_msg(mavlink_message_t* msg);
	void set_roi_cmd();
	void configure_cmd();
	void control_cmd();

	// should be called periodically
	void update_mount_position();
	void debug_output();			///< For testing and development. Called in the medium loop.

	// to allow manual input of an angle from the pilot when RC_Channel_aux cannot be used
	void set_manual_rc_channel(RC_Channel *rc);			// define which RC_Channel is to be used for manual control
	void set_manual_rc_channel_function(int8_t fn);		// set whether manual rc channel controlls roll (1), pitch (2) or yaw (3).

	// hook for eeprom variables
    static const struct AP_Param::GroupInfo var_info[];

private:

	//methods
	void set_mode(enum MAV_MOUNT_MODE mode);

	void set_retract_angles(float roll, float pitch, float yaw);		///< set mount retracted position
	void set_neutral_angles(float roll, float pitch, float yaw);
	void set_control_angles(float roll, float pitch, float yaw);
	void set_GPS_target_location(Location targetGPSLocation);	///< used to tell the mount to track GPS location

	// internal methods
	void calc_GPS_target_angle(struct Location *target);
	void stabilize();

	//members
	AP_AHRS *_ahrs;          ///< Rotation matrix from earth to plane.
	GPS     *&_gps;
	const struct Location *_current_loc;
	static const float t7 = 10000000.0;
	float _roll_control_angle;  ///< radians
	float _pitch_control_angle; ///< radians
	float _yaw_control_angle;   ///< radians

	float _roll_angle;	 ///< degrees
	float _pitch_angle;	 ///< degrees
	float _yaw_angle;	 ///< degrees

	AP_Int8 _stab_roll;  ///< (1 = yes, 0 = no)
	AP_Int8 _stab_pitch; ///< (1 = yes, 0 = no)
	AP_Int8 _stab_yaw;   ///< (1 = yes, 0 = no)

	AP_Int8 _mount_mode;

	struct Location _target_GPS_location;

	AP_Vector3f _retract_angles;		///< retracted position for mount, vector.x = roll vector.y = pitch, vector.z=yaw
	AP_Vector3f _neutral_angles;		///< neutral position for mount, vector.x = roll vector.y = pitch, vector.z=yaw
	AP_Vector3f _control_angles;		///< GCS controlled position for mount, vector.x = roll vector.y = pitch, vector.z=yaw

	// RC_Channel for providing direct angular input from pilot
	RC_Channel* _manual_rc;
	int8_t		_manual_rc_function;
};
#endif
