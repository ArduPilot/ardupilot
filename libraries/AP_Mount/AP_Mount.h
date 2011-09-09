// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/************************************************************	          
* AP_mount -- library to control a 2 or 3 axis mount.		*   
*															*   
* Author:  Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*		   Amilcar Lucas;									*
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

//#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_DCM.h>

class AP_Mount
{
public:
	//Constructors
	AP_Mount(GPS *gps, AP_DCM *dcm);

	//enums
	enum MountMode{
		k_gps_target = 0,
		k_stabilise = 1, //note the correct English spelling :)
		k_roam = 2,
		k_assisted = 3,
		k_landing = 4,
		k_none = 5,
		k_manual = 6
	};

	enum MountType{
		k_pan_tilt = 0,			//yaw-pitch
		k_tilt_roll = 1,		//pitch-roll
		k_pan_tilt_roll = 2,	//yaw-pitch-roll
	};
	
	//Accessors
	void set_pitch_yaw(int pitchCh, int yawCh);
	void set_pitch_roll(int pitchCh, int rollCh);
	void set_pitch_roll_yaw(int pitchCh, int rollCh, int yawCh);

	void set_GPS_target(Location targetGPSLocation); 		//used to tell the mount to track GPS location
	void set_assisted(int roll, int pitch, int yaw);
	void set_mount_free_roam(int roll, int pitch, int yaw);//used in the FPV for example,   
	void set_mount_landing(int roll, int pitch, int yaw); //set mount landing position	
	void set_none();
	
	//methods
	void update_mount();
	void update_mount_type();	//Auto-detect the mount gimbal type depending on the functions assigned to the servos
	void set_mode(MountMode mode);    
	
	int pitch_angle; //degrees*100
	int roll_angle;	//degrees*100
	int yaw_angle;	//degrees*100
protected:
	//methods
	void calc_GPS_target_vector(struct Location *target);
	//void CalculateDCM(int roll, int pitch, int yaw);
	//members
	AP_DCM		*_dcm;
	GPS 		*_gps;

	MountMode _mount_mode;
	MountType _mount_type;

	struct Location _target_GPS_location;
	Vector3f _GPS_vector;			//target vector calculated stored in meters
	
	Vector3i _roam_angles;			//used for roam mode vector.x = roll vector.y = pitch, vector.z=yaw	
	Vector3i _landing_angles;		//landing position for mount, vector.x = roll vector.y = pitch, vector.z=yaw

	Vector3i _assist_angles;			//used to keep angles that user has supplied from assisted targeting
	Vector3f _assist_vector;			//used to keep vector calculated from _AssistAngles
};
#endif