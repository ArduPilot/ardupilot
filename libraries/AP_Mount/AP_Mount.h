// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/************************************************************	          
* AP_mount -- library to control a 2 or 3 axis mount.		*   
*															*   
* Author:  Joe Holdsworth;									*
*		   Ritchie Wilson;									*
*			Amiclair Lucus;									*   
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

#include <AP_Common.h>
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
		gps = 0,
		stabilise = 1, //note the correct english spelling :)
		roam = 2,
		assisted = 3,
		landing = 4,
		none = 5
	};

	enum MountType{
		pitch_yaw = 0,
		pitch_roll = 1, //note the correct english spelling :)
		pitch_roll_yaw = 2,
	};
	
	//Accessors
	void SetPitchYaw(int pitchCh, int yawCh);
	void SetPitchRoll(int pitchCh, int rollCh);
	void SetPitchRollYaw(int pitchCh, int rollCh, int yawCh);

	void SetGPSTarget(Location targetGPSLocation); 		//used to tell the mount to track GPS location
	void SetAssisted(int roll, int pitch, int yaw);
	void SetMountFreeRoam(int roll, int pitch, int yaw);//used in the FPV for example,   
	void SetMountLanding(int roll, int pitch, int yaw); //set mount landing position	
	void SetNone();
	
	//methods
	void UpDateMount();
	void SetMode(MountMode mode);    
	
	int pitchAngle; //degrees*100
	int rollAngle;	//degrees*100
	int yawAngle;	//degrees*100
protected:
	//methods
	void CalcGPSTargetVector(struct Location *target);
	//void CalculateDCM(int roll, int pitch, int yaw);
	//members
	AP_DCM		*_dcm;
	GPS 		*_gps;

	MountMode _mountmode;
	MountType _mountType;

	struct Location _targetGPSLocation;
	Vector3f _GPSVector;			//target vector calculated stored in meters
	
	Vector3i _RoamAngles;			//used for roam mode vector.x = roll vector.y = pitch, vector.z=yaw	
	Vector3i _LandingAngles;		//landing position for mount, vector.x = roll vector.y = pitch, vector.z=yaw

	Vector3i _AssistAngles;			//used to keep angles that user has supplied from assisted targetting
	Vector3f _AssistVector;			//used to keep vector calculated from _AssistAngles

	Matrix3f _m;					//holds 3 x 3 matrix, var is used as temp in calcs
	Vector3f _targ;					//holds target vector, var is used as temp in calcs
};
#endif