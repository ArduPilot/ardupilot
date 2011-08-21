/****************************************************************	          
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
    * Usage:													*   
    *			Use in main code to control	mounts attached to	*
	*			vehicle.										*
	*															*
	*Comments:  All angles in degrees * 100, distances in meters*
	*			unless otherwise stated.						*
 ***************************************************************/ 
#ifndef AP_Mount_H
#define AP_Mount_H

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_DCM.h>

//all angles in degrees * 100
class AP_Mount
{
public:
	//Constructors
	AP_Mount(GPS *gps, AP_DCM *dcm);

	//enums
	enum MountMode{
		gps = 0,
		stabilise = 1, //note the correct english spelling :)
		fpv = 2,
		assisted = 3,
		antenna = 4,
		landing = 5,
		none = 6
	};

	enum MountType{
		pitch_yaw = 0,
		pitch_roll = 1, //note the correct english spelling :)
		pitch_roll_yaw = 2,
	};
	
	//Accessors
	//used with dcm matrix to calculate target vector
	void SetPitchYaw();
	void SetPitchRoll();
	void SetPitchRollYaw();
	void SetGPSTarget(Location targetGPSLocation); 		//used to tell the mount to track GPS location
	void SetAssisted(int roll, int pitch, int yaw);
	void SetAntenna(Location grndStation);
	
	//action
	void SetMountFPV(int roll, int pitch, int yaw);		//used in the FPV,   
	void SetMountLanding(int roll, int pitch, int yaw); //set mount landing position
	
	//methods
	void UpDateMount();
	void SetMode(MountMode mode);    
	
	int pitchAngle; //degrees*100
	int rollAngle;	//degrees*100
protected:
	//methods
	void CalcGPSTargetVector(struct Location *target);
	//void CalculateDCM(int roll, int pitch, int yaw);
	//members
	AP_DCM			*_dcm;
	GPS 		*_gps;

	MountMode _mountmode;
	struct Location _grndStation;
	struct Location _targetGPSLocation;
	Vector3f _targetVector;				//target vector calculated stored in meters
	Vector3i _mountFPVVector;			//used for FPV mode vector.x = roll vector.y = pitch, vector.z=yaw	
	Vector3i _mountLanding;				//landing position for mount, vector.x = roll vector.y = pitch, vector.z=yaw


};
#endif