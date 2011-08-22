#include "AP_Mount.h"

AP_Mount::AP_Mount(GPS *gps, AP_DCM *dcm)
{
	_dcm=dcm;
	_gps=gps;
}

void AP_Mount::SetPitchYaw(Location targetGPSLocation)
{
	_targetGPSLocation=targetGPSLocation;
	
	//set mode
	_mountmode=gps;

	//update mount position
	UpDateMount();
}

void AP_Mount::SetGPSTarget(Location targetGPSLocation)
{
	_targetGPSLocation=targetGPSLocation;
	
	//set mode
	_mountmode=gps;

	//update mount position
	UpDateMount();
}

void AP_Mount::SetAssisted(int roll, int pitch, int yaw)
{
	_AssistAngles.x = roll;
	_AssistAngles.y = pitch;
	_AssistAngles.z = yaw;

	//set mode
	_mountmode=assisted;

	//update mount position
	UpDateMount();
}

//sets the servo angles for FPV, note angles are * 100
void AP_Mount::SetMountFreeRoam(int roll, int pitch, int yaw)
{
	_RoamAngles.x=roll;
	_RoamAngles.y=pitch;
	_RoamAngles.z=yaw;

	//set mode
	_mountmode=roam;

	//now update mount position
	UpDateMount();
}

//sets the servo angles for landing, note angles are * 100
void AP_Mount::SetMountLanding(int roll, int pitch, int yaw)
{
	_LandingAngles.x=roll;
	_LandingAngles.y=pitch;
	_LandingAngles.z=yaw;

	//set mode
	_mountmode=landing;

	//now update mount position
	UpDateMount();
}

void AP_Mount::SetNone()
{
	//set mode
	_mountmode=none;

	//now update mount position
	UpDateMount();
}

void AP_Mount::UpDateMount()
{
	switch(_mountmode)
	{
		case gps:
			{
				if(_gps->fix) 
				{
					CalcGPSTargetVector(&_targetGPSLocation);
				}
				m = _dcm->get_dcm_transposed();
				targ = m*_GPSVector;
				pitchAngle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				rollAngle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				break;
			}
		case stabilise:
			{
				//to do
				break;
			}
		case roam:
			{
				pitchAngle=100*_RoamAngles.y;
				rollAngle=100*_RoamAngles.x;
				yawAngle=100*_RoamAngles.z;
				break;
			}
		case assisted:
			{
				m = _dcm->get_dcm_transposed();    
				//rotate vector
				targ = m*_AssistVector;
				pitchAngle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				rollAngle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				break;
			}
		case landing:
			{
				pitchAngle=100*_RoamAngles.y;
				rollAngle=100*_RoamAngles.x;
				yawAngle=100*_RoamAngles.z;
				break;
			}
		case none:
			{
				//do nothing
				break;
			}
		default:
			{
				//do nothing
				break;
			}			
	}
}	

void AP_Mount::SetMode(MountMode mode)
{
	_mountmode=mode;
}

void AP_Mount::CalcGPSTargetVector(struct Location *target)
{
	_targetVector.x = (target->lng-_gps->longitude) * cos((_gps->latitude+target->lat)/2)*.01113195;
	_targetVector.y = (target->lat-_gps->latitude)*.01113195;
	_targetVector.z = (_gps->altitude-target->alt);
}
