#include "AP_Mount.h"

AP_Mount::AP_Mount(GPS *gps, AP_DCM *dcm, AP_Var::Key key, const prog_char_t *name):
_group(key, name),
_mountMode(&_group, 0, 0, name ? PSTR("MODE")         : 0), // suppress name if group has no name
_mountType(&_group, 0, 0, name ? PSTR("TYPE")         : 0),
_dcm(dcm);
_gps(gps);
{

}

void AP_Mount::SetPitchYaw(Location targetGPSLocation)
{
	_targetGPSLocation=targetGPSLocation;
	
	//set mode
	_mountMode=gps;

	//update mount position
	UpDateMount();
}

void AP_Mount::SetGPSTarget(Location targetGPSLocation)
{
	_targetGPSLocation=targetGPSLocation;
	
	//set mode
	_mountMode=gps;

	//update mount position
	UpDateMount();
}

void AP_Mount::SetAssisted(int roll, int pitch, int yaw)
{
	_AssistAngles.x = roll;
	_AssistAngles.y = pitch;
	_AssistAngles.z = yaw;

	//set mode
	_mountMode=assisted;

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
	_mountMode=roam;

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
	_mountMode=landing;

	//now update mount position
	UpDateMount();
}

void AP_Mount::SetNone()
{
	//set mode
	_mountMode=none;

	//now update mount position
	UpDateMount();
}

void AP_Mount::UpDateMount()
{
	switch(_mountMode)
	{
		case gps:
			{
				if(_gps->fix) 
				{
					CalcGPSTargetVector(&_targetGPSLocation);
				}
				m = _dcm->get_dcm_transposed();
				targ = m*_GPSVector;
				this->CalcMountAnglesFromVector(*targ)
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
				targ = m*_AssistVector;
				this->CalcMountAnglesFromVector(*targ)
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
	_mountMode=mode;
}

void AP_Mount::CalcGPSTargetVector(struct Location *target)
{
	_targetVector.x = (target->lng-_gps->longitude) * cos((_gps->latitude+target->lat)/2)*.01113195;
	_targetVector.y = (target->lat-_gps->latitude)*.01113195;
	_targetVector.z = (_gps->altitude-target->alt);
}

void AP_Mount::CalcMountAnglesFromVector(Vector3f *targ)
{
	switch(_mountType)
	{
		case pitch_yaw:
			{
				//need to tidy up maths for below
				pitchAngle = atan2((sqrt(sq(targ.y) + sq(targ.x)) * .01113195), targ.z) * -1;
				yawAngle = 9000 + atan2(-targ.y, targ.x) * 5729.57795;
				break;
			}
		case pitch_roll:
			{
				pitchAngle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				rollAngle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				break;
			}			
		case pitch_roll_yaw:
			{
				//to do
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
