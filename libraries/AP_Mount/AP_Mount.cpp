#include "AP_Mount.h"

AP_Mount::AP_Mount(GPS *gps, AP_DCM *dcm)
{
	_dcm=dcm;
	_gps=gps;
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
	_targetVector.x = roll;
	_targetVector.y = pitch;
	_targetVector.z = yaw;

	//set mode
	_mountmode=assisted;

	//update mount position
	UpDateMount();
}

void AP_Mount::SetAntenna(Location grndStation)
{
	_grndStation=grndStation;
	
	//set mode
	_mountmode=antenna;

	//update mount position
	UpDateMount();
}


//sets the servo angles for FPV, note angles are * 100
void AP_Mount::SetMountFPV(int roll, int pitch, int yaw)
{
	_mountFPVVector.x=roll;
	_mountFPVVector.y=pitch;
	_mountFPVVector.z=yaw;

	//set mode
	_mountmode=fpv;

	//now update mount position
	UpDateMount();
}

//sets the servo angles for landing, note angles are * 100
void AP_Mount::SetMountLanding(int roll, int pitch, int yaw)
{
	_mountLanding.x=roll;
	_mountLanding.y=pitch;
	_mountLanding.z=yaw;

	//set mode
	_mountmode=landing;

	//now update mount position
	UpDateMount();
}

void AP_Mount::UpDateMount()
{
	switch(_mountmode)
	{
		case fpv:
			{
				pitchAngle=100*_mountFPVVector.y;
				rollAngle=100*_mountFPVVector.x;
				break;
			}
		case assisted:
			{
				Matrix3f m = _dcm->get_dcm_transposed();    
				//rotate vector
				//to do: find out notion of x y convention
				Vector3<float> targ = m*_targetVector;
				pitchAngle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				rollAngle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				break;
			}
		case gps:
			{
				if(_gps->fix) 
				{
					CalcGPSTargetVector(&_targetGPSLocation);
				}
				Matrix3f m = _dcm->get_dcm_transposed();
				Vector3<float> targ = m*_targetVector;
				pitchAngle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				rollAngle =   degrees(atan2(targ.y,targ.z))*100;	//roll
				break;
			}
		case antenna:
			{
				if(_gps->fix) 
				{
					CalcGPSTargetVector(&_grndStation);
				}
				Matrix3f m = _dcm->get_dcm_transposed();
				Vector3<float> targ = m*_targetVector;
				pitchAngle =  degrees(atan2(-targ.x,targ.z))*100;	//pitch 
				rollAngle =   degrees(atan2(targ.y,targ.z))*100;	//roll
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
