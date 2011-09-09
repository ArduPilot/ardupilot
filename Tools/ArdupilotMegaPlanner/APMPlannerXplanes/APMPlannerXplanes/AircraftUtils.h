#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"

class Aircraft
{
private:
	XPLMDataRef	dr_plane_x;
	XPLMDataRef	dr_plane_y;
	XPLMDataRef	dr_plane_z;
	XPLMDataRef	dr_plane_the;
	XPLMDataRef	dr_plane_phi;
	XPLMDataRef	dr_plane_psi;
	XPLMDataRef	dr_plane_gear_deploy;
	XPLMDataRef	dr_plane_throttle;
public:
	float		plane_x;
	float		plane_y;
	float		plane_z;
	float		plane_the;
	float		plane_phi;
	float		plane_psi;
	float		plane_gear_deploy[5];
	float		plane_throttle[8];
	Aircraft(int AircraftNo);
	void GetAircraftData(void);
	void SetAircraftData(void);
};
