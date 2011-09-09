#include "AircraftUtils.h"
#include <string.h>

Aircraft::Aircraft(int AircraftNo)
{
	char	x_str[80];
	char	y_str[80];
	char	z_str[80];
	char	the_str[80];
	char	phi_str[80];
	char	psi_str[80];
	char	gear_deploy_str[80];
	char	throttle_str[80];

	strcpy(x_str, "sim/multiplayer/position/planeX_x");
	strcpy(y_str,	"sim/multiplayer/position/planeX_y");
	strcpy(z_str,	"sim/multiplayer/position/planeX_z");
	strcpy(the_str,	"sim/multiplayer/position/planeX_the");
	strcpy(phi_str,	"sim/multiplayer/position/planeX_phi");
	strcpy(psi_str,	"sim/multiplayer/position/planeX_psi");
	strcpy(gear_deploy_str,	"sim/multiplayer/position/planeX_gear_deploy");
	strcpy(throttle_str, "sim/multiplayer/position/planeX_throttle");

	char cTemp = (AircraftNo + 0x30);
	x_str[30]			=	cTemp;
	y_str[30]			=	cTemp;
	z_str[30]			=	cTemp;
	the_str[30]			=	cTemp;
	phi_str[30]			=	cTemp;
	psi_str[30]			=	cTemp;
	gear_deploy_str[30] =	cTemp;
	throttle_str[30]	=	cTemp;

	dr_plane_x				= XPLMFindDataRef(x_str);
	dr_plane_y				= XPLMFindDataRef(y_str);
	dr_plane_z				= XPLMFindDataRef(z_str);
	dr_plane_the			= XPLMFindDataRef(the_str);
	dr_plane_phi			= XPLMFindDataRef(phi_str);
	dr_plane_psi			= XPLMFindDataRef(psi_str);
	dr_plane_gear_deploy	= XPLMFindDataRef(gear_deploy_str);
	dr_plane_throttle		= XPLMFindDataRef(throttle_str);
}

void Aircraft::GetAircraftData(void)
{
	plane_x = XPLMGetDataf(dr_plane_x);
	plane_y = XPLMGetDataf(dr_plane_y);
	plane_z = XPLMGetDataf(dr_plane_z);
	plane_the = XPLMGetDataf(dr_plane_the);
	plane_phi = XPLMGetDataf(dr_plane_phi);
	plane_psi = XPLMGetDataf(dr_plane_psi);
	XPLMGetDatavf(dr_plane_gear_deploy, plane_gear_deploy, 0, 5);
	XPLMGetDatavf(dr_plane_throttle, plane_throttle, 0, 8);
}

void Aircraft::SetAircraftData(void)
{
	XPLMSetDataf(dr_plane_x, plane_x);
	XPLMSetDataf(dr_plane_y, plane_y);
	XPLMSetDataf(dr_plane_z, plane_z);
	XPLMSetDataf(dr_plane_the, plane_the);
	XPLMSetDataf(dr_plane_phi, plane_phi);
	XPLMSetDataf(dr_plane_psi, plane_psi);
	XPLMSetDatavf(dr_plane_gear_deploy, plane_gear_deploy, 0, 5);
	XPLMSetDatavf(dr_plane_throttle, plane_throttle, 0, 8);
}

