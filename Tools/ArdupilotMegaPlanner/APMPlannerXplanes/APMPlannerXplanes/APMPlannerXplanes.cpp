#include <string.h>
#include <math.h>
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMCamera.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#if LIN
#include <GL/gl.h>
#else
#if __GNUC__
#include <OpenGL/gl.h>
#else
#include <windows.h>
#include <..\..\..\..\..\..\..\program files (x86)\microsoft sdks\windows\v7.0a\include\gl\gl.h>
#endif
#endif
#include "AircraftUtils.h"

#include <stdio.h>

using namespace System;
using namespace System::Net;
using namespace System::Net::Sockets;

public ref class Globals abstract sealed {
public:
   static IPEndPoint^ RemoteIpEndPoint = gcnew IPEndPoint( IPAddress::Any, 4900 );
   static Socket^ XplanesSEND = gcnew Socket(AddressFamily::InterNetwork,
                            SocketType::Dgram, ProtocolType::Udp);
};

XPLMDataRef		gPlaneX = NULL;
XPLMDataRef		gPlaneY = NULL;
XPLMDataRef		gPlaneZ = NULL;
XPLMDataRef		gPlaneTheta = NULL;
XPLMDataRef		gPlanePhi = NULL;
XPLMDataRef		gPlanePsi = NULL;
XPLMDataRef		gOverRidePlanePosition = NULL;
XPLMDataRef		gAGL = NULL;
XPLMDataRef		gLatRef = NULL;
XPLMDataRef		gLongRef = NULL;

float fTextColour[3];
char szString[100];

const	double	kMaxPlaneDistance = 5280.0 / 3.2 * 10.0;
const	double	kFullPlaneDist = 5280.0 / 3.2 * 3.0;

static	inline float sqr(float a) { return a * a; }

static	inline	float	CalcDist3D(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return sqrt(sqr(x2-x1) + sqr(y2-y1) + sqr(z2-z1));
}

const	double	kFtToMeters = 0.3048;

static	XPLMDataRef	gFOVDataRef = NULL;

int	AircraftDrawCallback(	XPLMDrawingPhase     inPhase,    
                            int                  inIsBefore,    
                            void *               inRefcon);

PLUGIN_API int XPluginStart(	char *		outName,
								char *		outSig,
								char *		outDesc)
{
	int	planeCount;
	XPLMCountAircraft(&planeCount, 0, 0);

	fTextColour[0] = 1.0;
	fTextColour[1] = 1.0;
	fTextColour[2] = 1.0;


	strcpy(outName, "Quad Hil");
	strcpy(outSig, "based on xplanesdk.examples.drawaircraft");
	strcpy(outDesc, "A plugin that receives raw data to display");

	/* Prefetch the sim variables we will use. */
	gPlaneX = XPLMFindDataRef("sim/flightmodel/position/local_x");
	gPlaneY = XPLMFindDataRef("sim/flightmodel/position/local_y");
	gPlaneZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
	gPlaneTheta = XPLMFindDataRef("sim/flightmodel/position/theta");
	gPlanePhi = XPLMFindDataRef("sim/flightmodel/position/phi");
	gPlanePsi = XPLMFindDataRef("sim/flightmodel/position/psi");
	gOverRidePlanePosition = XPLMFindDataRef("sim/operation/override/override_planepath");
	gAGL = XPLMFindDataRef("sim/flightmodel/position/y_agl");
	gLatRef = XPLMFindDataRef("sim/flightmodel/position/lat_ref");
	gLongRef = XPLMFindDataRef("sim/flightmodel/position/lon_ref");


	/* Next register the drawing callback.  We want to be drawn
	 * after X-Plane draws its 3-d objects. */
	XPLMRegisterDrawCallback(
					AircraftDrawCallback,
					xplm_Phase_Window, 	/* Draw when sim is doing objects */
					0,						/* After objects */
					NULL);					/* No refcon needed */

	Globals::XplanesSEND->Bind(Globals::RemoteIpEndPoint);
/*
	char FileName[256], AircraftPath[256];

	for (long index = 0; index < planeCount; ++index)
	{
		XPLMGetNthAircraftModel(index, FileName, AircraftPath);
		pAircraft[index] = (char *)AircraftPath;

		if (XPLMAcquirePlanes((char **)&pAircraft, NULL, NULL))
			XPLMSetAircraftModel(index, AircraftPath);
	}
*/

	return 1;
}


PLUGIN_API void	XPluginStop(void)
{
}

PLUGIN_API void XPluginDisable(void)
{
		int Enable[10];
		Enable[0] = 0;
		
		XPLMSetDatavi(gOverRidePlanePosition,&Enable[0],0,1); // disable physics
}

PLUGIN_API int XPluginEnable(void)
{
		int Enable[10];
		Enable[0] = 1;
		
		XPLMSetDatavi(gOverRidePlanePosition,&Enable[0],0,1); // disable physics

	return 1;
}

PLUGIN_API void XPluginReceiveMessage(	XPLMPluginID	inFromWho,
										long			inMessage,
										void *			inParam)
{
}


/*
 * AircraftDrawCallback
 *
 * This is the actual drawing callback that does the work; for us it will
 * be called after X-Plane has drawn its 3-d objects.  The coordinate system
 * is 'normal' for 3-d drawing, meaning 0,0,0 is at the earth's surface at the
 * lat/lon reference point, with +Y = up, +Z = South, +X = East.  (Note that
 * these relationships are only true at 0,0,0 due to the Earth's curvature!!)
 *
 * Drawing hooks that draw before X-Plane return 1 to let X-Plane draw or 0
 * to inhibit drawing.  For drawing hooks that run after X-Plane, this return
 * value is ignored but we will return 1 anyway.
 *
 */

public struct quad
{
	double t1;
	double t2;
	double t3;
	double t4;

	double lat;
	double lng;
	double alt;
	double altagl;

	double roll;
	double pitch;
	double yaw;
};

int	AircraftDrawCallback(	XPLMDrawingPhase     inPhase,
                            int                  inIsBefore,
                            void *               inRefcon)
{
	if (Globals::XplanesSEND->Available > 0) {
		
		array<Byte>^bytesReceived = gcnew array<Byte>(256);
		int bytes = 0;
		bytes = Globals::XplanesSEND->Receive( bytesReceived, bytesReceived->Length, static_cast<SocketFlags>(0) );

		int count = 0;
		double t1 = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double t2 = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double t3 = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double t4 = BitConverter::ToDouble(bytesReceived, count * 8);

		count++;
		double lat = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double lng = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double alt = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double altagl = BitConverter::ToDouble(bytesReceived, count * 8);

		count++;
		double roll = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double pitch = BitConverter::ToDouble(bytesReceived, count * 8);
		count++;
		double yaw = BitConverter::ToDouble(bytesReceived, count * 8);

		double X,Y,Z;

		XPLMSetDataf(gLatRef,(float)lat);
		XPLMSetDataf(gLongRef,(float)lng);


		XPLMWorldToLocal(lat,lng,alt * kFtToMeters,&X,&Y,&Z);

			XPLMSetDataf(gPlaneX, X);
			XPLMSetDataf(gPlaneY, Y);
			XPLMSetDataf(gPlaneZ, Z);

			
			XPLMSetDataf(gPlaneTheta, pitch);
			XPLMSetDataf(gPlanePhi, roll);
			XPLMSetDataf(gPlanePsi, yaw);

			sprintf(szString,"m1: %f, m2: %f, m3: %f, m4: %f\0", t1,t2,t3,t4);
	XPLMDrawString(fTextColour,700,700,szString,NULL,xplmFont_Basic);


			return 1;

	} else {
		return 1;
	}
	

	double	x,y,z,theta,phi,psi;

	//double Lat = 34.09, Lon = -117.25, Alt = 1170;
	float Heading = 0, Pitch = 0, Roll = 0, Altitude;

	x = XPLMGetDataf(gPlaneX);
	y = XPLMGetDataf(gPlaneY);
	z = XPLMGetDataf(gPlaneZ);
	theta = XPLMGetDataf(gPlaneTheta);
	phi = XPLMGetDataf(gPlanePhi);
	psi = XPLMGetDataf(gPlanePsi);
	Altitude = XPLMGetDataf(gAGL);


	XPLMSetDataf(gPlaneX, x + 1);

	return 1;
}
