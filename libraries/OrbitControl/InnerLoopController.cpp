// InnerLoopController.cpp: implementation of the InnerLoopController class.
//
// Christopher Lum
// lum@u.washington.edu
//
// Dai Tsukada
// dat6@uw.edu
//
//////////////////////////////////////////////////////////////////////

//Version History
//	03/31/15: Created

// standard headers
//#include <iostream>						//cout, endl, cerr
//#include <string>						//string
//#include <vector>						//vector
#include <math.h>						//min, max

// local header files
#include "InnerLoopController.h"		//InnerLoopController class
#include "ControlSurfaceDeflections.h"	//ControlSurfaceDeflections class

// using declaration


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------
////
/// Default constructor.
///
/// Input:			-none
///
/// Output:			-none
///
/// Side-effects:	-none
////
InnerLoopController::InnerLoopController()
{
	kPhi = 1.9;   // roll loop forward gain
	kP = 1;       // roll loop damping gain
	kR = 2;       // yaw damper gain
	kTheta = 3;   // pitch loop forward gain
	kQ = 1.2;     // pitch loop damping gain
	kAlt = 0.2;   // altitude loop forward gain

	// initialize integrators
	intYawDamper = 0;
	intAltitude = 0;

	// initialize previous values for input
	last_psiDotErr = 0;
	last_p = 0;
	last_q = 0;
	last_r = 0;
	last_phi = 0;
	last_theta = 0;
	last_uB = 0;
	last_vB = 0;
	last_wB = 0;
	last_rad_ref = 0;
	last_alt_ref = 0;
	last_alt = 0; 
	last_dt = 0;
}



//-------------------------------------------------------------------------
////
/// Destructor
////
InnerLoopController::~InnerLoopController()
{
}

//////////////////////////////////////////////////////////////////////
// Overloaded operators
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Public interface methods
//////////////////////////////////////////////////////////////////////

/// Compute control input
///	
/// Input:			- psiDotErr = heading rate error computed in outer loop controller (rad/s)
///                 - p         = roll rate (rad/s)
///                 - q         = pitch rate (rad/s)
///                 - r         = yaw rate (rad/s)
///                 - phi       = bank angle (rad)
///                 - theta     = pitch angle (rad)
///                 - uB        = velocity in x-direction (m/s)
///                 - vB        = velocity in y-direction (m/s)
///                 - wB        = velocity in z-direction (m/s)
///                 - rad_ref   = reference radius (m)
///                 - alt_ref   = reference altitude (m)
///                 - alt       = actual altitude (m)
///                 - dt        = sample time (s)
///
/// Output:			- dA = Aileron deflection (rad)
///                 - dE = Elevator deflection (rad)
///                 - dR = Rudder deflection (rad)
///
/// Side-effects:	- none
////
ControlSurfaceDeflections InnerLoopController::computeControl(double psiDotErr, double p, double q, double r, 
		double phi, double theta, double uB, double vB, double wB, double rad_ref, double alt_ref, double alt, double dt)
{
	////
	/// Check input data range (subject to change depending on aircraft specification)
	////
	if (psiDotErr>0.5 || psiDotErr<-0.5)
	{
		// invalid signal from outer loop controller
		//throw std::runtime_error("InnerLoopController Error: Invalid outer loop signal");
		//ControlSurfaceDeflections U = ControlSurfaceDeflections();
		//return(U);

		psiDotErr = last_psiDotErr;
	}
	//if(p>0.9 || p<-0.9 || q>0.9 || q<-0.9|| r>0.5 || r<-0.5 
	//	|| phi>1 || phi<-1 || theta>1.2 || theta<-1.2
	//	|| uB>100 || uB<-100 || vB>100 || vB<-100 || wB>30 || wB<-30
	//	|| alt>6000 || alt<-100)
	//{
	//	// invalid state (inertial measurement) input
	//	//throw std::runtime_error("InnerLoopController Error: Invalid state input");
	//	//ControlSurfaceDeflections U = ControlSurfaceDeflections();
	//	//return(U);
	//}

	// invalid state (inertial measurement) input
	if (p>0.9 || p<-0.9) {
		p = last_p;
	}
	if (q>0.9 || q<-0.9) {
		q = last_q;
	}
	if (r>0.5 || r<-0.5) {
		r = last_r;
	}
	if (phi>1 || phi<-1) {
		phi = last_phi;
	}
	if (theta>1.2 || theta<-1.2) {
		theta = last_theta;
	}
	if (uB>100 || uB<-100) {
		uB = last_uB;
	}
	if (vB>100 || vB<-100) {
		vB = last_vB;
	}
	if (wB>30 || wB<-30) {
		wB = last_wB;
	}
	if (alt>6000 || alt<-100) {
		alt = last_alt;
	}

	if (rad_ref>10000 || rad_ref<0) {
		// invalid reference radius input
		rad_ref = last_rad_ref;
	}
	if (alt_ref>5000 || alt_ref<0) {
		// invalid altitude reference input
		alt_ref = last_alt_ref;
	}
	if (dt == 0) {
		// invalid time step
		dt = last_dt;
	}

	////
	/// Define Constants
	////
	const double g = 9.80665;
	const double pi = 3.14159;

	////
	/// Inner Loop Interface
	////
	double vA = sqrt(uB*uB + vB*vB + wB*wB);
	double psiDot = psiDotErr + vA/rad_ref;
	double r_ref = vA/(rad_ref*cos(phi));
	
	////
	/// Roll Inner Loop
	////
	double phi_e = atan((psiDot*vA) * (1/g)) - phi;
	double dA = -(phi_e*kPhi - p*kP);

	////
	/// Yaw Damper
	////
	double r_e = r_ref - r;
	intYawDamper += (kR/5)*r_e*dt;
	// signal saturation
	if (intYawDamper < -0.7) {
		intYawDamper = -0.7;
	} else if (intYawDamper > 0.7) {
		intYawDamper = 0.7;
	}
	double dR = -(r_e*kR + intYawDamper);

	////
	/// Altitude Loop
	////
	double alt_e = alt_ref - alt;
	intAltitude += (kAlt/15)*alt_e*dt;
	// signal saturation
	if (intAltitude < -60) {
		intAltitude = -60;
	} else if (intAltitude > 60) {
		intAltitude = 60;
	}
	double theta_cmd = (alt_e*kAlt + intAltitude) * (pi/180);

	////
	/// Pitch Inner Loop
	////
	double theta_e = theta_cmd - theta;
	double dE = -(theta_e*kTheta - q*kQ);

	// save input information
	last_psiDotErr = psiDotErr;
	last_p = p;
	last_q = q;
	last_r = r;
	last_phi = phi;
	last_theta = theta;
	last_uB = uB;
	last_vB = vB;
	last_wB = wB;
	last_rad_ref = rad_ref;
	last_alt_ref = alt_ref;
	last_alt = alt;
	last_dt = dt;


	// instantiate control surface deflection object (data type to store control inputs)
	ControlSurfaceDeflections U = ControlSurfaceDeflections();

	U.SetAileron(dA);
	U.SetRudder(dR);
	U.SetElevator(dE);

	return(U);
}
	

//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================
