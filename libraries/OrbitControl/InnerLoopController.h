// InnerLoopController.h: interface for the InnerLoopController class.
//
// Christopher Lum
// lum@u.washington.edu
//
// Dai Tsukada
// dat6@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_InnerLoopController_h
#define GUARD_InnerLoopController_h

/*
// generate dll 
#ifdef ORBITCONTROLLERLIBRARY_EXPORTS
#define ORBITCONTROLLERLIBRARY_API __declspec(dllexport) 
#else
#define ORBITCONTROLLERLIBRARY_API __declspec(dllimport) 
#endif
*/

// Standard headers
//#include <string>			//string
//#include <vector>			//vector

// Local header files
#include "ControlSurfaceDeflections.h"		//ControlSurfaceDeflections class

//-------------------------------------------------------------------------------
/// Define InnerLoopController objects which are used to represent a inner loop controller.
///
////
class InnerLoopController {

public:
	///////////// Construction/Destruction ///////////////////////////////

	// default constructor
	InnerLoopController();
	
	// destructor
	virtual ~InnerLoopController();

	///////////// Overload operators /////////////////////////////////////


	///////////// Public interface methods ///////////////////////////////
	ControlSurfaceDeflections computeControl(double psiDotErr, double p, double q, double r,
		double phi, double theta, double uB, double vB, double wB, double rad_ref, double alt_ref, double alt, double dt);
	
	// ====== Get/Set Functions ==========================


	// data members

protected:

	///////////// Protected data members /////////////////////////////////

	// data members

	////
	/// Lateral/directional gains
	////
	double kPhi;   // roll loop forward gain
	double kP;       // roll loop damping gain
	double kR;       // yaw damper gain

	////
	/// Vertical gains
	////
	double kTheta;   // pitch loop forward gain
	double kQ;     // pitch loop damping gain
	double kAlt;   // altitude loop forward gain
	
private:

	///////////// Private data members ///////////////////////////////////

	// integrator terms
	double intYawDamper;
	double intAltitude;

	// previous values for inputs
	double last_psiDotErr;
	double last_p;
	double last_q;
	double last_r;
	double last_phi;
	double last_theta;
	double last_uB;
	double last_vB;
	double last_wB;
	double last_rad_ref;
	double last_alt_ref;
	double last_alt; 
	double last_dt;

};
#endif
