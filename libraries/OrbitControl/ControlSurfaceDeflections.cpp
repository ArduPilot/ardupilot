// ControlSurfaceDeflections.cpp: implementation of the ControlSurfaceDeflections class.
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
#include "ControlSurfaceDeflections.h"		//ControlSurfaceDeflections class

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

ControlSurfaceDeflections::ControlSurfaceDeflections()
{
	// default settings
	aileron = 0;
	elevator = 0;
	rudder = 0;
}


//-------------------------------------------------------------------------
////
/// Destructor
////
ControlSurfaceDeflections::~ControlSurfaceDeflections()
{
}

//////////////////////////////////////////////////////////////////////
// Overloaded operators
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Public interface methods
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// Get/Set Functions
//////////////////////////////////////////////////////////////////////


//-------------------------------------------------------------------------
////
/// Get the aileron deflection
///	
/// Input:			-none
///
/// Output:			-aileron:	aileron deflection (rad)
///
/// Side-effects:	-none
////
double ControlSurfaceDeflections::GetAileron(void) const
{
	return(aileron);
}

void ControlSurfaceDeflections::SetAileron(double val)
{
	//check if this is in a reasonable range (30deg > val > -30deg)	
	if (val < -0.5236) {
		aileron = -0.5236;
	}
	else if (val > 0.5236) {
		aileron = 0.5236;
	}
	else {
		aileron = val;
	}
}

/// Get the elevator deflection
///	
/// Input:			-none
///
/// Output:			-elevator:	elevetor deflection (rad)
///
/// Side-effects:	-none
////
double ControlSurfaceDeflections::GetElevator(void) const
{
	return(elevator);
}

void ControlSurfaceDeflections::SetElevator(double val)
{
	//check if this is in a reasonable range (30deg > val > -30deg)	
	if (val < -0.5236) {
		elevator = -0.5236;
	}
	else if (val > 0.5236) {
		elevator = 0.5236;
	}
	else {
		elevator = val;
	}
}

/// Get the rudder deflection
///	
/// Input:			-none
///
/// Output:			-rudder:	rudder deflection (rad)
///
/// Side-effects:	-none
////
double ControlSurfaceDeflections::GetRudder(void) const
{
	return(rudder);
}

void ControlSurfaceDeflections::SetRudder(double val)
{
	//check if this is in a reasonable range (30deg > val > -30deg)	
	if (val < -0.5236) {
		rudder = -0.5236;
	}
	else if (val > 0.5236) {
		rudder = 0.5236;
	}
	else {
		rudder = val;
	}
}


//////////////////////////////////////////////////////////////////////
// Global functions
//////////////////////////////////////////////////////////////////////

//==================== End of File =================================
