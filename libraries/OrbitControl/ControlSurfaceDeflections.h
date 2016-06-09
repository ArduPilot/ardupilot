// ControlSurfaceDeflections.h: interface for the ControlSurfaceDeflections class.
//
// Christopher Lum
// lum@u.washington.edu
//
// Dai Tsukada
// dat6@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_ControlSurfaceDeflections_h
#define GUARD_ControlSurfaceDeflections_h

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

//-------------------------------------------------------------------------------
/// Define ControlSurfaceDeflections objects which are used to represent a control surface deflections.
///
////
class ControlSurfaceDeflections {

public:
	///////////// Construction/Destruction ///////////////////////////////

	// default constructor
	ControlSurfaceDeflections();
	
	// destructor
	virtual ~ControlSurfaceDeflections();

	///////////// Overload operators /////////////////////////////////////


	///////////// Public interface methods ///////////////////////////////
	

	// ====== Get/Set Functions ==========================
	double GetAileron(void) const;
	void SetAileron(double);

	double GetElevator(void) const;
	void SetElevator(double);

	double GetRudder(void) const;
	void SetRudder(double);
	
	// data members

protected:

	///////////// Protected data members /////////////////////////////////

	// data members
	
	////
	/// aileron (angle in radians)
	////
	double	aileron;

	////
	/// elevator (angle in radians)
	////
	double	elevator;

	////
	/// rudder (angle in radians)
	////
	double	rudder;

	
private:

	///////////// Private data members ///////////////////////////////////

};
#endif
