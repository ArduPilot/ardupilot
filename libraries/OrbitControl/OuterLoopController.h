// OuterLoopController.h: interface for the OuterLoopController class.
//
// Christopher Lum
// lum@u.washington.edu
//
// Dai Tsukada
// dat6@uw.edu
//
//////////////////////////////////////////////////////////////////////

#ifndef GUARD_OuterLoopController_h
#define GUARD_OuterLoopController_h

/*
// generate dll 
#ifdef ORBITCONTROLLERLIBRARY_EXPORTS
#define ORBITCONTROLLERLIBRARY_API __declspec(dllexport) 
#else
#define ORBITCONTROLLERLIBRARY_API __declspec(dllimport) 
#endif
*/

// Standard headers
//#include <string>         //string
//#include <vector>         //vector

// Local header files

//-------------------------------------------------------------------------------
/// Define OuterLoopController objects which are used to represent a outer loop controller.
///
////
class OuterLoopController {

public:
    ///////////// Construction/Destruction ///////////////////////////////

    // default constructor
    OuterLoopController();

    // destructor
    virtual ~OuterLoopController();

    ///////////// Overload operators /////////////////////////////////////


    ///////////// Public interface methods ///////////////////////////////
    double computeOuterLoopSignal(double rad_act, double rad_ref);
    double activateController();

    // ====== Get/Set Functions ==========================


    // data members

protected:

    ///////////// Protected data members /////////////////////////////////

    ////
    /// time constant of low pass filter (fade-in activation gain)
    ////
    double fadein_tau;

    ////
    /// proportional gain for outer loop controller
    ////
    double Kp_outer;

private:

    ///////////// Private data members ///////////////////////////////////

    ////
    /// gain fader value
    ////
    double fader;

    ////
    /// previous value of gain fader
    ////
    double last_fader;

    ////
    /// previous value of reference radius
    ////
    double last_rad_ref;

    ////
    /// previous value of actual radius
    ////
    double last_rad_act;

    ////
    /// radius derivative values
    ////
    double dr;
    double dr_1;
    double dr_2;
    double dr_3;
    double dr_4;
    double dr_5;
    int dr_count;

};
#endif
