#pragma once
// @file    AC_Simulink_AHRSHandle.h
// @brief   This class provides AP_AHRS_View class instance and enables access to AP_AHRS_View methods from Simulink-generated code.
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS_View.h>

class AC_Simulink_AHRS
{
public:
    static void initializeInterfaces(AP_AHRS_View *&ahrs);

    AP_AHRS_View *&getHandle_AP_AHRS_View() const ;

    // get singleton instance
    static AC_Simulink_AHRS *get_singleton();

    // Functions specific to Simulink blocks.
    // The following functions are invoked by Simulink-generated code when the corresponding blocks are added to the model.

    void init(void);
    // get estimated angular velocity
    Vector3f getAngularVelocity();
    // get estimated attitude in Quaternion
    void getAttitudeQuat(Quaternion &attQuat);
    // get estimated position in NED frame
    bool getPositionNED(Vector3f &posNED);
    // get estimated velocity in NED frame
    bool getVelocityNED(Vector3f &velNED);

protected:
    // References to external libraries
    AP_AHRS_View *&_ahrs;

private:
    AC_Simulink_AHRS(AP_AHRS_View *&ahrs) ;

    static AC_Simulink_AHRS *_singleton;
};

extern AC_Simulink_AHRS *sl_ahrs_handle;
