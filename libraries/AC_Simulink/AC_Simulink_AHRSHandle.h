#pragma once
// @file    AC_Simulink_AHRSHandle.h
// @brief   AHRS handle class for Simulink interface
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS_View.h>

class AC_Simulink_AHRS
{
public:
    static void initializeInterfaces(AP_AHRS_View *&ahrs);

    AP_AHRS_View *&getHandle_AP_AHRS_View() const ;

    // get singleton instance
    static AC_Simulink_AHRS *get_singleton();
    
    //Block specific functions
    void init(void);
    Vector3f getAngularVelocity();
    void getAttitudeQuat(Quaternion &attQuat);
    bool getPositionNED(Vector3f &posNED);
    bool getVelocityNED(Vector3f &velNED);

protected:
    // References to external libraries
    AP_AHRS_View *&_ahrs;

private:
    AC_Simulink_AHRS(AP_AHRS_View *&ahrs) ;

    static AC_Simulink_AHRS *_singleton;
};

extern AC_Simulink_AHRS *sl_ahrs_handle;
