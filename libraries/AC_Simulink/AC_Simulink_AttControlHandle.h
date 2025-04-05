#pragma once
// @file    AC_Simulink_AttControlHandle.h
// @brief   Attitude Controller handle class for Simulink interface
#include <AP_Common/AP_Common.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

class AC_Simulink_AttControlHandle
{
public:
    static void initializeInterfaces(AC_AttitudeControl *&att_control);

    AC_AttitudeControl *&getHandle_AC_AttControl() const ;

    // get singleton instance
    static AC_Simulink_AttControlHandle *get_singleton();
    
    //Block specific functions
    void init(void);
    Quaternion getAttitudeSetpointQuat(void);
    Vector3f getAngularVelocitySetpoint(void);

protected:
    // References to external libraries
    AC_AttitudeControl *&_att_control;

private:
    AC_Simulink_AttControlHandle(AC_AttitudeControl *&att_control) ;

    static AC_Simulink_AttControlHandle *_singleton;
};

extern AC_Simulink_AttControlHandle *sl_attControl_handle;
