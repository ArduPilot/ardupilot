#pragma once
// @file    AC_Simulink_AttControlHandle.h
// @brief   This class provides AC_AttitudeControl class instance and enables access to AC_AttitudeControl methods from Simulink-generated code.
#include <AP_Common/AP_Common.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

class AC_Simulink_AttControlHandle
{
public:
    static void initializeInterfaces(AC_AttitudeControl *&att_control);

    AC_AttitudeControl *&getHandle_AC_AttControl() const ;

    // get singleton instance
    static AC_Simulink_AttControlHandle *get_singleton();

    // Functions specific to Simulink blocks.
    // The following functions are invoked by Simulink-generated code when the corresponding blocks are added to the model.
    void init(void);
    // get attitude setpoint in quaternion
    Quaternion getAttitudeSetpointQuat(void);
    // get angular velocity setpoint in quaternion
    Vector3f getAngularVelocitySetpoint(void);

protected:
    // References to external libraries
    AC_AttitudeControl *&_att_control;

private:
    AC_Simulink_AttControlHandle(AC_AttitudeControl *&att_control) ;

    static AC_Simulink_AttControlHandle *_singleton;
};

extern AC_Simulink_AttControlHandle *sl_attControl_handle;
