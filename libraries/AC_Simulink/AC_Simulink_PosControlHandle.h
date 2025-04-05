#pragma once
// @file    AC_Simulink_PosControlHandle.h
// @brief   Position Controller handle class for Simulink interface
#include <AP_Common/AP_Common.h>
#include <AC_AttitudeControl/AC_PosControl.h>

class AC_Simulink_PosControlHandle
{
public:
    static void initializeInterfaces(AC_PosControl *&pos_control);

    AC_PosControl *&getHandle_AC_PosControl() const ;

    // get singleton instance
    static AC_Simulink_PosControlHandle *get_singleton();
    
    //Block specific functions
    void init(void);
    Vector3p getPositionSetPointNEU(void);
    Vector3f getVelocitySetPointNEU(void);
protected:
    // References to external libraries
    AC_PosControl *&_pos_control;

private:
    AC_Simulink_PosControlHandle(AC_PosControl *&pos_control) ;

    static AC_Simulink_PosControlHandle *_singleton;
};

extern AC_Simulink_PosControlHandle *sl_posControl_handle;
