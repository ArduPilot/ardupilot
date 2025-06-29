#pragma once
// @file    AC_Simulink_PosControlHandle.h
// @brief   This class provides AC_PosControl class instance and enables access to AC_PosControl methods from Simulink-generated code.
#include <AP_Common/AP_Common.h>
#include <AC_AttitudeControl/AC_PosControl.h>

class AC_Simulink_PosControlHandle
{
public:
    static void initializeInterfaces(AC_PosControl *&pos_control);

    AC_PosControl *&getHandle_AC_PosControl() const ;

    // get singleton instance
    static AC_Simulink_PosControlHandle *get_singleton();

    // Functions specific to Simulink blocks.
    // The following functions are invoked by Simulink-generated code when the corresponding blocks are added to the model.
    void init(void);
    // get position setpoint in NEU frame
    Vector3p getPositionSetPointNEU(void);
    // get velocity setpoint in NEU frame
    Vector3f getVelocitySetPointNEU(void);
protected:
    // References to external libraries
    AC_PosControl *&_pos_control;

private:
    AC_Simulink_PosControlHandle(AC_PosControl *&pos_control) ;

    static AC_Simulink_PosControlHandle *_singleton;
};

extern AC_Simulink_PosControlHandle *sl_posControl_handle;
