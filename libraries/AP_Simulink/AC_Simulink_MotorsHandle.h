#pragma once
// @file    AC_Simulink_MotorsHandle.h
// @brief   This class provides AP_MotorsMulticopter class instance and enables access to AP_MotorsMulticopter methods from Simulink-generated code.
#include <AP_Common/AP_Common.h>
#include <AP_Motors/AP_MotorsMulticopter.h>

class AC_Simulink_MotorsHandle
{
public:
    static void initializeInterfaces(AP_MotorsMulticopter *&motors);

    AP_MotorsMulticopter *&getHandle_AP_Motors() const ;

    // get singleton instance
    static AC_Simulink_MotorsHandle *get_singleton();

    // Functions specific to Simulink blocks.
    // The following functions are invoked by Simulink-generated code when the corresponding blocks are added to the model.
    void init(void);
    // set roll, pitch and yaw torue setpoint
    void setTorque(float tauRoll,float tauPitch,float tauYaw);
    // set vertical thrust
    void setThrust(float throttle);

protected:
    // References to external libraries
    AP_MotorsMulticopter *&_motors;

private:
    AC_Simulink_MotorsHandle(AP_MotorsMulticopter *&motors) ;

    static AC_Simulink_MotorsHandle *_singleton;
};

extern AC_Simulink_MotorsHandle *sl_motors_handle;
