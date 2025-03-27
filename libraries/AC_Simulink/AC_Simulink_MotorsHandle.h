#pragma once
// @file    AC_Simulink_MotorsHandle.h
// @brief   Motor handle class for Simulink interface
#include <AP_Common/AP_Common.h>
#include <AP_Motors/AP_MotorsMulticopter.h>

class AC_Simulink_MotorsHandle
{
public:
    static void initializeInterfaces(AP_MotorsMulticopter *&motors);

    AP_MotorsMulticopter *&getHandle_AP_Motors() const ;

    // get singleton instance
    static AC_Simulink_MotorsHandle *get_singleton();
    
    //Block specific functions
    void init(void);
    void setTorque(float tauRoll,float tauPitch,float tauYaw);
    void setThrust(float throttle);

protected:
    // References to external libraries
    AP_MotorsMulticopter *&_motors;

private:
    AC_Simulink_MotorsHandle(AP_MotorsMulticopter *&motors) ;

    static AC_Simulink_MotorsHandle *_singleton;
};

extern AC_Simulink_MotorsHandle *sl_motors_handle;
