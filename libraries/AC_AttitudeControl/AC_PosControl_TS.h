#pragma once

#include "AC_PosControl.h"

class AC_PosControl_TS : public AC_PosControl
{
public:

    /// Constructor
    AC_PosControl_TS(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const AP_Motors& motors, AC_AttitudeControl& attitude_control);

    /// init_xy_controller - initialise the xy controller
    ///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
    ///     should be called once whenever significant changes to the position target are made
    ///     this does not update the xy target
    void init_xy_controller(bool init_I_terms) override;
};
