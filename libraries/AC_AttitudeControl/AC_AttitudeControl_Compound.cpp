#include "AC_AttitudeControl_Multi.h"
#include "AC_AttitudeControl_Compound.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID.h>

void AC_AttitudeControl_Compound::set_radio_passthrough_auxiliary_thruster()
{

}

// To-Do : Use Forward Thruster when certain amount of accel_forward is requested.
void AC_AttitudeControl_Compound::accel_to_thrust()
{
  AC_PosControl::accel_to_lean_angles(float dt, float ekfNavVelGainScaler, bool use_althold_lean_angle);

    if (accel_forward > 1.0f)
    {
      _pitch_target = 0.0f;
      run_auxiliary_thruster_controller();
    }
    else
    {
      _thrust_target = 0.0f;
    }
}

/*
void AC_AttitudeControl_Compound::lean_angles_and_thrust_to_accel()
{
  AC_PosControl::lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss);

}
*/

void AC_AttitudeControl_Compound::run_auxiliary_thruster_controller()
{
      _thrust_target = accel_forward;
      Vector3f accel_NED = ahrs.get_accel_ef_blended();
      //To-do : Implement accel forward to throttle controller like fixed-wing
      motors.set_foward_in(_thrust_out);
  }
}
