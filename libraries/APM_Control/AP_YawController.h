#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AC_PID/AC_PID.h>

class AP_YawController
{
public:
    AP_YawController(const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _pid_info.target = 0;
        _pid_info.FF = 0;
        _pid_info.P = 0;
    }

    /* Do not allow copies */
    AP_YawController(const AP_YawController &other) = delete;
    AP_YawController &operator=(const AP_YawController&) = delete;

    // return true if rate control is enabled
    bool rate_control_enabled(void) const { return _rate_enable != 0; }

    // get actuator output for sideslip and yaw damping control
    int32_t get_servo_out(float scaler, bool disable_integrator);

    // get actuator output for direct rate control
    // desired_rate is in deg/sec. scaler is the surface speed scaler
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator);

    void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

    const AP_Logger::PID_Info& get_pid_info(void) const
    {
        return _pid_info;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_Vehicle::FixedWing &aparm;
    AP_Float _K_A;
    AP_Float _K_I;
    AP_Float _K_D;
    AP_Float _K_FF;
    AP_Int16 _imax;
    AP_Int8  _rate_enable;
    AC_PID rate_pid{0.08, 0.3, 0, 0.3, 0.666, 3, 0, 12, 0.02, 150, 1};

    uint32_t _last_t;
    float _last_out;
    float _last_rate_hp_out;
    float _last_rate_hp_in;
    float _K_D_last;

    float _integrator;

    AP_Logger::PID_Info _pid_info;
};
