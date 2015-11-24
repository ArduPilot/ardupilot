/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>

/*
  QuadPlane specific functionality
 */
class QuadPlane
{
public:
    friend class Plane;
    QuadPlane(AP_AHRS_NavEKF &_ahrs);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // setup quadplane
    void setup(void);
    
    // stabilize in hover mode
    void stabilize_hover(void);

    // update transition handling
    void update(void);

    // set motor arming
    void set_armed(bool armed);
    
private:
    AP_AHRS_NavEKF &ahrs;
    AP_Vehicle::MultiCopter aparm;
    AP_MotorsQuad motors{50};
    AC_PID        pid_rate_roll {0.15, 0.1, 0.004,  2000, 20, 0.02};
    AC_PID        pid_rate_pitch{0.15, 0.1, 0.004,  2000, 20, 0.02};
    AC_PID        pid_rate_yaw  {0.15, 0.1, 0.004,  2000, 20, 0.02};
    AC_P          p_stabilize_roll{4.5};
    AC_P          p_stabilize_pitch{4.5};
    AC_P          p_stabilize_yaw{4.5};

    AC_AttitudeControl_Multi attitude_control{ahrs, aparm, motors,
            p_stabilize_roll, p_stabilize_pitch, p_stabilize_yaw,
            pid_rate_roll, pid_rate_pitch, pid_rate_yaw};

    AP_InertialNav_NavEKF inertial_nav{ahrs};

    AC_P                    p_pos_xy{1};
    AC_P                    p_alt_hold{1};
    AC_P                    p_vel_z{5};
    AC_PID                  pid_accel_z{0.5, 1, 0, 800, 20, 0.02};
    AC_PI_2D                pi_vel_xy{1.0, 0.5, 1000, 5, 0.02};
    
    AC_PosControl pos_control{ahrs, inertial_nav, motors, attitude_control,
            p_alt_hold, p_vel_z, pid_accel_z,
            p_pos_xy, pi_vel_xy};

    // update transition handling
    void update_transition(void);
    
    AP_Int16 transition_time_ms;
    
    // last time quadplane was active, used for transition
    uint32_t last_run_ms;

    // last throttle value when active
    int16_t  last_throttle;

    const float smoothing_gain = 6;
};
