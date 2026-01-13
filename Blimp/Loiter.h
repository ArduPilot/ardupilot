#pragma once

class Vector4b
{
public:
    bool    x;
    bool    y;
    bool    z;
    bool    yaw;

    constexpr Vector4b()
        : x(0)
        , y(0)
        , z(0)
        , yaw(0) {}

    constexpr Vector4b(const bool x0, const bool y0, const bool z0, const bool yaw0)
        : x(x0)
        , y(y0)
        , z(z0)
        , yaw(yaw0) {}

    Vector4b operator &&(const Vector4b &v)
    {
        Vector4b temp{x && v.x, y && v.y, z && v.z, yaw && v.yaw};
        return temp;
    }

    Vector4b operator ||(const Vector4b &v)
    {
        Vector4b temp{x || v.x, y || v.y, z || v.z, yaw || v.yaw};
        return temp;
    }

};



class Loiter
{
public:
    friend class Blimp;
    friend class Fins;

    float scaler_x;
    float scaler_y;
    float scaler_z;
    float scaler_yaw;
    float lvl_scaler_rll;
    float lvl_scaler_pit;

    float targ_dist;

    //constructor
    Loiter(uint16_t loop_rate)
    {
        scaler_x = 1;
        scaler_y = 1;
        scaler_z = 1;
        scaler_yaw = 1;
        lvl_scaler_rll = 1;
        lvl_scaler_pit = 1;
        AP_Param::setup_object_defaults(this, var_info);
    };

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

    // Vel & pos PIDs
    //p, i, d, ff, imax, filt_T_hz, filt_E_hz, filt_D_hz
    AC_PID pid_vel_x{3, 0.2, 0, 0, 0.5, 0, 0, 0};
    AC_PID pid_vel_y{3, 0.2, 0, 0, 0.5, 0, 0, 0};
    AC_PID pid_vel_z{7, 1.5, 0, 0, 0.5, 0, 0, 0};
    AC_PID pid_vel_yaw{3, 0.4, 0, 0, 0.5, 0, 0, 0};

    AC_PID pid_pos_x{1, 0.05, 0, 0, 0.5, 0, 0, 0};
    AC_PID pid_pos_y{1, 0.05, 0, 0, 0.5, 0, 0, 0};
    AC_PID pid_pos_z{0.7, 0,   0, 0, 0.5, 0, 0, 0};
    AC_PID pid_pos_yaw{1.2, 0.5, 0, 0, 0.5, 0, 0, 0};

    AC_PID pid_lvl_pitch{1, 0.2, 0, 0, 0.5, 0, 0, 0};
    AC_PID pid_lvl_roll{1, 0.2, 0, 0, 0.5, 0, 0, 0};
    AP_Float lvl_max;
    AP_Float lvl_relax_tc;

    AP_Float    max_vel_x;
    AP_Float    max_vel_y;
    AP_Float    max_vel_z;
    AP_Float    max_vel_yaw;
    AP_Float    max_pos_x;
    AP_Float    max_pos_y;
    AP_Float    max_pos_z;
    AP_Float    max_pos_yaw;

    AP_Int16    dis_mask;
    AP_Float    pid_dz;
    AP_Float    scaler_spd;
    AP_Float    pos_lag;
    AP_Int16    options;

    enum option {
        LVL_EN_YAW_RATE= (1 << 0),
        LVL_EN_YAW_POS=  (1 << 1),
        LVL_EN_Z_RATE=   (1 << 2),
    };

    //Run Loiter controller with target position and yaw in global frame. Expects to be called at loop rate.
    void run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled);
    //Run Loiter controller with target velocity and yaw velocity in body frame. Expects to be called at loop rate.
    void run_vel(Vector3f& target_vel, float& target_vel_yaw, Vector4b axes_disabled, bool log);

    void run_level_roll(float& out_right_com);
    void run_level_pitch(float& out_front_com);
    void run_yaw_stab(float& out_yaw_com, float& target_yaw);
    void run_down_stab(float& out_down_com);

    bool target_within(float distance){
        return (targ_dist <= distance);
    }
};
