/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
#define K1 2.0
#define K2 5.0
#define K3 1.0
#define K4 2.2  */

#define K1 .32
#define K2 2.5
#define K3 .5
#define K4 1.1


static int16_t
get_stabilize_pitch(int32_t target_angle)
{
    int16_t torque;
    int32_t angle_error = wrap_180(target_angle - (ahrs.pitch_sensor + balance_offset));  //balance_offset

    // dynamically adjust the CG when we are supposed to be at vertical
    if(target_angle == 0){
        balance_offset = g.pid_balance.get_i(angle_error, G_Dt);
    }
    int32_t rate_error  = 0 - (omega.y * DEGX100);

    int16_t bal_P = g.pid_balance.kP() * (float)angle_error;
    //int16_t bal_P = g.pid_balance.get_p(angle_error);
    int16_t bal_D = g.pid_balance.kD() * (float)rate_error;
    //int16_t bal_D = g.pid_balance.get_d(angle_error, G_Dt);

    torque = bal_P + bal_D;

    //cliSerial->printf_P(PSTR("a:%d, P:%d, D:%d\n"), (int16_t)ahrs.pitch_sensor, bal_P, bal_D);
   // cliSerial->printf_P(PSTR("a:%d\te:%d\n"), (int16_t)ahrs.pitch_sensor, angle_error);

    return torque;
}

static int16_t
get_velocity_pitch()
{
    //cliSerial->printf_P(PSTR("ws:%d, se:%d\n"), wheel.speed, speed_error);
    return g.p_vel * (float)wheel.speed; /// 1.8
}

static int16_t
get_stabilize_yaw(int32_t target_angle)
{
    int32_t angle_error;

    // angle error
    angle_error         = wrap_180(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error         = constrain(angle_error, -1000, 1000);
    //int16_t output      = (float)g.pid_yaw.get_pid(angle_error, G_Dt) / wheel_ratio;
    int16_t output      = (float)g.pid_yaw.get_p(angle_error) / wheel_ratio;
    return output;
}




static int16_t
get_acro_yaw(int32_t target_rate)
{
    //return g.pi_stabilize_yaw.get_p(target_rate);
    return 0;
}


/*
  reset all I integrators
 */
static void reset_I_all(void)
{
    balance_offset = 0;
    g.pid_wheel_left_mixer.reset_I();
    g.pid_wheel_right_mixer.reset_I();
    g.pid_balance.reset_I();
    g.pid_yaw.reset_I();

    reset_nav_I();
}

static void reset_nav_I(void)
{
    g.pid_nav.reset_I();
}

