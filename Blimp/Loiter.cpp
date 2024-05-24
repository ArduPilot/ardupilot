#include "Blimp.h"

#include <AC_AttitudeControl/AC_PosControl.h>

const AP_Param::GroupInfo Loiter::var_info[] = {

    //Higher number means slower change. Zero means immediate change (no filter).
    AP_GROUPINFO("SCALER_SPD", 0, Loiter, scaler_spd, 0.99),

    //Number of seconds' worth of travel that the actual position can be behind the target position.
    AP_GROUPINFO("LAG", 1, Loiter, pos_lag, 1),

    AP_GROUPEND
};

#define MA scaler_spd
#define MO (1-MA)

void Loiter::run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    float yaw_ef = blimp.ahrs.get_yaw();
    Vector3f err_xyz = target_pos - blimp.pos_ned;
    float err_yaw = wrap_PI(target_yaw - yaw_ef);

    Vector4b zero;
    if ((fabsf(err_xyz.x) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.x = true;
    }
    if ((fabsf(err_xyz.y) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.y = true;
    }
    if ((fabsf(err_xyz.z) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if ((fabsf(err_yaw)   < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }

    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    Vector3f target_vel_ef;
    if (!axes_disabled.x)
        target_vel_ef.x = blimp.pid_pos_x.update_all(target_pos.x, blimp.pos_ned.x, dt, limit.x);
    if (!axes_disabled.x)
        target_vel_ef.y = blimp.pid_pos_y.update_all(target_pos.y, blimp.pos_ned.y, dt, limit.y);
    if (!axes_disabled.z) {
        target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
    }

    float target_vel_yaw = 0;
    if (!axes_disabled.yaw) {
        target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
        blimp.pid_pos_yaw.set_target_rate(target_yaw);
        blimp.pid_pos_yaw.set_actual_rate(yaw_ef);
    }

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_x, blimp.g.max_vel_x),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_y, blimp.g.max_vel_y),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    if (!blimp.motors->armed()) {
        blimp.pid_pos_x.set_integrator(0);
        blimp.pid_pos_y.set_integrator(0);
        blimp.pid_pos_z.set_integrator(0);
        blimp.pid_pos_yaw.set_integrator(0);
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw();
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(-target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif

    run_vel(target_vel_ef_c, target_vel_yaw_c, axes_disabled, false);
}

//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
void Loiter::run_vel(Vector3f& target_vel_ef, float& target_vel_yaw, Vector4b axes_disabled, bool log)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    blimp.rotate_NE_to_BF(target_vel_ef.xy());
    //Just for the sake of clarity...
    Vector3f target_vel_bf = target_vel_ef;

    //New value for scaler
    float scaler_x_n = 1;
    float scaler_y_n = 1;
    float scaler_z_n = 1;
    float scaler_yaw_n = 1;

    if ((Fins::motor_frame_class)blimp.g2.frame_class.get() == Fins::MOTOR_FRAME_FISHBLIMP) {
        float xz_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->down_out);
        if (xz_out > 1) {
            scaler_x_n = 1 / xz_out;
            scaler_z_n = 1 / xz_out;
        }
        float yyaw_out = fabsf(blimp.motors->right_out) + fabsf(blimp.motors->yaw_out);
        if (yyaw_out > 1) {
            scaler_y_n = 1 / yyaw_out;
            scaler_yaw_n = 1 / yyaw_out;
        }
    }
    else if ((Fins::motor_frame_class)blimp.g2.frame_class.get() == Fins::MOTOR_FRAME_FOUR_MOTOR) {
        float xyaw_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->yaw_out);
        if (xyaw_out > 1) {
            scaler_x_n = 1 / xyaw_out;
            scaler_yaw_n = 1 / xyaw_out;
        }
    }

    scaler_x = scaler_x*MA + scaler_x_n*MO;
    scaler_y = scaler_y*MA + scaler_y_n*MO;
    scaler_z = scaler_z*MA + scaler_z_n*MO;
    scaler_yaw = scaler_yaw*MA + scaler_yaw_n*MO;

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("BSC", "TimeUS,x,y,z,yaw,xn,yn,zn,yawn",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                scaler_x, scaler_y, scaler_z, scaler_yaw, scaler_x_n, scaler_y_n, scaler_z_n, scaler_yaw_n);
#endif
    if (AP_HAL::millis() % blimp.g.stream_rate < 30) {
        gcs().send_named_float("BSCXN", scaler_x_n);
        gcs().send_named_float("BSCYN", scaler_y_n);
        gcs().send_named_float("BSCZN", scaler_z_n);
        gcs().send_named_float("BSCYAWN", scaler_yaw_n);
        gcs().send_named_float("BSCX", scaler_x);
        gcs().send_named_float("BSCY", scaler_y);
        gcs().send_named_float("BSCZ", scaler_z);
        gcs().send_named_float("BSCYAW", scaler_yaw);
    }

    Vector4b zero;
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.x = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.y = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }
    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    Vector3f target_vel_bf_c{constrain_float(target_vel_bf.x, -blimp.g.max_vel_x, blimp.g.max_vel_x),
                             constrain_float(target_vel_bf.y, -blimp.g.max_vel_y, blimp.g.max_vel_y),
                             constrain_float(target_vel_bf.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector3f vel_bf_filtd = blimp.vel_ned_filtd;
    blimp.rotate_NE_to_BF(vel_bf_filtd.xy());

    Vector2f actuator;
    if (!axes_disabled.x) {
        actuator.x = blimp.pid_vel_x.update_all(target_vel_bf_c.x * scaler_x, vel_bf_filtd.x, dt, limit.x);
    }

    if (!axes_disabled.y) {
        actuator.y = blimp.pid_vel_y.update_all(target_vel_bf_c.y * scaler_y, vel_bf_filtd.y, dt, limit.y);
    }

    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_bf_c.z * scaler_z, vel_bf_filtd.z, dt, limit.z);
    }

    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yaw, blimp.vel_yaw_filtd, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_vel_x.set_integrator(0);
        blimp.pid_vel_y.set_integrator(0);
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
    }

    // blimp.rotate_NE_to_BF(actuator); //Don't need this anymore because we're already in BF

    if (zero.x) {
        blimp.motors->front_out = 0;
    } else if (axes_disabled.x);
    else {
        blimp.motors->front_out = actuator.x;
    }
    if (zero.y) {
        blimp.motors->right_out = 0;
    } else if (axes_disabled.y);
    else {
        blimp.motors->right_out = actuator.y;
    }
    if (zero.z) {
        blimp.motors->down_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->down_out = act_down;
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

#if HAL_LOGGING_ENABLED
    if(log) {
        AC_PosControl::Write_PSCN(0.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_bf_c.x * 100.0, vel_bf_filtd.x * 100.0, 0.0, 0.0, 0.0);
        AC_PosControl::Write_PSCE(0.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_bf_c.y * 100.0, vel_bf_filtd.y * 100.0, 0.0, 0.0, 0.0);
        AC_PosControl::Write_PSCD(0.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_bf_c.z * 100.0, -vel_bf_filtd.z * 100.0, 0.0, 0.0, 0.0);
    }
#endif
}
