#include "Blimp.h"

#include <AC_AttitudeControl/AC_PosControl.h>

#define MA 0.99
#define MO (1-MA)

void Loiter::run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    float scaler_xz_n;
    float xz_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->down_out);
    if (xz_out > 1) {
        scaler_xz_n = 1 / xz_out;
    } else {
        scaler_xz_n = 1;
    }
    scaler_xz = scaler_xz*MA + scaler_xz_n*MO;

    float scaler_yyaw_n;
    float yyaw_out = fabsf(blimp.motors->right_out) + fabsf(blimp.motors->yaw_out);
    if (yyaw_out > 1) {
        scaler_yyaw_n = 1 / yyaw_out;
    } else {
        scaler_yyaw_n = 1;
    }
    scaler_yyaw = scaler_yyaw*MA + scaler_yyaw_n*MO;

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("BSC", "TimeUS,xz,yyaw,xzn,yyawn",
                                "Qffff",
                                AP_HAL::micros64(),
                                scaler_xz, scaler_yyaw, scaler_xz_n, scaler_yyaw_n);
#endif

    float yaw_ef = blimp.ahrs.get_yaw_rad();
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
    if (!axes_disabled.x && !axes_disabled.y) target_vel_ef = {blimp.pid_pos_xy.update_all(target_pos, blimp.pos_ned, dt, {(float)limit.x, (float)limit.y, (float)limit.z}), 0};
    if (!axes_disabled.z) {
        target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
    }

    float target_vel_yaw = 0;
    if (!axes_disabled.yaw) {
        target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
        blimp.pid_pos_yaw.set_target_rate(target_yaw);
        blimp.pid_pos_yaw.set_actual_rate(yaw_ef);
    }

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector2f target_vel_ef_c_scaled_xy = {target_vel_ef_c.x * scaler_xz, target_vel_ef_c.y * scaler_yyaw};
    Vector2f vel_ned_filtd_scaled_xy = {blimp.vel_ned_filtd.x * scaler_xz, blimp.vel_ned_filtd.y * scaler_yyaw};

    Vector2f actuator;
    if (!axes_disabled.x && !axes_disabled.y) actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c_scaled_xy, vel_ned_filtd_scaled_xy, dt, {(float)limit.x, (float)limit.y});
    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_xz, blimp.vel_ned_filtd.z * scaler_xz, dt, limit.z);
    }
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yyaw, blimp.vel_yaw_filtd * scaler_yyaw, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_pos_xy.set_integrator(Vector2f(0,0));
        blimp.pid_pos_z.set_integrator(0);
        blimp.pid_pos_yaw.set_integrator(0);
        blimp.pid_vel_xy.set_integrator(Vector2f(0,0));
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw_rad();
    }

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
    AC_PosControl::Write_PSCN(0.0, target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, -target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}

void Loiter::run_vel(Vector3f& target_vel_ef, float& target_vel_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

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

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector2f target_vel_ef_c_scaled_xy = {target_vel_ef_c.x * scaler_xz, target_vel_ef_c.y * scaler_yyaw};
    Vector2f vel_ned_filtd_scaled_xy = {blimp.vel_ned_filtd.x * scaler_xz, blimp.vel_ned_filtd.y * scaler_yyaw};

    Vector2f actuator;
    if (!axes_disabled.x && !axes_disabled.y) actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c_scaled_xy, vel_ned_filtd_scaled_xy, dt, {(float)limit.x, (float)limit.y});
    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_xz, blimp.vel_ned_filtd.z * scaler_xz, dt, limit.z);
    }
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yyaw, blimp.vel_yaw_filtd * scaler_yyaw, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_vel_xy.set_integrator(Vector2f(0,0));
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
    }

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
    AC_PosControl::Write_PSCN(0.0, 0.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, 0.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, 0.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}
