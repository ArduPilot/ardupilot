/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for autotune of multirotors. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell
 */

#define AUTOTUNE_FFI_RATIO_FOR_TESTING     0.5f     // I is set 2x smaller than VFF during testing
#define AUTOTUNE_FFI_RATIO_FINAL           0.5f     // I is set 0.5x VFF after testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_STEP                  0.05f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.05f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_RD_MAX                  0.200f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  5.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in

#include "AC_AutoTune_Multi.h"

const AP_Param::GroupInfo AC_AutoTune_Multi::var_info[] = {

    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AC_AutoTune_Multi, axis_bitmask,  7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

    // @Param: AGGR
    // @DisplayName: Autotune aggressiveness
    // @Description: Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
    // @Range: 0.05 0.10
    // @User: Standard
    AP_GROUPINFO("AGGR", 2, AC_AutoTune_Multi, aggressiveness, 0.1f),

    // @Param: MIN_D
    // @DisplayName: AutoTune minimum D
    // @Description: Defines the minimum D gain
    // @Range: 0.001 0.006
    // @User: Standard
    AP_GROUPINFO("MIN_D", 3, AC_AutoTune_Multi, min_d,  0.001f),

    AP_GROUPEND
};

// constructor
AC_AutoTune_Multi::AC_AutoTune_Multi()
{
    tune_seq[0] = TUNE_COMPLETE;
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_AutoTune_Multi::do_gcs_announcements()
{
    const uint32_t now = AP_HAL::millis();
    if (now - announce_time < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    float tune_rp = 0.0f;
    float tune_rd = 0.0f;
    float tune_sp = 0.0f;
    float tune_accel = 0.0f;
    char axis_char = '?';
    switch (axis) {
    case ROLL:
        tune_rp = tune_roll_rp;
        tune_rd = tune_roll_rd;
        tune_sp = tune_roll_sp;
        tune_accel = tune_roll_accel;
        axis_char = 'R';
        break;
    case PITCH:
        tune_rp = tune_pitch_rp;
        tune_rd = tune_pitch_rd;
        tune_sp = tune_pitch_sp;
        tune_accel = tune_pitch_accel;
        axis_char = 'P';
        break;
    case YAW:
        tune_rp = tune_yaw_rp;
        tune_rd = tune_yaw_rLPF;
        tune_sp = tune_yaw_sp;
        tune_accel = tune_yaw_accel;
        axis_char = 'Y';
        break;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: (%c) %s", axis_char, type_string());
    send_step_string();
    if (!is_zero(lean_angle)) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: lean=%f target=%f", (double)lean_angle, (double)target_angle);
    }
    if (!is_zero(rotation_rate)) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: rotation=%f target=%f", (double)(rotation_rate*0.01f), (double)(target_rate*0.01f));
    }
    switch (tune_type) {
    case RD_UP:
    case RD_DOWN:
    case RP_UP:
    case RP_DOWN:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: p=%f d=%f", (double)tune_rp, (double)tune_rd);
        break;
    case RFF_UP:
    case RFF_DOWN:
        break;
    case SP_DOWN:
    case SP_UP:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: p=%f accel=%f", (double)tune_sp, (double)tune_accel);
        break;
    case MAX_GAINS:
    case TUNE_COMPLETE:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: success %u/%u", counter, AUTOTUNE_SUCCESS_COUNT);

    announce_time = now;
}

void AC_AutoTune_Multi::test_init()
{
    twitch_test_init();
}

void AC_AutoTune_Multi::test_run(AxisType test_axis, const float dir_sign)
{
    twitch_test_run(test_axis, dir_sign);
}

// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune_Multi::load_test_gains()
{
    AC_AutoTune::load_test_gains();

    switch (axis) {
    case ROLL:
        attitude_control->get_rate_roll_pid().kI(tune_roll_rp*0.01f);
        attitude_control->get_rate_roll_pid().ff(0.0f);
        attitude_control->get_rate_roll_pid().filt_T_hz(0.0f);
        attitude_control->get_rate_roll_pid().slew_limit(0.0f);
        break;
    case PITCH:
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*0.01f);
        attitude_control->get_rate_pitch_pid().ff(0.0f);
        attitude_control->get_rate_pitch_pid().filt_T_hz(0.0f);
        attitude_control->get_rate_pitch_pid().slew_limit(0.0f);
        break;
    case YAW:
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
        attitude_control->get_rate_yaw_pid().kD(0.0f);
        attitude_control->get_rate_yaw_pid().ff(0.0f);
        attitude_control->get_rate_yaw_pid().filt_T_hz(0.0f);
        attitude_control->get_rate_yaw_pid().slew_limit(0.0f);
        break;
    }
}

// save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void AC_AutoTune_Multi::save_tuning_gains()
{

    AC_AutoTune::save_tuning_gains();

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && !is_zero(tune_roll_rp)) {
        // rate roll gains
        attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(orig_roll_smax);
        attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
        orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
        orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        // rate yaw gains
        attitude_control->get_rate_yaw_pid().kD(0.0f);
        attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(orig_yaw_smax);
        attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
        attitude_control->get_rate_yaw_pid().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
        orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
        orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
        orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    }

    // update GCS and log save gains event
    update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
    AP::logger().Write_Event(LogEvent::AUTOTUNE_SAVEDGAINS);

    reset();
}

// update gains for the rate p up tune type
void AC_AutoTune_Multi::updating_rate_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_rate_p_up_d_down(tune_roll_rd, min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case PITCH:
        updating_rate_p_up_d_down(tune_pitch_rd, min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case YAW:
        updating_rate_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the rate d up tune type
void AC_AutoTune_Multi::updating_rate_d_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_rate_d_up(tune_roll_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case PITCH:
        updating_rate_d_up(tune_pitch_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case YAW:
        updating_rate_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the rate d down tune type
void AC_AutoTune_Multi::updating_rate_d_down_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_rate_d_down(tune_roll_rd, min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case PITCH:
        updating_rate_d_down(tune_pitch_rd, min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case YAW:
        updating_rate_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the angle p up tune type
void AC_AutoTune_Multi::updating_angle_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_angle_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case PITCH:
        updating_angle_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case YAW:
        updating_angle_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the angle p down tune type
void AC_AutoTune_Multi::updating_angle_p_down_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_angle_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case PITCH:
        updating_angle_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case YAW:
        updating_angle_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    }
}

// updating_rate_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void AC_AutoTune_Multi::updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
{
    if (meas_rate_max > rate_target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D gain so stop tuning
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        // we have a good measurement of bounce back
        if (meas_rate_max-meas_rate_min > meas_rate_max*aggressiveness) {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is bigger than our threshold so increment the success counter
            counter++;
        } else {
            if (ignore_next == false) {
                // bounce back is smaller than our threshold so decrement the success counter
                if (counter > 0) {
                    counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio*2.0f;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    counter = AUTOTUNE_SUCCESS_COUNT;
                    AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                ignore_next = false;
            }
        }
    }
}

// updating_rate_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void AC_AutoTune_Multi::updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
{
    if (meas_rate_max > rate_target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D gain
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D so stop tuning
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        // we have a good measurement of bounce back
        if (meas_rate_max-meas_rate_min < meas_rate_max*aggressiveness) {
            if (ignore_next == false) {
                // bounce back is less than our threshold so increment the success counter
                counter++;
            } else {
                ignore_next = false;
            }
        } else {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is larger than our threshold so decrement the success counter
            if (counter > 0) {
                counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d*tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// updating_rate_p_up_d_down - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void AC_AutoTune_Multi::updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
{
    if (meas_rate_max > rate_target*(1+0.5f*aggressiveness)) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        counter++;
    } else if ((meas_rate_max < rate_target) && (meas_rate_max > rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (meas_rate_max-meas_rate_min > meas_rate_max*aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (counter > 0) {
            counter--;
        }
        // decrease D gain (which should decrease bounce back)
        tune_d -= tune_d*tune_d_step_ratio;
        // do not decrease the D term past the minimum
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // do not decrease the P term past the minimum
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
        // cancel change in direction
        positive_direction = !positive_direction;
    } else {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (counter > 0) {
                counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                counter = AUTOTUNE_SUCCESS_COUNT;
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

// updating_angle_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void AC_AutoTune_Multi::updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
{
    if (meas_angle_max < angle_target*(1+0.5f*aggressiveness)) {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so increment the success counter
            counter++;
        } else {
            ignore_next = false;
        }
    } else {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was higher than target so decrement the success counter
        if (counter > 0) {
            counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            counter = AUTOTUNE_SUCCESS_COUNT;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// updating_angle_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void AC_AutoTune_Multi::updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
{
    if ((meas_angle_max > angle_target*(1+0.5f*aggressiveness)) ||
        ((meas_angle_max > angle_target) && (meas_rate_min < -meas_rate_max*aggressiveness))) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        counter++;
    } else {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (counter > 0) {
                counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                counter = AUTOTUNE_SUCCESS_COUNT;
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

void AC_AutoTune_Multi::Log_AutoTune()
{
    if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
        switch (axis) {
        case ROLL:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
            break;
        case PITCH:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
            break;
        case YAW:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max);
            break;
        }
    } else {
        switch (axis) {
        case ROLL:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
            break;
        case PITCH:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
            break;
        case YAW:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max);
            break;
        }
    }

}

void AC_AutoTune_Multi::Log_AutoTuneDetails()
{
    Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
}

// @LoggerMessage: ATUN
// @Description: Copter/QuadPlane AutoTune
// @Vehicles: Copter, Plane
// @Field: TimeUS: Time since system startup
// @Field: Axis: which axis is currently being tuned
// @Field: TuneStep: step in autotune process
// @Field: Targ: target angle or rate, depending on tuning step
// @Field: Min: measured minimum target angle or rate
// @Field: Max: measured maximum target angle or rate
// @Field: RP: new rate gain P term
// @Field: RD: new rate gain D term
// @Field: SP: new angle P term
// @Field: ddt: maximum measured twitching acceleration

// Write an Autotune data packet
void AC_AutoTune_Multi::Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
{
    AP::logger().Write(
        "ATUN",
        "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt",
        "s--ddd---o",
        "F--000---0",
        "QBBfffffff",
        AP_HAL::micros64(),
        axis,
        tune_step,
        meas_target*0.01f,
        meas_min*0.01f,
        meas_max*0.01f,
        new_gain_rp,
        new_gain_rd,
        new_gain_sp,
        new_ddt);
}

// Write an Autotune data packet
void AC_AutoTune_Multi::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    // @LoggerMessage: ATDE
    // @Description: AutoTune data packet
    // @Field: TimeUS: Time since system startup
    // @Field: Angle: current angle
    // @Field: Rate: current angular rate
    AP::logger().WriteStreaming(
        "ATDE",
        "TimeUS,Angle,Rate",
        "sdk",
        "F00",
        "Qff",
        AP_HAL::micros64(),
        angle_cd*0.01f,
        rate_cds*0.01f);
}

// get intra test rate I gain for the specified axis
float AC_AutoTune_Multi::get_intra_test_ri(AxisType test_axis)
{
    float ret = 0.0f;
    switch (test_axis) {
    case ROLL:
        ret = orig_roll_rp * AUTOTUNE_PI_RATIO_FOR_TESTING;
        break;
    case PITCH:
        ret = orig_pitch_rp * AUTOTUNE_PI_RATIO_FOR_TESTING;
        break;
    case YAW:
        ret = orig_yaw_rp * AUTOTUNE_PI_RATIO_FOR_TESTING;
        break;
    }
    return ret;
}

// get tuned rate I gain for the specified axis
float AC_AutoTune_Multi::get_tuned_ri(AxisType test_axis)
{
    float ret = 0.0f;
    switch (test_axis) {
    case ROLL:
        ret = tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL;
        break;
    case PITCH:
        ret = tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL;
        break;
    case YAW:
        ret = tune_yaw_rp*AUTOTUNE_PI_RATIO_FINAL;
        break;
    }
    return ret;
}

// get minimum rate P (for any axis)
float AC_AutoTune_Multi::get_rp_min() const
{
    return AUTOTUNE_RP_MIN;
}

// get minimum angle P (for any axis)
float AC_AutoTune_Multi::get_sp_min() const
{
    return AUTOTUNE_SP_MIN;
}

// get minimum rate Yaw filter value
float AC_AutoTune_Multi::get_yaw_rate_filt_min() const
{
    return AUTOTUNE_RLPF_MIN;
}
