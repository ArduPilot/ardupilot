/*
  C++ implementation of quicktune based on original lua script
 */

// quicktune is not performance sensitive, save flash
#pragma GCC optimize("Os")

#include "AP_Quicktune.h"

#if AP_QUICKTUNE_ENABLED

#define UPDATE_RATE_HZ 40
#define STAGE_DELAY 4000
#define PILOT_INPUT_DELAY 4000
#define YAW_FLTE_MAX 2.0
#define FLTD_MUL 0.5
#define FLTT_MUL 0.5
#define DEFAULT_SMAX 50.0
#define OPTIONS_TWO_POSITION (1<<0)

#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Arming/AP_Arming.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Quicktune::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Quicktune enable
    // @Description: Enable quicktune system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Quicktune, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: AXES
    // @DisplayName: Quicktune axes
    // @Description: Axes to tune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 2, AP_Quicktune, axes_enabled, 7),

    // @Param: DOUBLE_TIME
    // @DisplayName: Quicktune doubling time
    // @Description: Time to double a tuning parameter. Raise this for a slower tune.
    // @Range: 5 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("DOUBLE_TIME", 3, AP_Quicktune, double_time, 10),

    // @Param: GAIN_MARGIN
    // @DisplayName: Quicktune gain margin
    // @Description: Reduction in gain after oscillation detected. Raise this number to get a more conservative tune
    // @Range: 20 80
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("GAIN_MARGIN", 4, AP_Quicktune, gain_margin, 60),

    // @Param: OSC_SMAX
    // @DisplayName: Quicktune oscillation rate threshold
    // @Description: Threshold for oscillation detection. A lower value will lead to a more conservative tune.
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("OSC_SMAX", 5, AP_Quicktune, osc_smax, 4),

    // @Param: YAW_P_MAX
    // @DisplayName: Quicktune Yaw P max
    // @Description: Maximum value for yaw P gain
    // @Range: 0.1 3
    // @User: Standard
    AP_GROUPINFO("YAW_P_MAX", 6, AP_Quicktune, yaw_p_max, 0.5),

    // @Param: YAW_D_MAX
    // @DisplayName: Quicktune Yaw D max
    // @Description: Maximum value for yaw D gain
    // @Range: 0.001 1
    // @User: Standard
    AP_GROUPINFO("YAW_D_MAX", 7, AP_Quicktune, yaw_d_max, 0.01),

    // @Param: RP_PI_RATIO
    // @DisplayName: Quicktune roll/pitch PI ratio
    // @Description: Ratio between P and I gains for roll and pitch. Raise this to get a lower I gain
    // @Range: 1.0 2.0
    // @User: Standard
    AP_GROUPINFO("RP_PI_RATIO", 8, AP_Quicktune, rp_pi_ratio, 1.0),

    // @Param: Y_PI_RATIO
    // @DisplayName: Quicktune Yaw PI ratio
    // @Description: Ratio between P and I gains for yaw. Raise this to get a lower I gain
    // @Range: 1.0 20
    // @User: Standard
    AP_GROUPINFO("Y_PI_RATIO", 9, AP_Quicktune, y_pi_ratio, 10),

    // @Param: AUTO_FILTER
    // @DisplayName: Quicktune auto filter enable
    // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTO_FILTER", 10, AP_Quicktune, auto_filter, 1),

    // @Param: AUTO_SAVE
    // @DisplayName: Quicktune auto save
    // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune. Zero (the default value) disables automatic saving, and you will need to have a 3 position switch to save or use GCS auxilliary functions.
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("AUTO_SAVE", 11, AP_Quicktune, auto_save, 0),

    // @Param: REDUCE_MAX
    // @DisplayName: Quicktune maximum gain reduction
    // @Description: This controls how much quicktune is allowed to lower gains from the original gains. If the vehicle already has a reasonable tune and is not oscillating then you can set this to zero to prevent gain reductions. The default of 20% is reasonable for most vehicles. Using a maximum gain reduction lowers the chance of an angle P oscillation happening if quicktune gets a false positive oscillation at a low gain, which can result in very low rate gains and a dangerous angle P oscillation.
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("REDUCE_MAX", 12, AP_Quicktune, reduce_max, 20),

    // @Param: OPTIONS
    // @DisplayName: Quicktune options
    // @Description: Additional options. When the Two Position Switch option is enabled then a high switch position will start the tune, low will disable the tune. you should also set a QUIK_AUTO_SAVE time so that you will be able to save the tune.
    // @Bitmask: 0:UseTwoPositionSwitch
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 13, AP_Quicktune, options, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: maximum angle error for tune abort
    // @Description: If while tuning the angle error goes over this limit then the tune will aborts to prevent a bad oscillation in the case of the tuning algorithm failing. If you get an error "Quicktune: attitude error ABORTING" and you think it is a false positive then you can either raise this parameter or you can try increasing the QWIK_DOUBLE_TIME to do the tune more slowly.
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 14, AP_Quicktune, angle_max, 10),
    
    AP_GROUPEND
};

// Call at loop rate
void AP_Quicktune::update(bool mode_supports_quicktune)
{
    if (enable < 1) {
        if (need_restore) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "QuickTune disabled");
            abort_tune();
        }
        return;
    }
    const uint32_t now = AP_HAL::millis();

    if (!mode_supports_quicktune) {
        /*
          user has switched to a non-quicktune mode. If we have
          pending parameter changes then revert
         */
        if (need_restore) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "QuickTune aborted");
        }
        abort_tune();
        return;
    }

    if (need_restore) {
        const float att_error = AC_AttitudeControl::get_singleton()->get_att_error_angle_deg();
        if (angle_max > 0 && att_error > angle_max) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Quicktune: attitude error %.1fdeg - ABORTING", att_error);
            abort_tune();
            return;
        }
    }

    if (have_pilot_input()) {
        last_pilot_input_ms = now;
    }

    SwitchPos sw_pos_tune = SwitchPos::MID;
    SwitchPos sw_pos_save = SwitchPos::HIGH;
    if ((options & OPTIONS_TWO_POSITION) != 0) {
        sw_pos_tune = SwitchPos::HIGH;
        sw_pos_save = SwitchPos::NONE;
    }

    const auto &vehicle = *AP::vehicle();

    if (sw_pos == sw_pos_tune &&
        (!hal.util->get_soft_armed() || !vehicle.get_likely_flying())) {
        if (now - last_warning_ms > 5000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Quicktune: Must be flying to tune");
            last_warning_ms = now;
        }
        return;
    }
    if (sw_pos == SwitchPos::LOW || !hal.util->get_soft_armed() || !vehicle.get_likely_flying()) {
        // Abort, revert parameters
        if (need_restore) {
            need_restore = false;
            restore_all_params();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Quicktune: Reverted");
            tune_done_time_ms = 0;
        }
        reset_axes_done();
        return;
    }
    if (sw_pos == sw_pos_save) {
        // Save all params
        if (need_restore) {
            need_restore = false;
            save_all_params();
        }
    }
    if (sw_pos != sw_pos_tune) {
        return;
    }

    if (now - last_stage_change_ms < STAGE_DELAY) {
        // Update slew gain
        if (slew_parm != Param::END) {
            const float P = get_param_value(slew_parm);
            const AxisName axis = get_axis(slew_parm);
            adjust_gain(slew_parm, P+slew_delta);
            slew_steps = slew_steps - 1;
            Write_QWIK(get_slew_rate(axis), P, slew_parm);
            if (slew_steps == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f", get_param_name(slew_parm), P);
                slew_parm = Param::END;
                if (get_current_axis() == AxisName::DONE) {
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Quicktune: DONE");
                    tune_done_time_ms = now;
                }
            }
        }
        return;
    }

    const AxisName axis = get_current_axis();

    if (axis == AxisName::DONE) {
        // Nothing left to do, check autosave time
        if (tune_done_time_ms != 0 && auto_save > 0) {
            if (now - tune_done_time_ms > (auto_save*1000)) {
                need_restore = false;
                save_all_params();
                tune_done_time_ms = 0;
            }
        }
        return;
    }

    if (!need_restore) {
        need_restore = true;
        // We are just starting tuning, get current values
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Quicktune: Starting tune");
        // Get all params
        for (uint8_t p = 0; p < uint8_t(Param::END); p++) {
            param_saved[p] = get_param_value(Param(p));
        }
        // Set up SMAX
        const Param is[3] { Param::RLL_SMAX, Param::PIT_SMAX, Param::YAW_SMAX };
        for (const auto p : is) {
            const float smax = get_param_value(p);
            if (smax <= 0) {
                adjust_gain(p, DEFAULT_SMAX);
            }
        }
    }

    if (now - last_pilot_input_ms < PILOT_INPUT_DELAY) {
        return;
    }

    if (!BIT_IS_SET(filters_done, uint8_t(axis))) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting %s tune", get_axis_name(axis));
        setup_filters(axis);
    }

    const Param pname = get_pname(axis, current_stage);
    const float pval = get_param_value(pname);
    const float limit = gain_limit(pname);
    const bool limited = (limit > 0.0 && pval >= limit);
    const float srate = get_slew_rate(axis);
    const bool oscillating = srate > osc_smax;
    
    // Check if reached limit
    if (limited || oscillating) {
        float reduction = (100.0-gain_margin)*0.01;
        if (!oscillating) {
            reduction = 1.0;
        }
        float new_gain = pval * reduction;
        if (limit > 0.0 && new_gain > limit) {
            new_gain = limit;
        }
        float old_gain = param_saved[uint8_t(pname)];
        if (new_gain < old_gain && (pname == Param::PIT_D || pname == Param::RLL_D)) {
            // We are lowering a D gain from the original gain. Also
            // lower the P gain by the same amount so that we don't
            // trigger P oscillation. We don't drop P by more than a
            // factor of 2
            const float ratio = fmaxf(new_gain / old_gain, 0.5);
            const uint8_t P_TO_D_OFS = uint8_t(Param::RLL_D) - uint8_t(Param::RLL_P);
            const Param P_name = Param(uint8_t(pname)-P_TO_D_OFS); //from D to P
            const float old_pval = get_param_value(P_name);;
            const float new_pval = old_pval * ratio;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Adjusting %s %.3f -> %.3f", get_param_name(P_name), old_pval, new_pval);
            adjust_gain_limited(P_name, new_pval);
        }
        // Set up slew gain
        slew_parm = pname;
        const float slew_target = limit_gain(pname, new_gain);
        slew_steps = UPDATE_RATE_HZ/2;
        slew_delta = (slew_target - get_param_value(pname)) / slew_steps;

        Write_QWIK(srate, pval, pname);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Quicktune: %s done", get_param_name(pname));
        advance_stage(axis);
        last_stage_change_ms = now;
    } else {
        float new_gain = pval*get_gain_mul();
        // cope with the gain starting at zero (some users with have a zero D gain)
        new_gain = MAX(new_gain, 0.0001);
        adjust_gain_limited(pname, new_gain);
        Write_QWIK(srate, pval, pname);
        if (now - last_gain_report_ms > 3000U) {
            last_gain_report_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f sr:%.2f", get_param_name(pname), new_gain, srate);
        }
    }
}

/*
  abort the tune if it has started
 */
void AP_Quicktune::abort_tune()
{
    if (need_restore) {
        need_restore = false;
        restore_all_params();
    }
    tune_done_time_ms = 0;
    reset_axes_done();
    sw_pos = SwitchPos::LOW;
}

void AP_Quicktune::update_switch_pos(const  RC_Channel::AuxSwitchPos ch_flag) 
{
    sw_pos = SwitchPos(ch_flag);
}

void AP_Quicktune::reset_axes_done()
{
    axes_done = 0;
    filters_done = 0;
    current_stage = Stage::D;
}

void AP_Quicktune::setup_filters(AP_Quicktune::AxisName axis)
{
    if (auto_filter <= 0) {
        BIT_SET(filters_done, uint8_t(axis));
        return;
    }
    AP_InertialSensor *imu = AP_InertialSensor::get_singleton();
    if (imu == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    const float gyro_filter = imu->get_gyro_filter_hz();
    adjust_gain(get_pname(axis, Stage::FLTT), gyro_filter * FLTT_MUL);
    adjust_gain(get_pname(axis, Stage::FLTD), gyro_filter * FLTD_MUL);

    if (axis == AxisName::YAW) {
        const float FLTE = get_param_value(Param::YAW_FLTE);
        if (FLTE < 0.0 || FLTE > YAW_FLTE_MAX) {
            adjust_gain(Param::YAW_FLTE, YAW_FLTE_MAX);
        }
    }
    BIT_SET(filters_done, uint8_t(axis));
}

// Check for pilot input to pause tune
bool AP_Quicktune::have_pilot_input() const
{
    auto &RC = rc();
    const float roll = RC.get_roll_channel().norm_input_dz();
    const float pitch = RC.get_pitch_channel().norm_input_dz();
    const float yaw = RC.get_yaw_channel().norm_input_dz();

    if (fabsf(roll) > 0 || fabsf(pitch) > 0 || fabsf(yaw) > 0) {
        return true;
    }
    return false;
}

// Get the axis name we are working on, or DONE for all done 
AP_Quicktune::AxisName AP_Quicktune::get_current_axis() const
{
    for (int8_t i = 0; i < int8_t(AxisName::DONE); i++) {
        if (BIT_IS_SET(axes_enabled, i) && !BIT_IS_SET(axes_done, i)) {
            return AxisName(i);
        }
    }
    return AxisName::DONE;
}

// get the AC_PID object for an axis
AC_PID *AP_Quicktune::get_axis_pid(AP_Quicktune::AxisName axis) const
{
    auto &attitude_control = *AC_AttitudeControl::get_singleton();
    switch(axis) {
    case AxisName::RLL:
        return &attitude_control.get_rate_roll_pid();
    case AxisName::PIT:
        return &attitude_control.get_rate_pitch_pid();
    case AxisName::YAW:
        return &attitude_control.get_rate_yaw_pid();
    default:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    }
    return nullptr;
}

// get slew rate parameter for an axis
float AP_Quicktune::get_slew_rate(AP_Quicktune::AxisName axis) const
{
    auto *pid = get_axis_pid(axis);
    if (pid == nullptr) {
        return 0;
    }
    return pid->get_pid_info().slew_rate;
}

// Move to next stage of tune
void AP_Quicktune::advance_stage(AP_Quicktune::AxisName axis)
{
    if (current_stage == Stage::D) {
        current_stage = Stage::P;
    } else {
        BIT_SET(axes_done, uint8_t(axis));
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Quicktune: %s done", get_axis_name(axis));
        current_stage = Stage::D;
    }
}

void AP_Quicktune::adjust_gain(AP_Quicktune::Param param, float value)
{
    need_restore = true;
    BIT_SET(param_changed, uint8_t(param));
    set_param_value(param, value);

    if (get_stage(param) == Stage::P) {
        // Also change I gain
        const uint8_t P_TO_I_OFS = uint8_t(Param::RLL_I) - uint8_t(Param::RLL_P);
        const uint8_t P_TO_FF_OFS = uint8_t(Param::RLL_FF) - uint8_t(Param::RLL_P);
        const Param iname = Param(uint8_t(param)+P_TO_I_OFS);
        const Param ffname = Param(uint8_t(param)+P_TO_FF_OFS);
        float FF = get_param_value(ffname);
        if (FF > 0) {
            // If we have any FF on an axis then we don't couple I to P,
            // usually we want I = FF for a one second time constant for trim
            return;
        }
        BIT_SET(param_changed, uint8_t(iname));

        // Work out ratio of P to I that we want
        float pi_ratio = rp_pi_ratio;
        if (get_axis(param) == AxisName::YAW) {
            pi_ratio = y_pi_ratio;
        }
        if (pi_ratio >= 1) {
            set_param_value(iname, value/pi_ratio);
        }
    }

}

void AP_Quicktune::adjust_gain_limited(AP_Quicktune::Param param, float value)
{
    adjust_gain(param, limit_gain(param, value));
}

float AP_Quicktune::limit_gain(AP_Quicktune::Param param, float value)
{
    const float saved_value = param_saved[uint8_t(param)];
    if (reduce_max >= 0 && reduce_max < 100 && saved_value > 0) {
        // Check if we exceeded gain reduction
        const float reduction_pct = 100.0 * (saved_value - value) / saved_value;
        if (reduction_pct > reduce_max) {
            const float new_value = saved_value * (100 - reduce_max) * 0.01;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Limiting %s %.3f -> %.3f", get_param_name(param), value, new_value);
            value = new_value;
        }
    }
    return value;
}

const char* AP_Quicktune::get_param_name(AP_Quicktune::Param param) const
{
    switch (param)
    {
        case Param::RLL_P:
            return "Roll P";
        case Param::RLL_I:
            return "Roll I";
        case Param::RLL_D:
            return "Roll D";
        case Param::PIT_P:
            return "Pitch P";
        case Param::PIT_I:
            return "Pitch I";
        case Param::PIT_D:
            return "Pitch D";
        case Param::YAW_P:
            return "Yaw P";
        case Param::YAW_I:
            return "Yaw I";
        case Param::YAW_D:
            return "Yaw D";
        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return "UNK";
    }
}

float AP_Quicktune::get_gain_mul() const
{
    return expf(logf(2.0)/(UPDATE_RATE_HZ*MAX(1,double_time)));
}

void AP_Quicktune::restore_all_params()
{
    for (uint8_t p = 0; p < uint8_t(Param::END); p++) {
        const auto param = Param(p);
        if (BIT_IS_SET(param_changed, p)) {
            set_param_value(param, param_saved[p]);
            BIT_CLEAR(param_changed, p);
        }
    }
}

void AP_Quicktune::save_all_params()
{
    for (uint8_t p = 0; p < uint8_t(Param::END); p++) {
        const auto param = Param(p);
        set_and_save_param_value(param, get_param_value(param));
        param_saved[p] = get_param_value(param);
        BIT_CLEAR(param_changed, p);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Quicktune: Saved");
}

AP_Quicktune::Param AP_Quicktune::get_pname(AP_Quicktune::AxisName axis, AP_Quicktune::Stage stage) const
{
    const uint8_t axis_offset = uint8_t(axis) * param_per_axis;
    switch (stage)
    {
    case Stage::P:
        return Param(uint8_t(Param::RLL_P) + axis_offset);
    case Stage::D:
        return Param(uint8_t(Param::RLL_D) + axis_offset);
    case Stage::FLTT:
        return Param(uint8_t(Param::RLL_FLTT) + axis_offset);
    case Stage::FLTD:
        return Param(uint8_t(Param::RLL_FLTD) + axis_offset);
    default:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return Param::END;
    }
}

AP_Quicktune::Stage AP_Quicktune::get_stage(AP_Quicktune::Param param) const
{
    if (param == Param::RLL_P || param == Param::PIT_P || param == Param::YAW_P) {
        return Stage::P;
    } else {
        return Stage::END; //Means "anything but P gain"
    }
}

AP_Float *AP_Quicktune::get_param_pointer(AP_Quicktune::Param param) const
{
    const AxisName axis = get_axis(param);
    auto *pid = get_axis_pid(axis);
    if (pid == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return nullptr;
    }
    const Param roll_param = Param(uint8_t(param) % param_per_axis);
    switch (roll_param)
    {
        case Param::RLL_P:
            return &pid->kP();
        case Param::RLL_I:
            return &pid->kI();
        case Param::RLL_D:
            return &pid->kD();
        case Param::RLL_SMAX:
            return &pid->slew_limit();
        case Param::RLL_FLTT:
            return &pid->filt_T_hz();
        case Param::RLL_FLTD:
            return &pid->filt_D_hz();
        case Param::RLL_FLTE:
            return &pid->filt_E_hz();
        case Param::RLL_FF:
            return &pid->ff();
        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return nullptr;
    }
}

float AP_Quicktune::get_param_value(AP_Quicktune::Param param) const
{
    AP_Float *ptr = get_param_pointer(param);
    if (ptr != nullptr) {
        return ptr->get();
    }
    return 0.0;
}

void AP_Quicktune::set_param_value(AP_Quicktune::Param param, float value)
{
    AP_Float *ptr = get_param_pointer(param);
    if (ptr != nullptr) {
       ptr->set(value);
    }
}

void AP_Quicktune::set_and_save_param_value(AP_Quicktune::Param param, float value)
{
    AP_Float *ptr = get_param_pointer(param);
    if (ptr != nullptr) {
       ptr->set_and_save(value);
    }
}

AP_Quicktune::AxisName AP_Quicktune::get_axis(AP_Quicktune::Param param) const
{
    if (param >= Param::END) {
        return AxisName::END;
    }
    return AxisName(uint8_t(param) / param_per_axis);
}

const char* AP_Quicktune::get_axis_name(AP_Quicktune::AxisName axis) const
{
    switch (axis)
    {
        case AxisName::RLL:
            return "Roll";
        case AxisName::PIT:
            return "Pitch";
        case AxisName::YAW:
            return "Yaw";
        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return "UNK";
    }
}

float AP_Quicktune::gain_limit(AP_Quicktune::Param param) const
{
    if (get_axis(param) == AxisName::YAW) {
        if (param == Param::YAW_P) {
            return yaw_p_max;
        }
        if (param == Param::YAW_D) {
            return yaw_d_max;
        }
    }
    return 0.0;
}


// @LoggerMessage: QWIK
// @Description: Quicktune
// @Field: TimeUS: Time since system startup
// @Field: ParamNo: number of parameter being tuned
// @Field: SRate: slew rate
// @Field: Gain: test gain for current axis and PID element
// @Field: Param: name of parameter being being tuned
void AP_Quicktune::Write_QWIK(float srate, float gain, AP_Quicktune::Param param)
{
#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("QWIK","TimeUS,ParamNo,SRate,Gain,Param", "s#---", "-----", "QBffN",
                                AP_HAL::micros64(),
                                unsigned(param),
                                srate,
                                gain,
                                get_param_name(param));
#endif
}

#endif //AP_QUICKTUNE_ENABLED
