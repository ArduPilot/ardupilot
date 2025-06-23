#include "AP_Arming_Sub.h"
#include "Sub.h"

bool AP_Arming_Sub::rc_calibration_checks(bool display_failure)
{
    const RC_Channel *channels[] = {
        sub.channel_roll,
        sub.channel_pitch,
        sub.channel_throttle,
        sub.channel_yaw
    };
    return rc_checks_copter_sub(display_failure, channels);
}

bool AP_Arming_Sub::has_disarm_function() const {
    bool has_shift_function = false;
    // make sure the craft has a disarm button assigned before it is armed
    // check all the standard btn functions
    for (uint8_t i = 0; i < 16; i++) {
        switch (sub.get_button(i)->function(false)) {
            case JSButton::k_shift :
                has_shift_function = true;
                break;
            case JSButton::k_arm_toggle :
                return true;
            case JSButton::k_disarm :
                return true;
        }
    }

    // check all the shift functions if there's shift assigned
    if (has_shift_function) {
        for (uint8_t i = 0; i < 16; i++) {
            switch (sub.get_button(i)->function(true)) {
                case JSButton::k_arm_toggle :
                case JSButton::k_disarm :
                    return true;
            }
        }
    }
    // check if an AUX function that disarms or estops is setup
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARMDISARM) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP)) {
        return true;
    }  
    return false;
}

bool AP_Arming_Sub::pre_arm_checks(bool display_failure)
{
    if (armed) {
        return true;
    }
    // don't allow arming unless there is a disarm button configured
    if (!has_disarm_function()) {
        check_failed(display_failure, "Must assign a disarm or arm_toggle button or disarm aux function");
        return false;
    }

    return AP_Arming::pre_arm_checks(display_failure);
}

bool AP_Arming_Sub::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional sub-specific checks
    if (check_enabled(Check::INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(false, failure_msg, sizeof(failure_msg))) {
            check_failed(Check::INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

bool AP_Arming_Sub::arm(AP_Arming::Method method, bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }

    in_arm_motors = true;

    //if RC checks enabled, and RC_OPTIONS enabled for "0" throttle, and enabled check for throttle within trim position
    if (check_enabled(Check::RC) &&
     rc().option_is_enabled(RC_Channels::Option::ARMING_CHECK_THROTTLE) &&
     (sub.g.thr_arming_position == WITHIN_THR_TRIM)) {
        const char *rc_item = "Throttle";
        // check throttle is within trim+/- dz, ie centered throttle
        if (!sub.channel_throttle->in_trim_dz()) {
           check_failed(Check::RC, true, "%s not centered/close to trim", rc_item);
           AP_Notify::events.arming_failed = true;
           in_arm_motors = false;
           return false;
        }
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

#if HAL_LOGGING_ENABLED
    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);
#endif

    // disable cpu failsafe because initialising everything takes a while
    sub.mainloop_failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

    send_arm_disarm_statustext("Arming motors");

    AP_AHRS &ahrs = AP::ahrs();

    sub.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)

        // Always use absolute altitude for ROV
        // ahrs.resetHeightDatum();
        // AP::logger().Write_Event(LogEvent::EKF_ALT_RESET);
    } else if (!ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!sub.set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    hal.util->set_soft_armed(true);

    // enable output to motors
    sub.enable_motor_output();

    // finally actually arm the motors
    sub.motors.armed(true);

#if HAL_LOGGING_ENABLED
    // log flight mode in case it was changed while vehicle was disarmed
    AP::logger().Write_Mode((uint8_t)sub.control_mode, sub.control_mode_reason);
#endif

    // reenable failsafe
    sub.mainloop_failsafe_enable();

    // perf monitor ignores delay due to arming
    AP::scheduler().perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // if we do not have an ekf origin then we can't use the WMM tables
    if (!sub.ensure_ekf_origin()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Compass performance degraded");
        if (check_enabled(Check::PARAMETERS)) {
            check_failed(Check::PARAMETERS, true, "No world position, check ORIGIN_* parameters");
            return false;
        }
    }
    // return success
    return true;
}

bool AP_Arming_Sub::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // return immediately if we are already disarmed
    if (!sub.motors.armed()) {
        return false;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    send_arm_disarm_statustext("Disarming motors");

    auto &ahrs = AP::ahrs();

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && AP::compass().get_learn_type() == Compass::LearnType::COPY_FROM_EKF) {
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                AP::compass().set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // send disarm command to motors
    sub.motors.armed(false);

    // reset the mission
    sub.mission.reset();

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif

    hal.util->set_soft_armed(false);

    // clear input holds
    sub.clear_input_hold();

    return true;
}
