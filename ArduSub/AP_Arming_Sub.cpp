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
    return false;
}

bool AP_Arming_Sub::pre_arm_checks(bool display_failure)
{
    if (armed) {
        return true;
    }
    // don't allow arming unless there is a disarm button configured
    if (!has_disarm_function()) {
        check_failed(display_failure, "Must assign a disarm or arm_toggle button");
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
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!AP::ahrs().prearm_healthy()) {
            const char *reason = AP::ahrs().prearm_failure_reason();
            if (reason == nullptr) {
                reason = "AHRS not healthy";
            }
            check_failed(ARMING_CHECK_INS, display_failure, "%s", reason);
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

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);

    // disable cpu failsafe because initialising everything takes a while
    sub.mainloop_failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    AP_AHRS &ahrs = AP::ahrs();

    sub.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)

        // Always use absolute altitude for ROV
        // ahrs.resetHeightDatum();
        // AP::logger().Write_Event(LogEvent::EKF_ALT_RESET);
    } else if (ahrs.home_is_set() && !ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!sub.set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

    // enable output to motors
    sub.enable_motor_output();

    // finally actually arm the motors
    sub.motors.armed(true);

    // log flight mode in case it was changed while vehicle was disarmed
    AP::logger().Write_Mode(sub.control_mode, sub.control_mode_reason);

    // reenable failsafe
    sub.mainloop_failsafe_enable();

    // perf monitor ignores delay due to arming
    AP::scheduler().perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

bool AP_Arming_Sub::disarm()
{
    // return immediately if we are already disarmed
    if (!sub.motors.armed()) {
        return false;
    }

    if (!AP_Arming::disarm()) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors");
#endif

    AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && AP::compass().get_learn_type() == Compass::LEARN_EKF) {
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

    AP::logger().set_vehicle_armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    // clear input holds
    sub.clear_input_hold();

    return true;
}
