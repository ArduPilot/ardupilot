#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500         // restart tuning if pilot has left sticks in middle for 2 seconds
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 # define AUTOTUNE_LEVEL_ANGLE_CD           500         // angle which qualifies as level (Plane uses more relaxed 5deg)
 # define AUTOTUNE_LEVEL_RATE_RP_CD         1000        // rate which qualifies as level for roll and pitch (Plane uses more relaxed 10deg/sec)
#else
 # define AUTOTUNE_LEVEL_ANGLE_CD           250         // angle which qualifies as level
 # define AUTOTUNE_LEVEL_RATE_RP_CD         500         // rate which qualifies as level for roll and pitch
#endif
#define AUTOTUNE_LEVEL_RATE_Y_CD            750         // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     250         // time we require the aircraft to be level before starting next test
#define AUTOTUNE_LEVEL_TIMEOUT_MS           2000        // time out for level
#define AUTOTUNE_LEVEL_WARNING_INTERVAL_MS  5000        // level failure warning messages sent at this interval to users

AC_AutoTune::AC_AutoTune()
{
}

// autotune_init - should be called when autotune mode is selected
bool AC_AutoTune::init_internals(bool _use_poshold,
                                 AC_AttitudeControl *_attitude_control,
                                 AC_PosControl *_pos_control,
                                 AP_AHRS_View *_ahrs_view)
{
    use_poshold = _use_poshold;
    attitude_control = _attitude_control;
    pos_control = _pos_control;
    ahrs_view = _ahrs_view;
    motors = AP_Motors::get_singleton();
    const uint32_t now_ms = AP_HAL::millis();

    // exit immediately if motor are not armed
    if ((motors == nullptr) || !motors->armed()) {
        return false;
    }

    // initialise position controller
    init_position_controller();

    switch (mode) {
    case TuneMode::FAILED:
        // Fall through to restart the tuning process from scratch
        FALLTHROUGH;

    case TuneMode::UNINITIALISED:
        // First-time run: store the current gains as the baseline (original gains)
        backup_gains_and_initialise();
        // Set the mode to TUNING to begin the autotune process
        mode = TuneMode::TUNING;
        // Notify GCS that autotune has started
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case TuneMode::TUNING:
        // Resuming from previous tuning session, restart from current axis and tune step
        reset_vehicle_test_variables();
        step = Step::WAITING_FOR_LEVEL;
        step_start_time_ms = now_ms;
        level_start_time_ms = now_ms;
        // Reload gains with low I-term and restart logging
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_RESTART);
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case TuneMode::FINISHED:
    case TuneMode::VALIDATING:
        // The user is now validating the tuned gains in flight
        mode = TuneMode::VALIDATING;
        update_gcs(AUTOTUNE_MESSAGE_TESTING);
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_PILOT_TESTING);
        break;
    }

    have_position = false;

    return true;
}

// stop - should be called when the ch7/ch8 switch is switched OFF
void AC_AutoTune::stop()
{
    // set gains to their original values
    load_gains(GainType::ORIGINAL);

    update_gcs(AUTOTUNE_MESSAGE_STOPPED);

    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_OFF);

    // Note: we leave the mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// Autotune aux function trigger
void AC_AutoTune::do_aux_function(const RC_Channel::AuxSwitchPos ch_flag)
{
    if (mode != TuneMode::FINISHED) {
        if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {
            gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: must be complete to test gains");
        }
        return;
    }

    switch(ch_flag) {
        case RC_Channel::AuxSwitchPos::LOW:
            // load original gains
            load_gains(GainType::ORIGINAL);
            update_gcs(AUTOTUNE_MESSAGE_TESTING_END);
            break;
        case RC_Channel::AuxSwitchPos::MIDDLE:
            // Middle position is unused for now_ms
            break;
        case RC_Channel::AuxSwitchPos::HIGH:
            // Load tuned gains
            load_gains(GainType::TUNED);
            update_gcs(AUTOTUNE_MESSAGE_TESTING);
            break;
    }

    testing_switch_used = true;
}

// Possibly save gains, called on disarm
void AC_AutoTune::disarmed(const bool in_autotune_mode)
{
    // True if pilot is testing tuned gains
    const bool testing_tuned = testing_switch_used && (loaded_gains == GainType::TUNED);

    // True if in autotune mode and no pilot testing commands have been received
    const bool tune_complete_no_testing = !testing_switch_used && in_autotune_mode;

    if (tune_complete_no_testing || testing_tuned) {
        save_tuning_gains();
    } else {
        reset();
    }
}

// initialise position controller
bool AC_AutoTune::init_position_controller(void)
{
    // initialize vertical maximum speeds and acceleration
    init_z_limits();

    // initialise the vertical position controller
    pos_control->D_init_controller();

    return true;
}

void AC_AutoTune::send_step_string()
{
    if (pilot_override) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Paused: Pilot Override Active");
        return;
    }
    switch (step) {
    case Step::WAITING_FOR_LEVEL:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Leveling");
        return;
    case Step::UPDATE_GAINS:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Updating Gains");
        return;
    case Step::ABORT:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Aborting Test");
        return;
    case Step::EXECUTING_TEST:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Testing");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: unknown step");
}

const char *AC_AutoTune::get_tune_type_name() const
{
    switch (tune_type) {
    case TuneType::RATE_D_UP:
        return "Rate D Up";
    case TuneType::RATE_D_DOWN:
        return "Rate D Down";
    case TuneType::RATE_P_UP:
        return "Rate P Up";
    case TuneType::RATE_FF_UP:
        return "Rate FF Up";
    case TuneType::ANGLE_P_UP:
        return "Angle P Up";
    case TuneType::ANGLE_P_DOWN:
        return "Angle P Down";
    case TuneType::MAX_GAINS:
        return "Find Max Gains";
    case TuneType::TUNE_CHECK:
        return "Check Tune Frequency Response";
    case TuneType::TUNE_COMPLETE:
        return "Tune Complete";
    }
    return "";
    // this should never happen
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
}

// return current axis string
const char *AC_AutoTune::get_axis_name() const
{
    switch (axis) {
    case AxisType::ROLL:
        return "Roll";
    case AxisType::PITCH:
        return "Pitch";
    case AxisType::YAW:
        return "Yaw(E)";
    case AxisType::YAW_D:
        return "Yaw(D)";
    }
    return "";
}

// Main update loop for Autotune mode. Handles all states: tuning, validating, or idle.
// Should be called at ≥100Hz for consistent performance.
void AC_AutoTune::run()
{
    // Initialise vertical climb rate and acceleration limits
    init_z_limits();

    // Exit early if the vehicle is disarmed or motor interlock is not enabled
    // (this condition should not occur if init() is working correctly)
    if (!motors->armed() || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        pos_control->D_relax_controller(0.0f);
        return;
    }

    float desired_yaw_rate_rads;   // used during manual control
    get_pilot_desired_rp_yrate_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rate_rads);

    // Get pilot's desired climb rate
    const float target_climb_rate_ms = get_desired_climb_rate_ms();

    const bool zero_rp_input = is_zero(desired_roll_rad) && is_zero(desired_pitch_rad);
    if (zero_rp_input) {
        // Use position hold if enabled
        get_poshold_attitude_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rad);
    }

    const uint32_t now_ms = AP_HAL::millis();

    switch (mode) {
    case TuneMode::TUNING:
        // Detect pilot override
        if (!zero_rp_input || !is_zero(desired_yaw_rate_rads) || !is_zero(target_climb_rate_ms)) {
            if (!pilot_override) {
                pilot_override = true;
                // Restore original gains while pilot is in control
            }
            // Update last override time
            override_time = now_ms;
            if (!zero_rp_input) {
                // Invalidate position hold if pilot inputs roll/pitch
                have_position = false;
            }
        } else if (pilot_override) {
            // Check if pilot has released sticks long enough to resume tuning
            if (now_ms - override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                pilot_override = false;
                step = Step::WAITING_FOR_LEVEL;
                step_start_time_ms = now_ms;
                level_start_time_ms = now_ms;
                // TODO: Consider using our current target.
                desired_yaw_rad = ahrs_view->get_yaw_rad(); // Reset yaw reference
            }
        }

        if (pilot_override) {
            // Pilot is actively controlling the vehicle; fly on original gains
            if (now_ms - last_pilot_override_warning > 1000) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: pilot overrides active");
                last_pilot_override_warning = now_ms;
            }
            load_gains(GainType::ORIGINAL);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rate_rads);
        } else {
            // Autotune controls the aircraft
            control_attitude();
            do_gcs_announcements();
        }
        break;

    case TuneMode::UNINITIALISED:
        // Should never reach this state; init() must be called before run()
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        FALLTHROUGH;

    case TuneMode::FAILED:
        FALLTHROUGH;

    case TuneMode::FINISHED:
        // Tuning is complete or failed; fly using original gains
        load_gains(GainType::ORIGINAL);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rate_rads);
        break;

    case TuneMode::VALIDATING:
        // Pilot is evaluating tuned gains
        load_gains(GainType::TUNED);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rate_rads);
        break;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Update vertical position controller with pilot climb rate input
    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
    pos_control->D_update_controller();
}

// return true if vehicle is close to level
bool AC_AutoTune::currently_level()
{
    // abort AutoTune if we pass 2 * AUTOTUNE_LEVEL_TIMEOUT_MS
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - level_start_time_ms > 3 * AUTOTUNE_LEVEL_TIMEOUT_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AutoTune: Failed to level, please tune manually");
        mode = TuneMode::FAILED;
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
    }

    // slew threshold to ensure sufficient settling time for aircraft unable to obtain small thresholds
    // relax threshold if we pass AUTOTUNE_LEVEL_TIMEOUT_MS
    const float threshold_mul = constrain_float((float)(now_ms - level_start_time_ms) / (float)AUTOTUNE_LEVEL_TIMEOUT_MS, 0.0, 2.0);

    if (fabsf(ahrs_view->get_roll_rad() - desired_roll_rad) > threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (fabsf(ahrs_view->get_pitch_rad() - desired_pitch_rad) > threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (fabsf(wrap_PI(ahrs_view->get_yaw_rad() - desired_yaw_rad)) > threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (ahrs_view->get_gyro().x > threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (ahrs_view->get_gyro().y > threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (ahrs_view->get_gyro().z > threshold_mul * cd_to_rad(AUTOTUNE_LEVEL_RATE_Y_CD)) {
        return false;
    }
    return true;
}

// Main tuning state machine. Handles all StepTypes: WAITING_FOR_LEVEL, EXECUTING_TEST, UPDATE_GAINS, ABORT.
// Updates attitude controller targets and processes test results to adjust PID gains.
void AC_AutoTune::control_attitude()
{
    rotation_rate = 0.0f;
    lean_angle = 0.0f;
    const float direction_sign = positive_direction ? 1.0f : -1.0f;
    const uint32_t now_ms = AP_HAL::millis();

    switch (step) {

    case Step::WAITING_FOR_LEVEL: {
        // Use intra-test gains while holding level between tests
        load_gains(GainType::INTRA_TEST);
        
        attitude_control->input_euler_angle_roll_pitch_yaw_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rad, true);

        // Require a short stable period before executing the next test
        if (!currently_level()) {
            step_start_time_ms = now_ms;
        }

        if (now_ms - step_start_time_ms > AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            // Begin the test phase
            step = Step::EXECUTING_TEST;
            step_start_time_ms = now_ms;
            step_timeout_ms = get_testing_step_timeout_ms();

            // Record starting angular position and rate
            switch (axis) {
            case AxisType::ROLL:
                start_rate = degrees(ahrs_view->get_gyro().x) * 100.0f;
                start_angle = ahrs_view->roll_sensor;
                break;
            case AxisType::PITCH:
                start_rate = degrees(ahrs_view->get_gyro().y) * 100.0f;
                start_angle = ahrs_view->pitch_sensor;
                break;
            case AxisType::YAW:
            case AxisType::YAW_D:
                start_rate = degrees(ahrs_view->get_gyro().z) * 100.0f;
                start_angle = ahrs_view->yaw_sensor;
                break;
            }

            // Apply test gains and initialise test-specific variables
            load_gains(GainType::TEST);
            test_init();
        }
        break;
    }

    case Step::EXECUTING_TEST: {
        // Run the test with current trial gains
        load_gains(GainType::TEST);
        test_run(axis, direction_sign);

        // Detect failure due to reverse response or excessive lean angle
        if (lean_angle <= -angle_lim_neg_rpy_cd() ||
            attitude_control->lean_angle_deg() * 100 > angle_lim_max_rp_cd()) {
            step = Step::ABORT;
        }

#if HAL_LOGGING_ENABLED
        Log_AutoTuneDetails();
        attitude_control->Write_Rate(*pos_control);
        log_pids();
#endif

        // Update yaw target for next test if required
        if (axis == AxisType::YAW || axis == AxisType::YAW_D) {
            desired_yaw_rad = ahrs_view->get_yaw_rad();
        }
        break;
    }

    case Step::UPDATE_GAINS:

#if HAL_LOGGING_ENABLED
        Log_AutoTune(); // Log gain adjustment results
#endif

        // Announce test results before gains are changed
        do_post_test_gcs_announcements();

        // Update gains based on the current tuning strategy
        switch (tune_type) {
        case TuneType::RATE_D_UP:
            updating_rate_d_up_all(axis);
            break;
        case TuneType::RATE_D_DOWN:
            updating_rate_d_down_all(axis);
            break;
        case TuneType::RATE_P_UP:
            updating_rate_p_up_all(axis);
            break;
        case TuneType::ANGLE_P_DOWN:
            updating_angle_p_down_all(axis);
            break;
        case TuneType::ANGLE_P_UP:
            updating_angle_p_up_all(axis);
            break;
        case TuneType::RATE_FF_UP:
            updating_rate_ff_up_all(axis);
            break;
        case TuneType::MAX_GAINS:
            updating_max_gains_all(axis);
            break;
        case TuneType::TUNE_CHECK:
            success_counter = AUTOTUNE_SUCCESS_COUNT;
            FALLTHROUGH;
        case TuneType::TUNE_COMPLETE:
            break;
        }

        // If tuning step was successful, proceed to the next step
        if (success_counter >= AUTOTUNE_SUCCESS_COUNT) {
            success_counter = 0;
            step_scaler = 1.0f;
            set_tuning_gains_with_backoff(axis);
            next_tune_type(tune_type, false);

            if (tune_type == TuneType::TUNE_COMPLETE) {
                // Complete tuning for this axis and determine the next one
                next_tune_type(tune_type, true);
                report_final_gains(axis);

                bool complete = false;
                switch (axis) {
                case AxisType::ROLL:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_ROLL;
                    if (pitch_enabled()) {
                        axis = AxisType::PITCH;
                    } else if (yaw_enabled()) {
                        axis = AxisType::YAW;
                    } else if (yaw_d_enabled()) {
                        axis = AxisType::YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case AxisType::PITCH:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_PITCH;
                    if (yaw_enabled()) {
                        axis = AxisType::YAW;
                    } else if (yaw_d_enabled()) {
                        axis = AxisType::YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case AxisType::YAW:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW;
                    if (yaw_d_enabled()) {
                        axis = AxisType::YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case AxisType::YAW_D:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW_D;
                    complete = true;
                    break;
                }

                if (complete) {
                    mode = TuneMode::FINISHED;
                    update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = true;
                    load_gains(GainType::ORIGINAL); // Reset for landing
                } else {
                    AP_Notify::events.autotune_next_axis = true;
                    reset_update_gain_variables();
                }
            }
        }
        FALLTHROUGH;

    case Step::ABORT:
        // Recover from failed test or move on after a successful one
        
        attitude_control->input_euler_angle_roll_pitch_yaw_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rad, true);

        load_gains(GainType::INTRA_TEST);

        step = Step::WAITING_FOR_LEVEL;
        positive_direction = reverse_test_direction();
        step_start_time_ms = now_ms;
        level_start_time_ms = now_ms;
        step_timeout_ms = AUTOTUNE_REQUIRED_LEVEL_TIME_MS;
        break;
    }
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void AC_AutoTune::backup_gains_and_initialise()
{
    const uint32_t now_ms = AP_HAL::millis();
    
    // initialise state because this is our first time
    if (roll_enabled()) {
        axis = AxisType::ROLL;
    } else if (pitch_enabled()) {
        axis = AxisType::PITCH;
    } else if (yaw_enabled()) {
        axis = AxisType::YAW;
    } else if (yaw_d_enabled()) {
        axis = AxisType::YAW_D;
    }
    // no axes are complete
    axes_completed = 0;

    // reset update gain variables for each vehicle
    reset_update_gain_variables();

    // start at the beginning of tune sequence
    next_tune_type(tune_type, true);

    step = Step::WAITING_FOR_LEVEL;
    positive_direction = false;
    step_start_time_ms = now_ms;
    level_start_time_ms = now_ms;
    step_scaler = 1.0f;

    desired_yaw_rad = ahrs_view->get_yaw_rad();
}

/*
  load a specified set of gains
 */
void AC_AutoTune::load_gains(enum GainType gain_type)
{
    if (loaded_gains == gain_type) {
        // Loaded gains are already of correct type
        return;
    }
    loaded_gains = gain_type;

    switch (gain_type) {
    case GainType::ORIGINAL:
        load_orig_gains();
        break;
    case GainType::INTRA_TEST:
        load_intra_test_gains();
        break;
    case GainType::TEST:
        load_test_gains();
        break;
    case GainType::TUNED:
        load_tuned_gains();
        break;
    }
}

// update_gcs - send message to ground station
void AC_AutoTune::update_gcs(uint8_t message_id) const
{
    switch (message_id) {
    case AUTOTUNE_MESSAGE_STARTED:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AutoTune: Started");
        break;
    case AUTOTUNE_MESSAGE_STOPPED:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AutoTune: Stopped");
        break;
    case AUTOTUNE_MESSAGE_SUCCESS:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: Success");
        break;
    case AUTOTUNE_MESSAGE_FAILED:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: Failed");
        break;
    case AUTOTUNE_MESSAGE_TESTING:
    case AUTOTUNE_MESSAGE_SAVED_GAINS:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s gains for %s%s%s%s",
                        (message_id == AUTOTUNE_MESSAGE_SAVED_GAINS) ? "Saved" : "Pilot Testing",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_ROLL)?"Roll ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_PITCH)?"Pitch ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW)?"Yaw(E)":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW_D)?"Yaw(D)":"");
        break;
    case AUTOTUNE_MESSAGE_TESTING_END:
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: original gains restored");
        break;
    }
}

// axis helper functions
bool AC_AutoTune::roll_enabled() const
{
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_ROLL;
}

bool AC_AutoTune::pitch_enabled() const
{
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_PITCH;
}

bool AC_AutoTune::yaw_enabled() const
{
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_YAW;
}

bool AC_AutoTune::yaw_d_enabled() const
{
#if APM_BUILD_TYPE(APM_BUILD_Heli)
    return false;
#else
    return get_axis_bitmask() & AUTOTUNE_AXIS_BITMASK_YAW_D;
#endif
}

/*
  check if we have a good position estimate
 */
bool AC_AutoTune::position_ok(void)
{
    if (!AP::ahrs().have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status {}; 
    AP::ahrs().get_filter_status(filt_status);

    // require a good absolute position and EKF must not be in const_pos_mode
    return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
}

// get attitude for slow position hold in autotune mode
void AC_AutoTune::get_poshold_attitude_rad(float &roll_out_rad, float &pitch_out_rad, float &yaw_out_rad)
{
    roll_out_rad = pitch_out_rad = 0;

    if (!use_poshold) {
        // we are not trying to hold position
        return;
    }

    // do we know where we are? If not then don't do poshold
    if (!position_ok()) {
        return;
    }

    if (!have_position) {
        have_position = true;
        start_position_ned_m = pos_control->get_pos_estimate_NED_m();
    }

    // don't go past 10 degrees, as autotune result would deteriorate too much
    const float angle_max_rad = radians(10.0);

    // hit the 10 degree limit at 20 meters position error
    const float dist_limit_m = 20.00;

    // we only start adjusting yaw if we are more than 5m from the
    // target position. That corresponds to a lean angle of 2.5 degrees
    const float yaw_dist_limit_m = 5.0;

    Vector3f pos_error_ned_m = (pos_control->get_pos_estimate_NED_m() - start_position_ned_m).tofloat();
    pos_error_ned_m.z = 0;
    float dist_m = pos_error_ned_m.length();
    if (dist_m < 0.10) {
        // don't do anything within 10cm
        return;
    }

    /*
      very simple linear controller
     */
    float scaling = constrain_float(angle_max_rad * dist_m / dist_limit_m, 0, angle_max_rad);
    Vector2f angle_ne(pos_error_ned_m.x, pos_error_ned_m.y);
    angle_ne *= scaling / dist_m;

    // rotate into body frame
    pitch_out_rad = angle_ne.x * ahrs_view->cos_yaw() + angle_ne.y * ahrs_view->sin_yaw();
    roll_out_rad  = angle_ne.x * ahrs_view->sin_yaw() - angle_ne.y * ahrs_view->cos_yaw();

    if (dist_m < yaw_dist_limit_m) {
        // no yaw adjustment
        return;
    }

    /*
      also point so that test occurs perpendicular to the wind,
      if we have drifted more than yaw_dist_limit_m from the desired
      position. This ensures that autotune doesn't have to deal with
      more than 2.5 degrees of attitude on the axis it is tuning
     */
    float target_yaw_rad = atan2f(pos_error_ned_m.y, pos_error_ned_m.x);
    if (axis == AxisType::PITCH) {
        // for roll and yaw tuning we point along the wind, for pitch
        // we point across the wind
        target_yaw_rad += radians(90);
    }
    // go to the nearest 180 degree mark, with 5 degree slop to prevent oscillation
    if (fabsf(yaw_out_rad - target_yaw_rad) > radians(95.0)) {
        target_yaw_rad += radians(180.0);
    }

    yaw_out_rad = target_yaw_rad;
}

// get the next tune type
void AC_AutoTune::next_tune_type(TuneType &curr_tune_type, bool reset)
{
    if (reset) {
        set_tune_sequence();
        tune_seq_index = 0;
    } else if (curr_tune_type == TuneType::TUNE_COMPLETE) {
        // leave tune_type as TUNE_COMPLETE to initiate next axis or exit autotune
        return;
    } else {
        tune_seq_index++;
    }

    curr_tune_type = tune_seq[tune_seq_index];
}

#endif  // AC_AUTOTUNE_ENABLED
