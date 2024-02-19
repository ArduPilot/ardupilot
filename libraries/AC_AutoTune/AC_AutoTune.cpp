#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 # define AUTOTUNE_LEVEL_ANGLE_CD           500     // angle which qualifies as level (Plane uses more relaxed 5deg)
 # define AUTOTUNE_LEVEL_RATE_RP_CD        1000     // rate which qualifies as level for roll and pitch (Plane uses more relaxed 10deg/sec)
#else
 # define AUTOTUNE_LEVEL_ANGLE_CD           250     // angle which qualifies as level
 # define AUTOTUNE_LEVEL_RATE_RP_CD         500     // rate which qualifies as level for roll and pitch
#endif
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     250     // time we require the aircraft to be level before starting next test
#define AUTOTUNE_LEVEL_TIMEOUT_MS          2000     // time out for level
#define AUTOTUNE_LEVEL_WARNING_INTERVAL_MS 5000     // level failure warning messages sent at this interval to users
#define AUTOTUNE_ANGLE_MAX_RLLPIT_SCALE     2.0 / 3.0   // maximum allowable angle during testing, as a fraction of angle_max

AC_AutoTune::AC_AutoTune()
{
}

// autotune_init - should be called when autotune mode is selected
bool AC_AutoTune::init_internals(bool _use_poshold,
                                 AC_AttitudeControl *_attitude_control,
                                 AC_PosControl *_pos_control,
                                 AP_AHRS_View *_ahrs_view,
                                 AP_InertialNav *_inertial_nav)
{
    use_poshold = _use_poshold;
    attitude_control = _attitude_control;
    pos_control = _pos_control;
    ahrs_view = _ahrs_view;
    inertial_nav = _inertial_nav;
    motors = AP_Motors::get_singleton();
    const uint32_t now = AP_HAL::millis();

    // exit immediately if motor are not armed
    if ((motors == nullptr) || !motors->armed()) {
        return false;
    }

    // initialise position controller
    init_position_controller();

    switch (mode) {
    case FAILED:
        // fall through to restart the tuning
        FALLTHROUGH;

    case UNINITIALISED:
        // autotune has never been run
        // so store current gains as original gains
        backup_gains_and_initialise();
        // advance mode to tuning
        mode = TUNING;
        // send message to ground station that we've started tuning
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case TUNING:
        // reset test variables for each vehicle
        reset_vehicle_test_variables();

        // we are restarting tuning so restart where we left off
        step = WAITING_FOR_LEVEL;
        step_start_time_ms = now;
        level_start_time_ms = now;
        // reset gains to tuning-start gains (i.e. low I term)
        load_gains(GAIN_INTRA_TEST);
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_RESTART);
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case SUCCESS:
        // we have completed a tune and the pilot wishes to test the new gains
        load_gains(GAIN_TUNED);
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
    load_gains(GAIN_ORIGINAL);

    // re-enable angle-to-rate request limits
    attitude_control->use_sqrt_controller(true);

    update_gcs(AUTOTUNE_MESSAGE_STOPPED);

    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_OFF);

    // Note: we leave the mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// initialise position controller
bool AC_AutoTune::init_position_controller(void)
{
    // initialize vertical maximum speeds and acceleration
    init_z_limits();

    // initialise the vertical position controller
    pos_control->init_z_controller();

    return true;
}

void AC_AutoTune::send_step_string()
{
    if (pilot_override) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Paused: Pilot Override Active");
        return;
    }
    switch (step) {
    case WAITING_FOR_LEVEL:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Leveling");
        return;
    case UPDATE_GAINS:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Updating Gains");
        return;
    case TESTING:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Testing");
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: unknown step");
}

const char *AC_AutoTune::type_string() const
{
    switch (tune_type) {
    case RD_UP:
        return "Rate D Up";
    case RD_DOWN:
        return "Rate D Down";
    case RP_UP:
        return "Rate P Up";
    case RFF_UP:
        return "Rate FF Up";
    case SP_UP:
        return "Angle P Up";
    case SP_DOWN:
        return "Angle P Down";
    case MAX_GAINS:
        return "Find Max Gains";
    case TUNE_CHECK:
        return "Check Tune Frequency Response";
    case TUNE_COMPLETE:
        return "Tune Complete";
    }
    return "";
    // this should never happen
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
}

// return current axis string
const char *AC_AutoTune::axis_string() const
{
    switch (axis) {
    case ROLL:
        return "Roll";
    case PITCH:
        return "Pitch";
    case YAW:
        return "Yaw(E)";
    case YAW_D:
        return "Yaw(D)";
    }
    return "";
}

// run - runs the autotune flight mode
// should be called at 100hz or more
void AC_AutoTune::run()
{
    // initialize vertical speeds and acceleration
    init_z_limits();

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    // this should not actually be possible because of the init() checks
    if (!motors->armed() || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        pos_control->relax_z_controller(0.0f);
        return;
    }

    float target_roll_cd, target_pitch_cd, target_yaw_rate_cds;
    get_pilot_desired_rp_yrate_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    // get pilot desired climb rate
    const float target_climb_rate_cms = get_pilot_desired_climb_rate_cms();

    const bool zero_rp_input = is_zero(target_roll_cd) && is_zero(target_pitch_cd);

    const uint32_t now = AP_HAL::millis();

    if (mode != SUCCESS) {
        if (!zero_rp_input || !is_zero(target_yaw_rate_cds) || !is_zero(target_climb_rate_cms)) {
            if (!pilot_override) {
                pilot_override = true;
                // set gains to their original values
                load_gains(GAIN_ORIGINAL);
                attitude_control->use_sqrt_controller(true);
            }
            // reset pilot override time
            override_time = now;
            if (!zero_rp_input) {
                // only reset position on roll or pitch input
                have_position = false;
            }
        } else if (pilot_override) {
            // check if we should resume tuning after pilot's override
            if (now - override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                pilot_override = false;             // turn off pilot override
                // set gains to their intra-test values (which are very close to the original gains)
                // load_gains(GAIN_INTRA_TEST); //I think we should be keeping the originals here to let the I term settle quickly
                step = WAITING_FOR_LEVEL; // set tuning step back from beginning
                step_start_time_ms = now;
                level_start_time_ms = now;
                desired_yaw_cd = ahrs_view->yaw_sensor;
            }
        }
    }
    if (pilot_override) {
        if (now - last_pilot_override_warning > 1000) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: pilot overrides active");
            last_pilot_override_warning = now;
        }
    }
    if (zero_rp_input) {
        // pilot input on throttle and yaw will still use position hold if enabled
        get_poshold_attitude(target_roll_cd, target_pitch_cd, desired_yaw_cd);
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // if pilot override call attitude controller
    if (pilot_override || mode != TUNING) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);
    } else {
        // somehow get attitude requests from autotuning
        control_attitude();
        // tell the user what's going on
        do_gcs_announcements();
    }

    // call position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cms);
    pos_control->update_z_controller();

}

// return true if vehicle is close to level
bool AC_AutoTune::currently_level()
{
    // abort AutoTune if we pass 2 * AUTOTUNE_LEVEL_TIMEOUT_MS
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - level_start_time_ms > 2 * AUTOTUNE_LEVEL_TIMEOUT_MS) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "AutoTune: Failed to level, please tune manually");
        mode = FAILED;
        LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
    }

    // slew threshold to ensure sufficient settling time for aircraft unable to obtain small thresholds
    // relax threshold if we pass AUTOTUNE_LEVEL_TIMEOUT_MS
    const float threshold_mul = constrain_float((float)(now_ms - level_start_time_ms) / (float)AUTOTUNE_LEVEL_TIMEOUT_MS, 0.0, 2.0);

    if (fabsf(ahrs_view->roll_sensor - roll_cd) > threshold_mul * AUTOTUNE_LEVEL_ANGLE_CD) {
        return false;
    }

    if (fabsf(ahrs_view->pitch_sensor - pitch_cd) > threshold_mul * AUTOTUNE_LEVEL_ANGLE_CD) {
        return false;
    }
    if (fabsf(wrap_180_cd(ahrs_view->yaw_sensor - desired_yaw_cd)) > threshold_mul * AUTOTUNE_LEVEL_ANGLE_CD) {
        return false;
    }
    if ((ToDeg(ahrs_view->get_gyro().x) * 100.0f) > threshold_mul * AUTOTUNE_LEVEL_RATE_RP_CD) {
        return false;
    }
    if ((ToDeg(ahrs_view->get_gyro().y) * 100.0f) > threshold_mul * AUTOTUNE_LEVEL_RATE_RP_CD) {
        return false;
    }
    if ((ToDeg(ahrs_view->get_gyro().z) * 100.0f) > threshold_mul * AUTOTUNE_LEVEL_RATE_Y_CD) {
        return false;
    }
    return true;
}

// main state machine to level vehicle, perform a test and update gains
// directly updates attitude controller with targets
void AC_AutoTune::control_attitude()
{
    rotation_rate = 0.0f;        // rotation rate in radians/second
    lean_angle = 0.0f;
    const float direction_sign = positive_direction ? 1.0f : -1.0f;
    const uint32_t now = AP_HAL::millis();

    // check tuning step
    switch (step) {

    case WAITING_FOR_LEVEL: {

        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control->use_sqrt_controller(true);

        get_poshold_attitude(roll_cd, pitch_cd, desired_yaw_cd);

        // hold level attitude
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_cd, pitch_cd, desired_yaw_cd, true);

        // hold the copter level for 0.5 seconds before we begin a twitch
        // reset counter if we are no longer level
        if (!currently_level()) {
            step_start_time_ms = now;
        }

        // if we have been level for a sufficient amount of time (0.5 seconds) move onto tuning step
        if (now - step_start_time_ms > AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            // initiate variables for next step
            step = TESTING;
            step_start_time_ms = now;
            step_time_limit_ms = get_testing_step_timeout_ms();
            // set gains to their to-be-tested values
            twitch_first_iter = true;
            test_rate_max = 0.0f;
            test_rate_min = 0.0f;
            test_angle_max = 0.0f;
            test_angle_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            load_gains(GAIN_TEST);
        } else {
            // when waiting for level we use the intra-test gains
            load_gains(GAIN_INTRA_TEST);
        }

        // Initialize test-specific variables
        switch (axis) {
        case ROLL:
            abort_angle = attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_RLLPIT_SCALE;
            start_rate = ToDeg(ahrs_view->get_gyro().x) * 100.0f;
            start_angle = ahrs_view->roll_sensor;
            break;
        case PITCH:
            abort_angle = attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_RLLPIT_SCALE;
            start_rate = ToDeg(ahrs_view->get_gyro().y) * 100.0f;
            start_angle = ahrs_view->pitch_sensor;
            break;
        case YAW:
        case YAW_D:
            // Aircraft with small lean angle will generally benefit from proportional smaller yaw twitch size
            abort_angle = attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_YAW_SCALE;
            start_rate = ToDeg(ahrs_view->get_gyro().z) * 100.0f;
            start_angle = ahrs_view->yaw_sensor;
            break;
        }

        // tests must be initialized last as some rely on variables above
        test_init();

        break;
    }

    case TESTING: {
        // Run the twitching step
        load_gains(GAIN_TEST);

        // run the test
        test_run(axis, direction_sign);

        // Check for failure causing reverse response
        if (lean_angle <= -attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_SCALE) {
            step = WAITING_FOR_LEVEL;
            positive_direction = twitch_reverse_direction();
            step_start_time_ms = now;
            level_start_time_ms = now;
        }

        // protect from roll over
        const float resultant_angle_cd = 100 * degrees(acosf(ahrs_view->cos_roll() * ahrs_view->cos_pitch()));
        if (resultant_angle_cd > attitude_control->lean_angle_max_cd() * AUTOTUNE_ANGLE_MAX_RLLPIT_SCALE) {
            step = WAITING_FOR_LEVEL;
            positive_direction = twitch_reverse_direction();
            step_start_time_ms = now;
            level_start_time_ms = now;
        }

#if HAL_LOGGING_ENABLED
        // log this iterations lean angle and rotation rate
        Log_AutoTuneDetails();
        ahrs_view->Write_Rate(*motors, *attitude_control, *pos_control);
        log_pids();
#endif

        if (axis == YAW || axis == YAW_D) {
            desired_yaw_cd = ahrs_view->yaw_sensor;
        }
        break;
    }

    case UPDATE_GAINS:

        // re-enable rate limits
        attitude_control->use_sqrt_controller(true);

#if HAL_LOGGING_ENABLED
        // log the latest gains
        Log_AutoTune();
#endif

        // Announce tune type test results
        // must be done before updating method because this method changes parameters for next test
        do_post_test_gcs_announcements();

        switch (tune_type) {
        // Check results after mini-step to increase rate D gain
        case RD_UP:
            updating_rate_d_up_all(axis);
            break;
        // Check results after mini-step to decrease rate D gain
        case RD_DOWN:
            updating_rate_d_down_all(axis);
            break;
        // Check results after mini-step to increase rate P gain
        case RP_UP:
            updating_rate_p_up_all(axis);
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_DOWN:
            updating_angle_p_down_all(axis);
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_UP:
            updating_angle_p_up_all(axis);
            break;
        case RFF_UP:
            updating_rate_ff_up_all(axis);
            break;
        case MAX_GAINS:
            updating_max_gains_all(axis);
            break;
        case TUNE_CHECK:
            counter = AUTOTUNE_SUCCESS_COUNT;
            FALLTHROUGH;
        case TUNE_COMPLETE:
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            counter = 0;

            // reset scaling factor
            step_scaler = 1.0f;


            // set gains for post tune before moving to the next tuning type
            set_gains_post_tune(axis);

            // increment the tune type to the next one in tune sequence
            next_tune_type(tune_type, false);

            if (tune_type == TUNE_COMPLETE) {
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                next_tune_type(tune_type, true);

                report_final_gains(axis);

                // advance to the next axis
                bool complete = false;
                switch (axis) {
                case ROLL:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_ROLL;
                    if (pitch_enabled()) {
                        axis = PITCH;
                    } else if (yaw_enabled()) {
                        axis = YAW;
                    } else if (yaw_d_enabled()) {
                        axis = YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case PITCH:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_PITCH;
                    if (yaw_enabled()) {
                        axis = YAW;
                    } else if (yaw_d_enabled()) {
                        axis = YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case YAW:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW;
                    if (yaw_d_enabled()) {
                        axis = YAW_D;
                    } else {
                        complete = true;
                    }
                    break;
                case YAW_D:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW_D;
                    complete = true;
                    break;
                }

                // if we've just completed all axes we have successfully completed the autotune
                // change to TESTING mode to allow user to fly with new gains
                if (complete) {
                    mode = SUCCESS;
                    update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = true;
                } else {
                    AP_Notify::events.autotune_next_axis = true;
                    reset_update_gain_variables();
                }
            }
        }

        if (axis == YAW || axis == YAW_D) {
            attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, ahrs_view->yaw_sensor, false);
        }

        // set gains to their intra-test values (which are very close to the original gains)
        load_gains(GAIN_INTRA_TEST);

        // reset testing step
        step = WAITING_FOR_LEVEL;
        positive_direction = twitch_reverse_direction();
        step_start_time_ms = now;
        level_start_time_ms = now;
        step_time_limit_ms = AUTOTUNE_REQUIRED_LEVEL_TIME_MS;
        break;
    }
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void AC_AutoTune::backup_gains_and_initialise()
{
    const uint32_t now = AP_HAL::millis();
    
    // initialise state because this is our first time
    if (roll_enabled()) {
        axis = ROLL;
    } else if (pitch_enabled()) {
        axis = PITCH;
    } else if (yaw_enabled()) {
        axis = YAW;
    } else if (yaw_d_enabled()) {
        axis = YAW_D;
    }
    // no axes are complete
    axes_completed = 0;

    // reset update gain variables for each vehicle
    reset_update_gain_variables();

    // start at the beginning of tune sequence
    next_tune_type(tune_type, true);

    step = WAITING_FOR_LEVEL;
    positive_direction = false;
    step_start_time_ms = now;
    level_start_time_ms = now;
    step_scaler = 1.0f;

    desired_yaw_cd = ahrs_view->yaw_sensor;
}

/*
  load a specified set of gains
 */
void AC_AutoTune::load_gains(enum GainType gain_type)
{
    switch (gain_type) {
    case GAIN_ORIGINAL:
        load_orig_gains();
        break;
    case GAIN_INTRA_TEST:
        load_intra_test_gains();
        break;
    case GAIN_TEST:
        load_test_gains();
        break;
    case GAIN_TUNED:
        load_tuned_gains();
        break;
    }
}

// update_gcs - send message to ground station
void AC_AutoTune::update_gcs(uint8_t message_id) const
{
    switch (message_id) {
    case AUTOTUNE_MESSAGE_STARTED:
        gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Started");
        break;
    case AUTOTUNE_MESSAGE_STOPPED:
        gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Stopped");
        break;
    case AUTOTUNE_MESSAGE_SUCCESS:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Success");
        break;
    case AUTOTUNE_MESSAGE_FAILED:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Failed");
        break;
    case AUTOTUNE_MESSAGE_TESTING:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Pilot Testing");
        break;
    case AUTOTUNE_MESSAGE_SAVED_GAINS:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Saved gains for %s%s%s%s",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_ROLL)?"Roll ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_PITCH)?"Pitch ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW)?"Yaw(E)":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW_D)?"Yaw(D)":"");
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
    nav_filter_status filt_status = inertial_nav->get_filter_status();

    // require a good absolute position and EKF must not be in const_pos_mode
    return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
}

// get attitude for slow position hold in autotune mode
void AC_AutoTune::get_poshold_attitude(float &roll_cd_out, float &pitch_cd_out, float &yaw_cd_out)
{
    roll_cd_out = pitch_cd_out = 0;

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
        start_position = inertial_nav->get_position_neu_cm();
    }

    // don't go past 10 degrees, as autotune result would deteriorate too much
    const float angle_max_cd = 1000;

    // hit the 10 degree limit at 20 meters position error
    const float dist_limit_cm = 2000;

    // we only start adjusting yaw if we are more than 5m from the
    // target position. That corresponds to a lean angle of 2.5 degrees
    const float yaw_dist_limit_cm = 500;

    Vector3f pdiff = inertial_nav->get_position_neu_cm() - start_position;
    pdiff.z = 0;
    float dist_cm = pdiff.length();
    if (dist_cm < 10) {
        // don't do anything within 10cm
        return;
    }

    /*
      very simple linear controller
     */
    float scaling = constrain_float(angle_max_cd * dist_cm / dist_limit_cm, 0, angle_max_cd);
    Vector2f angle_ne(pdiff.x, pdiff.y);
    angle_ne *= scaling / dist_cm;

    // rotate into body frame
    pitch_cd_out = angle_ne.x * ahrs_view->cos_yaw() + angle_ne.y * ahrs_view->sin_yaw();
    roll_cd_out  = angle_ne.x * ahrs_view->sin_yaw() - angle_ne.y * ahrs_view->cos_yaw();

    if (dist_cm < yaw_dist_limit_cm) {
        // no yaw adjustment
        return;
    }

    /*
      also point so that twitching occurs perpendicular to the wind,
      if we have drifted more than yaw_dist_limit_cm from the desired
      position. This ensures that autotune doesn't have to deal with
      more than 2.5 degrees of attitude on the axis it is tuning
     */
    float target_yaw_cd = degrees(atan2f(pdiff.y, pdiff.x)) * 100;
    if (axis == PITCH) {
        // for roll and yaw tuning we point along the wind, for pitch
        // we point across the wind
        target_yaw_cd += 9000;
    }
    // go to the nearest 180 degree mark, with 5 degree slop to prevent oscillation
    if (fabsf(yaw_cd_out - target_yaw_cd) > 9500) {
        target_yaw_cd += 18000;
    }

    yaw_cd_out = target_yaw_cd;
}

// get the next tune type
void AC_AutoTune::next_tune_type(TuneType &curr_tune_type, bool reset)
{
    if (reset) {
        set_tune_sequence();
        tune_seq_curr = 0;
    } else if (curr_tune_type == TUNE_COMPLETE) {
        // leave tune_type as TUNE_COMPLETE to initiate next axis or exit autotune
        return;
    } else {
        tune_seq_curr++;
    }

    curr_tune_type = tune_seq[tune_seq_curr];
}

#endif  // AC_AUTOTUNE_ENABLED
