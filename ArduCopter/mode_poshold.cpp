#include "Copter.h"

#if MODE_POSHOLD_ENABLED

/*
 * Init and run calls for PosHold flight mode
 *     PosHold tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 */

#define POSHOLD_SPEED_0                         10      // speed below which it is always safe to switch to loiter

#define POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS      6000    // Maximum duration (ms) allowed for braking before transitioning to loiter
#define POSHOLD_BRAKE_TO_LOITER_TIME_MS         1500    // Duration (ms) over which braking is blended into loiter control during BRAKE_TO_LOITER phase
#define POSHOLD_WIND_COMP_START_TIME_MS         1500    // Delay (ms) after entering loiter before wind compensation begins updating
#define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS 500     // Duration (ms) over which control is blended from autopilot to pilot input during CONTROLLER_TO_PILOT_OVERRIDE
#define POSHOLD_SMOOTH_RATE_FACTOR              0.0125f // Low-pass filter factor for smoothing pilot roll/pitch input as it returns to center
#define TC_WIND_COMP                            0.0025f // Time constant for filtering wind compensation lean angle estimates (used in low-pass filter)

// definitions that are independent of main loop rate
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE      1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f // wind compensation no more than 2/3rds of angle max to ensure pilot can always override

// poshold_init - initialise PosHold controller
bool ModePosHold::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise lean angles to current attitude
    pilot_roll_cd = 0.0f;
    pilot_pitch_cd = 0.0f;

    // compute brake_gain
    brake.gain = (15.0f * (float)g.poshold_brake_rate_degs + 95.0f) * 0.01f;

    if (copter.ap.land_complete) {
        // if landed begin in loiter mode
        roll_mode = RPMode::LOITER;
        pitch_mode = RPMode::LOITER;
    } else {
        // if not landed start in pilot override to avoid hard twitch
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
    }

    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise wind_comp each time PosHold is switched on
    init_wind_comp_estimate();

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void ModePosHold::run()
{
    const uint32_t now_ms = AP_HAL::millis();
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    const Vector3f& vel_neu_cms = pos_control->get_vel_estimate_NEU_cms();

    // enforce minimum allowed value for poshold_brake_rate_degs
    if (g.poshold_brake_rate_degs < POSHOLD_BRAKE_RATE_MIN) {
        g.poshold_brake_rate_degs.set_and_save(POSHOLD_BRAKE_RATE_MIN);
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
    float target_roll_cd = rad_to_cd(target_roll_rad);
    float target_pitch_cd = rad_to_cd(target_pitch_rad);

    // get pilot's desired yaw rate
    float target_yaw_rate_cds = rad_to_cd(get_pilot_desired_yaw_rate_rads());

    // get pilot desired climb rate (for alt-hold mode and take-off)
    float target_climb_rate_cms = get_pilot_desired_climb_rate();
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Pos Hold State Machine Determination
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate_cms);

    // state machine
    switch (poshold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;

        // initialise wind compensation estimate
        init_wind_comp_estimate();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        attitude_control->reset_yaw_target_and_rate();
        init_wind_comp_estimate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate_cms);

        // init and update loiter although pilot is controlling lean angles
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // poshold specific behaviour to calculate desired roll, pitch angles
    // convert inertial nav earth-frame velocities to body-frame
    // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
    float vel_fw_cms = vel_neu_cms.x * ahrs.cos_yaw() + vel_neu_cms.y * ahrs.sin_yaw();
    float vel_right_cms = -vel_neu_cms.x * ahrs.sin_yaw() + vel_neu_cms.y * ahrs.cos_yaw();

    // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles(wind_comp_roll_cd, wind_comp_pitch_cd);
    }

    // Roll state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final roll output to the attitude controller
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    switch (roll_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_roll_cd, target_roll_cd);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_roll_cd) && (fabsf(pilot_roll_cd) < 2 * g.poshold_brake_rate_degs)) {
                // initialise BRAKE mode
                roll_mode = RPMode::BRAKE;          // Set brake roll mode
                brake.roll_cd = 0.0f;               // initialise braking angle to zero
                brake.angle_max_roll_cd = 0.0f;     // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.start_time_roll_ms = now_ms;  // timestamp (ms) marking the start of roll-axis braking; updated during braking phase
                brake.time_updated_roll = false;    // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            roll_cd = pilot_roll_cd + wind_comp_roll_cd;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake.roll angle to counter-act velocity
            update_brake_angle_from_velocity(brake.roll_cd, vel_right_cms);

            // update braking time estimate
            if (!brake.time_updated_roll) {
                // check if brake angle is increasing
                if (fabsf(brake.roll_cd) >= brake.angle_max_roll_cd) {
                    brake.angle_max_roll_cd = fabsf(brake.roll_cd);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.start_time_roll_ms = now_ms;
                    brake.time_updated_roll = true;
                }
            } else {
                // scaling factors:
                // 1.5 times the time taken to level the aircraft in ms
                // 1000 to convert from seconds to ms
                // 0.01 to convert angle_max_roll_cd to degrees
                const uint32_t brake_timeout_roll_ms = MIN(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS, (1.5 * 1000 * 0.01) * brake.angle_max_roll_cd / g.poshold_brake_rate_degs);

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_right_cms) <= POSHOLD_SPEED_0) && (now_ms - brake.start_time_roll_ms > 500) && (brake_timeout_roll_ms > 500)) {
                    brake.start_time_roll_ms = now_ms - brake_timeout_roll_ms + 500;
                }

                if (now_ms - brake.start_time_roll_ms > brake_timeout_roll_ms) {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                    // Logic for engaging loiter is handled below the roll and pitch mode switch statements
                    roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                }
            }

            // final lean angle is braking angle + wind compensation angle
            roll_cd = brake.roll_cd + wind_comp_roll_cd;

            // check for pilot input
            if (!is_zero(target_roll_cd)) {
                // init transition to pilot override
                roll_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined roll-pitch modes and are handled below
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_roll_cd, target_roll_cd);

            // count-down loiter to pilot timer
            if (now_ms - controller_to_pilot_start_time_roll_ms > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS) {
                // when timer runs out switch to full pilot override for next iteration
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_roll_mix = (float)(now_ms - controller_to_pilot_start_time_roll_ms) / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS;

            // mix final loiter lean angle and pilot desired lean angles
            roll_cd = mix_controls(controller_to_pilot_roll_mix, controller_final_roll_cd, pilot_roll_cd + wind_comp_roll_cd);
            break;
    }

    // Pitch state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final pitch output to the attitude contpitcher
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    switch (pitch_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_pitch_cd, target_pitch_cd);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_pitch_cd) && (fabsf(pilot_pitch_cd) < 2 * g.poshold_brake_rate_degs)) {
                // initialise BRAKE mode
                pitch_mode = RPMode::BRAKE;         // set brake pitch mode
                brake.pitch_cd = 0.0f;              // initialise braking angle to zero
                brake.angle_max_pitch_cd = 0.0f;    // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.start_time_pitch_ms = now_ms; // timestamp (ms) marking the start of pitch-axis braking; updated during braking phase
                brake.time_updated_pitch = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            pitch_cd = pilot_pitch_cd + wind_comp_pitch_cd;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            update_brake_angle_from_velocity(brake.pitch_cd, -vel_fw_cms);

            // update braking time estimate
            if (!brake.time_updated_pitch) {
                // check if brake angle is increasing
                if (fabsf(brake.pitch_cd) >= brake.angle_max_pitch_cd) {
                    brake.angle_max_pitch_cd = fabsf(brake.pitch_cd);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.start_time_pitch_ms = now_ms;
                    brake.time_updated_pitch = true;
                }
            } else {
                // scaling factors:
                // 1.5 times the time taken to level the aircraft in ms
                // 1000 to convert from seconds to ms
                // 0.01 to convert angle_max_pitch_cd to degrees
                const uint32_t brake_timeout_pitch_ms = MIN(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS, (1.5 * 1000 * 0.01) * brake.angle_max_pitch_cd / g.poshold_brake_rate_degs);

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_fw_cms) <= POSHOLD_SPEED_0) && (now_ms - brake.start_time_pitch_ms > 500) && (brake_timeout_pitch_ms > 500)) {
                    brake.start_time_pitch_ms = now_ms - brake_timeout_pitch_ms + 500;
                }

                if (now_ms - brake.start_time_pitch_ms > brake_timeout_pitch_ms) {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                    pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                }
            }

            // final lean angle is braking angle + wind compensation angle
            pitch_cd = brake.pitch_cd + wind_comp_pitch_cd;

            // check for pilot input
            if (!is_zero(target_pitch_cd)) {
                // init transition to pilot override
                pitch_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined pitch-pitch modes and are handled below
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_pitch_cd, target_pitch_cd);

            // count-down loiter to pilot timer
            if (now_ms - controller_to_pilot_start_time_pitch_ms > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS) {
                // when timer runs out switch to full pilot override for next iteration
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_pitch_mix = (float)(now_ms - controller_to_pilot_start_time_pitch_ms) / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS;

            // mix final loiter lean angle and pilot desired lean angles
            pitch_cd = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch_cd, pilot_pitch_cd + wind_comp_pitch_cd);
            break;
    }

    //
    // Shared roll & pitch states (RPMode::BRAKE_TO_LOITER and RPMode::LOITER)
    //

    // switch into LOITER mode when both roll and pitch are ready
    if (roll_mode == RPMode::BRAKE_READY_TO_LOITER && pitch_mode == RPMode::BRAKE_READY_TO_LOITER) {
        roll_mode = RPMode::BRAKE_TO_LOITER;
        pitch_mode = RPMode::BRAKE_TO_LOITER;
        brake.loiter_transition_start_time_ms = now_ms;
        // init loiter controller
        loiter_nav->init_target_cm((pos_control->get_pos_estimate_NEU_cm().xy() - pos_control->get_pos_offset_NEU_cm().xy()).tofloat());
        // set delay to start of wind compensation estimate updates
        wind_comp_start_time_ms = now_ms;
    }

    // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
        pitch_mode = roll_mode;

        // handle combined roll+pitch mode
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER: {
                // reduce brake_to_loiter timer
                if (now_ms - brake.loiter_transition_start_time_ms > POSHOLD_BRAKE_TO_LOITER_TIME_MS) {
                    // progress to full loiter on next iteration
                    roll_mode = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // mix of brake and loiter controls.  0 = fully brake
                // controls, 1 = fully loiter controls
                const float brake_to_loiter_mix = (float)(now_ms - brake.loiter_transition_start_time_ms) / (float)POSHOLD_BRAKE_TO_LOITER_TIME_MS;

                // calculate brake.roll and pitch angles to counter-act velocity
                update_brake_angle_from_velocity(brake.roll_cd, vel_right_cms);
                update_brake_angle_from_velocity(brake.pitch_cd, -vel_fw_cms);

                // run loiter controller
                loiter_nav->update(false);

                // calculate final roll and pitch output by mixing loiter and brake controls
                roll_cd = mix_controls(brake_to_loiter_mix, brake.roll_cd + wind_comp_roll_cd, loiter_nav->get_roll_cd());
                pitch_cd = mix_controls(brake_to_loiter_mix, brake.pitch_cd + wind_comp_pitch_cd, loiter_nav->get_pitch_cd());

                // check for pilot input
                if (!is_zero(target_roll_cd) || !is_zero(target_pitch_cd)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll_cd)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // no need to reset brake.pitch here as wind comp has not been updated since last brake.pitch computation
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch_cd)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll_cd)) {
                            // switch roll-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset brake.roll here as wind comp has not been updated since last brake.roll computation
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                        }
                    }
                }
                break;
            }
            case RPMode::LOITER:
                // run loiter controller
                loiter_nav->update(false);

                // set roll angle based on loiter controller outputs
                roll_cd = loiter_nav->get_roll_cd();
                pitch_cd = loiter_nav->get_pitch_cd();

                // update wind compensation estimate
                update_wind_comp_estimate();

                // check for pilot input
                if (!is_zero(target_roll_cd) || !is_zero(target_pitch_cd)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll_cd)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // reset brake.pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        brake.pitch_cd = 0.0f;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch_cd)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                        if (is_zero(target_roll_cd)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll_cd = 0.0f;
                        }
                            // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                    }
                }
                break;

            default:
                // do nothing for uncombined roll and pitch modes
                break;
        }
    }

    // constrain target pitch/roll angles
    float angle_max_cd = copter.aparm.angle_max;
    roll_cd = constrain_float(roll_cd, -angle_max_cd, angle_max_cd);
    pitch_cd = constrain_float(pitch_cd, -angle_max_cd, angle_max_cd);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, target_yaw_rate_cds);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void ModePosHold::update_pilot_lean_angle_cd(float &lean_angle_filtered_cd, float &lean_angle_raw_cd)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the filtered angle to the new raw angle
    if ((lean_angle_filtered_cd > 0 && lean_angle_raw_cd < 0) || (lean_angle_filtered_cd < 0 && lean_angle_raw_cd > 0) || (fabsf(lean_angle_raw_cd) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered_cd = lean_angle_raw_cd;
    } else {
        // lean_angle_raw_cd must be pulling lean_angle_filtered_cd towards zero, smooth the decrease
        if (lean_angle_filtered_cd > 0) {
            // reduce the filtered lean angle at 1.25% per step or the brake rate (whichever is faster).
            // poshold_brake_rate_degs is in degrees/s; multiply by 100 to convert to centidegrees/s
            lean_angle_filtered_cd -= MAX(lean_angle_filtered_cd * POSHOLD_SMOOTH_RATE_FACTOR, 100.0 * g.poshold_brake_rate_degs * G_Dt);
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered_cd = MAX(lean_angle_filtered_cd, lean_angle_raw_cd);
        }else{
            lean_angle_filtered_cd += MAX(-lean_angle_filtered_cd * POSHOLD_SMOOTH_RATE_FACTOR, 100.0 * g.poshold_brake_rate_degs * G_Dt);
            lean_angle_filtered_cd = MIN(lean_angle_filtered_cd, lean_angle_raw_cd);
        }
    }
}

// mix_controls - mixes two controls based on the mix_ratio
//  mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
float ModePosHold::mix_controls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control;
}

// update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
//  brake_angle is slewed with the wpnav.poshold_brake_rate_degs_degs and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
void ModePosHold::update_brake_angle_from_velocity(float &brake_angle_cd, float velocity_cms)
{
    float lean_angle;
    float brake_delta_cd = 100.0f * g.poshold_brake_rate_degs * G_Dt;

    // calculate velocity-only based lean angle
    lean_angle = -brake.gain * velocity_cms * (1.0f + 500.0f / (fabsf(velocity_cms) + 60.0f));

    // do not let lean_angle be too far from brake_angle
    brake_angle_cd = constrain_float(lean_angle, brake_angle_cd - brake_delta_cd, brake_angle_cd + brake_delta_cd);

    // constrain final brake_angle
    brake_angle_cd = constrain_float(brake_angle_cd, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}

// initialise wind compensation estimate back to zero
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ef.zero();
    wind_comp_roll_cd = 0.0f;
    wind_comp_pitch_cd = 0.0f;
}

// update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
void ModePosHold::update_wind_comp_estimate()
{
    const uint32_t now_ms = AP_HAL::millis();
    // check wind estimate start has not been delayed
    if (now_ms - wind_comp_start_time_ms < POSHOLD_WIND_COMP_START_TIME_MS) {
        return;
    }

    // check horizontal velocity is low
    if (pos_control->get_vel_estimate_NEU_cms().xy().length() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    const Vector3f& accel_target_neu_cmss = pos_control->get_accel_target_NEU_cmss();

    // update wind compensation in earth-frame lean angles
    if (is_zero(wind_comp_ef.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.x = accel_target_neu_cmss.x;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.x = (1.0f - TC_WIND_COMP) * wind_comp_ef.x + TC_WIND_COMP * accel_target_neu_cmss.x;
    }
    if (is_zero(wind_comp_ef.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.y = accel_target_neu_cmss.y;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.y = (1.0f - TC_WIND_COMP) * wind_comp_ef.y + TC_WIND_COMP * accel_target_neu_cmss.y;
    }

    // limit acceleration
    const float accel_lim_cmss = tanf(cd_to_rad(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max)) * (GRAVITY_MSS * 100);
    const float wind_comp_ef_len = wind_comp_ef.length();
    if (!is_zero(accel_lim_cmss) && (wind_comp_ef_len > accel_lim_cmss)) {
        wind_comp_ef *= accel_lim_cmss / wind_comp_ef_len;
    }
}

// get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
void ModePosHold::get_wind_comp_lean_angles(float &roll_angle_cd, float &pitch_angle_cd)
{
    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle_cd = atanf((-wind_comp_ef.x * ahrs.sin_yaw() + wind_comp_ef.y * ahrs.cos_yaw()) / (GRAVITY_MSS * 100)) * (18000.0f / M_PI);
    pitch_angle_cd = atanf(-(wind_comp_ef.x * ahrs.cos_yaw() + wind_comp_ef.y * ahrs.sin_yaw()) / (GRAVITY_MSS * 100)) * (18000.0f / M_PI);
}

// roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::roll_controller_to_pilot_override()
{
    const uint32_t now_ms = AP_HAL::millis();
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_start_time_roll_ms = now_ms;
    // initialise pilot_roll_cd to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_roll_cd = 0.0f;
    // store final controller output for mixing with pilot input
    controller_final_roll_cd = roll_cd;
}

// pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::pitch_controller_to_pilot_override()
{
    const uint32_t now_ms = AP_HAL::millis();
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_start_time_pitch_ms = now_ms;
    // initialise pilot_pitch_cd to 0, wind_comp will be updated to compensate and update_pilot_lean_angle_cd function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_pitch_cd = 0.0f;
    // store final loiter outputs for mixing with pilot input
    controller_final_pitch_cd = pitch_cd;
}

#endif
