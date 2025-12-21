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
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD  radians(18.0f)  // max angle required (in radians) after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX_MS 0.10            // wind compensation estimates will only run when velocity is at or below this speed in cm/s
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f         // wind compensation no more than 2/3rds of angle max to ensure pilot can always override

// poshold_init - initialise PosHold controller
bool ModePosHold::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // initialise lean angles to current attitude
    pilot_roll_rad = 0.0f;
    pilot_pitch_rad = 0.0f;

    // compute brake_gain in rad/(m/s);
    brake.gain = radians((15.0f * (float)g.poshold_brake_rate_degs + 95.0f) * 0.01f);

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
    float controller_to_pilot_roll_mix;     // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    const Vector3f& vel_ned_ms = pos_control->get_vel_estimate_NED_ms();

    // enforce minimum allowed value for poshold_brake_rate_degs
    if (g.poshold_brake_rate_degs < POSHOLD_BRAKE_RATE_MIN) {
        g.poshold_brake_rate_degs.set_and_save(POSHOLD_BRAKE_RATE_MIN);
    }

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    loiter_nav->clear_pilot_desired_acceleration();

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // pilot desired yaw rate already rad/s
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // pilot desired climb rate (m/s)
    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // state machine
    AltHoldModeState poshold_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    switch (poshold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
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
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01f, 0.0f, 10.0f));
        }

        // avoidance-adjusted climb
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);

        // init and update loiter although pilot is controlling lean angles
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // avoidance-adjusted climb
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // convert earth-frame velocities to body-frame (m/s)
    float vel_fw_ms    = vel_ned_ms.x * ahrs.cos_yaw() + vel_ned_ms.y * ahrs.sin_yaw();
    float vel_right_ms = -vel_ned_ms.x * ahrs.sin_yaw() + vel_ned_ms.y * ahrs.cos_yaw();

    // If not in LOITER, retrieve latest wind compensation lean angles (radians) related to current yaw
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles_rad(wind_comp_roll_rad, wind_comp_pitch_rad);
    }

    // Roll state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final roll output to the attitude controller
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    // Roll state machine
    switch (roll_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_rad(pilot_roll_rad, target_roll_rad);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_roll_rad) &&
                (fabsf(pilot_roll_rad) < radians(2.0f * (float)g.poshold_brake_rate_degs))) {
                // initialise BRAKE mode
                roll_mode = RPMode::BRAKE;              // Set brake roll mode
                brake.roll_rad = 0.0f;                  // initialise braking angle to zero
                brake.angle_max_roll_rad = 0.0f;        // reset so we can detect when vehicle begins to flatten during braking
                brake.start_time_roll_ms = now_ms;      // timestamp (ms) marking start of roll-axis braking; updated during braking phase
                brake.time_updated_roll = false;        // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            roll_rad = pilot_roll_rad + wind_comp_roll_rad;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake.roll angle to counter-act velocity
            update_brake_angle_from_velocity(brake.roll_rad, vel_right_ms);

            // update braking time estimate
            if (!brake.time_updated_roll) {
                // check if brake angle is increasing
                if (fabsf(brake.roll_rad) >= brake.angle_max_roll_rad) {
                    brake.angle_max_roll_rad = fabsf(brake.roll_rad);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.start_time_roll_ms = now_ms;
                    brake.time_updated_roll = true;
                }
            } else {
                // scaling factors:
                // 1.5 × (time to level, s) × 1000 -> ms
                // time to level ≈ angle_max / brake_rate
                const uint32_t brake_timeout_roll_ms = MIN(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS, (1.5f * 1000.0f * (brake.angle_max_roll_rad / radians(g.poshold_brake_rate_degs))));

                // if velocity is very low reduce braking time to 0.5 s
                if ((fabsf(vel_right_ms) <= POSHOLD_SPEED_0) && (now_ms - brake.start_time_roll_ms > 500) && (brake_timeout_roll_ms > 500)) {
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
            roll_rad = brake.roll_rad + wind_comp_roll_rad;

            // check for pilot input
            if (!is_zero(target_roll_rad)) {
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
            update_pilot_lean_angle_rad(pilot_roll_rad, target_roll_rad);

            // count-down loiter to pilot timer
            if (now_ms - controller_to_pilot_start_time_roll_ms > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS) {
                // when timer runs out switch to full pilot override for next iteration
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_roll_mix = (float)(now_ms - controller_to_pilot_start_time_roll_ms) / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS;

            // mix final loiter lean angle and pilot desired lean angles
            roll_rad = mix_controls(controller_to_pilot_roll_mix, controller_final_roll_rad, pilot_roll_rad + wind_comp_roll_rad);
            break;
    }

    // Pitch state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final pitch output to the attitude contpitcher
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    // Pitch state machine
    switch (pitch_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_rad(pilot_pitch_rad, target_pitch_rad);

            // switch to BRAKE next iteration if no pilot input
            if (is_zero(target_pitch_rad) && (fabsf(pilot_pitch_rad) < radians(2 * g.poshold_brake_rate_degs))) {
                // initialise BRAKE mode
                pitch_mode = RPMode::BRAKE;         // set brake pitch mode
                brake.pitch_rad = 0.0f;             // initialise braking angle to zero
                brake.angle_max_pitch_rad = 0.0f;   // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.start_time_pitch_ms = now_ms; // timestamp (ms) marking the start of pitch-axis 
                brake.time_updated_pitch = false;   // flag the braking time can be re-estimated
            }

            // final lean angle = pilot input + wind compensation
            pitch_rad = pilot_pitch_rad + wind_comp_pitch_rad;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            update_brake_angle_from_velocity(brake.pitch_rad, -vel_fw_ms);

            // update braking time estimate
            if (!brake.time_updated_pitch) {
                // check if brake angle is increasing
                if (fabsf(brake.pitch_rad) >= brake.angle_max_pitch_rad) {
                    brake.angle_max_pitch_rad = fabsf(brake.pitch_rad);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.start_time_pitch_ms = now_ms;
                    brake.time_updated_pitch = true;
                }
            } else {
                // timeout ≈ 1.5 * (angle / rate) in ms
                const uint32_t brake_timeout_pitch_ms = MIN(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS, (1.5 * 1000.0 * (brake.angle_max_pitch_rad / radians(g.poshold_brake_rate_degs))));

                // if velocity is very low reduce braking time to 0.5 seconds
                if ((fabsf(vel_fw_ms) <= POSHOLD_SPEED_0) && (now_ms - brake.start_time_pitch_ms > 500) && (brake_timeout_pitch_ms > 500)) {
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
            pitch_rad = brake.pitch_rad + wind_comp_pitch_rad;

            // check for pilot input
            if (!is_zero(target_pitch_rad)) {
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
            update_pilot_lean_angle_rad(pilot_pitch_rad, target_pitch_rad);

            // count-down loiter to pilot timer
            if (now_ms - controller_to_pilot_start_time_pitch_ms > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS) {
                // when timer runs out switch to full pilot override for next iteration
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_pitch_mix = (float)(now_ms - controller_to_pilot_start_time_pitch_ms) / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS;

            // mix final loiter lean angle and pilot desired lean angles
            pitch_rad = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch_rad, pilot_pitch_rad + wind_comp_pitch_rad);
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
        loiter_nav->init_target_m((pos_control->get_pos_estimate_NED_m().xy() - pos_control->get_pos_offset_NED_m().xy()));
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

                // mix of brake and loiter controls
                const float brake_to_loiter_mix = (float)(now_ms - brake.loiter_transition_start_time_ms) / (float)POSHOLD_BRAKE_TO_LOITER_TIME_MS;

                // calculate brake roll/pitch angles to counter velocity
                update_brake_angle_from_velocity(brake.roll_rad,  vel_right_ms);
                update_brake_angle_from_velocity(brake.pitch_rad, -vel_fw_ms);

                // run loiter controller
                loiter_nav->update(false);

                // calculate final roll and pitch output by mixing loiter and brake controls
                roll_rad  = mix_controls(brake_to_loiter_mix, brake.roll_rad + wind_comp_roll_rad, loiter_nav->get_roll_rad());
                pitch_rad = mix_controls(brake_to_loiter_mix, brake.pitch_rad + wind_comp_pitch_rad, loiter_nav->get_pitch_rad());

                // check for pilot input
                if (!is_zero(target_roll_rad) || !is_zero(target_pitch_rad)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll_rad)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // no need to reset brake.pitch here as wind comp has not been updated since last brake.pitch computation
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch_rad)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll_rad)) {
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
                roll_rad  = loiter_nav->get_roll_rad();
                pitch_rad = loiter_nav->get_pitch_rad();

                // update wind compensation estimate
                update_wind_comp_estimate();

                // check for pilot input
                if (!is_zero(target_roll_rad) || !is_zero(target_pitch_rad)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll_rad)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // reset brake.pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        brake.pitch_rad = 0.0f;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch_rad)) {
                        // init transition to pilot override on pitch
                        pitch_controller_to_pilot_override();
                        // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                        if (is_zero(target_roll_rad)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll_rad = 0.0f;
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
    const float angle_max_rad = cd_to_rad(copter.aparm.angle_max);
    roll_rad  = constrain_float(roll_rad,  -angle_max_rad, angle_max_rad);
    pitch_rad = constrain_float(pitch_rad, -angle_max_rad, angle_max_rad);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(roll_rad, pitch_rad, target_yaw_rate_rads);

    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void ModePosHold::update_pilot_lean_angle_rad(float &lean_angle_filtered_rad, float &lean_angle_raw_rad)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the filtered angle to the new raw angle
    if ((lean_angle_filtered_rad > 0.0 && lean_angle_raw_rad < 0.0) || (lean_angle_filtered_rad < 0.0 && lean_angle_raw_rad > 0.0) || (fabsf(lean_angle_raw_rad) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD)) {
        lean_angle_filtered_rad = lean_angle_raw_rad;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        const float brake_rate_step_rad = radians((float)g.poshold_brake_rate_degs) * G_Dt;
        if (lean_angle_filtered_rad > 0.0) {
            // reduce the filtered lean angle at 1.25% per step or the brake rate (whichever is faster).
            lean_angle_filtered_rad -= MAX(lean_angle_filtered_rad * POSHOLD_SMOOTH_RATE_FACTOR, brake_rate_step_rad);
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered_rad = MAX(lean_angle_filtered_rad, lean_angle_raw_rad);
        } else {
            lean_angle_filtered_rad += MAX(-lean_angle_filtered_rad * POSHOLD_SMOOTH_RATE_FACTOR, brake_rate_step_rad);
            lean_angle_filtered_rad = MIN(lean_angle_filtered_rad, lean_angle_raw_rad);
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
void ModePosHold::update_brake_angle_from_velocity(float &brake_angle_rad, float velocity_ms)
{
    // slew limit per step (rad)
    const float brake_delta_rad = radians((float)g.poshold_brake_rate_degs) * G_Dt;

    // velocity-shaped lean angle:
    float lean_angle_rad = -brake.gain * velocity_ms * (1.0f + 5.0f / (fabsf(velocity_ms) + 0.60f));

    // do not let lean_angle be too far from brake_angle
    brake_angle_rad = constrain_float(lean_angle_rad, brake_angle_rad - brake_delta_rad, brake_angle_rad + brake_delta_rad);

    // constrain final brake_angle
    const float brake_angle_max_rad = cd_to_rad(g.poshold_brake_angle_max);
    brake_angle_rad = constrain_float(brake_angle_rad, -brake_angle_max_rad, brake_angle_max_rad);
}

// initialise wind compensation estimate back to zero
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ne_mss.zero();
    wind_comp_roll_rad = 0.0f;
    wind_comp_pitch_rad = 0.0f;
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
    if (pos_control->get_vel_estimate_NED_ms().xy().length() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX_MS) {
        return;
    }

    // get position controller accel target
    const Vector3f& accel_target_ned_mss = pos_control->get_accel_target_NED_mss();

    // update wind compensation in earth-frame lean angles
    if (is_zero(wind_comp_ne_mss.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ne_mss.x = accel_target_ned_mss.x;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ne_mss.x = (1.0f - TC_WIND_COMP) * wind_comp_ne_mss.x + TC_WIND_COMP * accel_target_ned_mss.x;
    }
    if (is_zero(wind_comp_ne_mss.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ne_mss.y = accel_target_ned_mss.y;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ne_mss.y = (1.0f - TC_WIND_COMP) * wind_comp_ne_mss.y + TC_WIND_COMP * accel_target_ned_mss.y;
    }

    // limit acceleration
    const float accel_lim_mss = tanf(cd_to_rad(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max)) * GRAVITY_MSS;
    const float wind_comp_ef_len = wind_comp_ne_mss.length();
    if (!is_zero(accel_lim_mss) && (wind_comp_ef_len > accel_lim_mss)) {
        wind_comp_ne_mss *= accel_lim_mss / wind_comp_ef_len;
    }
}

// get_wind_comp_lean_angles_rad - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
void ModePosHold::get_wind_comp_lean_angles_rad(float &roll_angle_rad, float &pitch_angle_rad)
{
    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle_rad = atanf((-wind_comp_ne_mss.x * ahrs.sin_yaw() + wind_comp_ne_mss.y * ahrs.cos_yaw()) / GRAVITY_MSS);
    pitch_angle_rad = atanf(-(wind_comp_ne_mss.x * ahrs.cos_yaw() + wind_comp_ne_mss.y * ahrs.sin_yaw()) / GRAVITY_MSS);
}

// roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::roll_controller_to_pilot_override()
{
    const uint32_t now_ms = AP_HAL::millis();
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_start_time_roll_ms = now_ms;
    // initialise pilot_roll_rad to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_roll_rad = 0.0f;
    // store final controller output for mixing with pilot input
    controller_final_roll_rad = roll_rad;
}

// pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::pitch_controller_to_pilot_override()
{
    const uint32_t now_ms = AP_HAL::millis();
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_start_time_pitch_ms = now_ms;
    // initialise pilot_pitch_rad to 0, wind_comp will be updated to compensate and update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_pitch_rad = 0.0f;
    // store final loiter outputs for mixing with pilot input
    controller_final_pitch_rad = pitch_rad;
}

#endif
