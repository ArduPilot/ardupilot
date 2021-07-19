#include "Copter.h"

#if MODE_POSHOLD_ENABLED == ENABLED

/*
 * Init and run calls for PosHold flight mode
 *     PosHold tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 */

#define POSHOLD_SPEED_0                         10      // speed below which it is always safe to switch to loiter

// 400hz loop update rate
#define POSHOLD_BRAKE_TIME_ESTIMATE_MAX         (600*4) // max number of cycles the brake will be applied before we switch to loiter
#define POSHOLD_BRAKE_TO_LOITER_TIMER           (150*4) // Number of cycles to transition from brake mode to loiter mode.  Must be lower than POSHOLD_LOITER_STAB_TIMER
#define POSHOLD_WIND_COMP_START_TIMER           (150*4) // Number of cycles to start wind compensation update after loiter is engaged
#define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER   (50*4)  // Set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
#define POSHOLD_SMOOTH_RATE_FACTOR              0.0125f // filter applied to pilot's roll/pitch input as it returns to center.  A lower number will cause the roll/pitch to return to zero more slowly if the brake_rate is also low.
#define POSHOLD_WIND_COMP_TIMER_10HZ            40      // counter value used to reduce wind compensation to 10hz
#define LOOP_RATE_FACTOR                        4       // used to adapt PosHold params to loop_rate
#define TC_WIND_COMP                            0.0025f // Time constant for poshold_update_wind_comp_estimate()

// definitions that are independent of main loop rate
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE      1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f // wind compensation no more than 2/3rds of angle max to ensure pilot can always override

// poshold_init - initialise PosHold controller
bool ModePosHold::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise lean angles to current attitude
    pilot_roll = 0.0f;
    pilot_pitch = 0.0f;

    // compute brake_gain
    brake.gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) / 100.0f;

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
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    const Vector3f& vel = inertial_nav.get_velocity();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate (for alt-hold mode and take-off)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Pos Hold State Machine Determination
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate);

    // state machine
    switch (poshold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        loiter_nav->update(false);

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;

        // initialise wind compensation estimate
        init_wind_comp_estimate();
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // init and update loiter although pilot is controlling lean angles
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        loiter_nav->update(false);

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Landed_Ground_Idle:
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        loiter_nav->update(false);
        attitude_control->reset_yaw_target_and_rate();
        init_wind_comp_estimate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // adjust climb rate using rangefinder
        if (copter.rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate, false);
        break;
    }

    // poshold specific behaviour to calculate desired roll, pitch angles
    // convert inertial nav earth-frame velocities to body-frame
    // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
    float vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
    float vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

    // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles(wind_comp_roll, wind_comp_pitch);
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
            update_pilot_lean_angle(pilot_roll, target_roll);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_roll) && (fabsf(pilot_roll) < 2 * g.poshold_brake_rate)) {
                // initialise BRAKE mode
                roll_mode = RPMode::BRAKE;        // Set brake roll mode
                brake.roll = 0.0f;                  // initialise braking angle to zero
                brake.angle_max_roll = 0.0f;        // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                brake.time_updated_roll = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            roll = pilot_roll + wind_comp_roll;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake.roll angle to counter-act velocity
            update_brake_angle_from_velocity(brake.roll, vel_right);

            // update braking time estimate
            if (!brake.time_updated_roll) {
                // check if brake angle is increasing
                if (fabsf(brake.roll) >= brake.angle_max_roll) {
                    brake.angle_max_roll = fabsf(brake.roll);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(brake.roll))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                    brake.time_updated_roll = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (brake.timeout_roll > 50*LOOP_RATE_FACTOR)) {
                brake.timeout_roll = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            if (brake.timeout_roll > 0) {
                brake.timeout_roll--;
            } else {
                // indicate that we are ready to move to Loiter.
                // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                //  logic for engaging loiter is handled below the roll and pitch mode switch statements
                roll_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            roll = brake.roll + wind_comp_roll;

            // check for pilot input
            if (!is_zero(target_roll)) {
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
            update_pilot_lean_angle(pilot_roll, target_roll);

            // count-down loiter to pilot timer
            if (controller_to_pilot_timer_roll > 0) {
                controller_to_pilot_timer_roll--;
            } else {
                // when timer runs out switch to full pilot override for next iteration
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_roll_mix = (float)controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // mix final loiter lean angle and pilot desired lean angles
            roll = mix_controls(controller_to_pilot_roll_mix, controller_final_roll, pilot_roll + wind_comp_roll);
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
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_pitch) && (fabsf(pilot_pitch) < 2 * g.poshold_brake_rate)) {
                // initialise BRAKE mode
                pitch_mode = RPMode::BRAKE;       // set brake pitch mode
                brake.pitch = 0.0f;                 // initialise braking angle to zero
                brake.angle_max_pitch = 0.0f;       // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                brake.time_updated_pitch = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            pitch = pilot_pitch + wind_comp_pitch;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            update_brake_angle_from_velocity(brake.pitch, -vel_fw);

            // update braking time estimate
            if (!brake.time_updated_pitch) {
                // check if brake angle is increasing
                if (fabsf(brake.pitch) >= brake.angle_max_pitch) {
                    brake.angle_max_pitch = fabsf(brake.pitch);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(brake.pitch))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                    brake.time_updated_pitch = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (brake.timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                brake.timeout_pitch = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            if (brake.timeout_pitch > 0) {
                brake.timeout_pitch--;
            } else {
                // indicate that we are ready to move to Loiter.
                // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            pitch = brake.pitch + wind_comp_pitch;

            // check for pilot input
            if (!is_zero(target_pitch)) {
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
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // count-down loiter to pilot timer
            if (controller_to_pilot_timer_pitch > 0) {
                controller_to_pilot_timer_pitch--;
            } else {
                // when timer runs out switch to full pilot override for next iteration
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_pitch_mix = (float)controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // mix final loiter lean angle and pilot desired lean angles
            pitch = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch, pilot_pitch + wind_comp_pitch);
            break;
    }

    //
    // Shared roll & pitch states (RPMode::BRAKE_TO_LOITER and RPMode::LOITER)
    //

    // switch into LOITER mode when both roll and pitch are ready
    if (roll_mode == RPMode::BRAKE_READY_TO_LOITER && pitch_mode == RPMode::BRAKE_READY_TO_LOITER) {
        roll_mode = RPMode::BRAKE_TO_LOITER;
        pitch_mode = RPMode::BRAKE_TO_LOITER;
        brake.to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
        // init loiter controller
        loiter_nav->init_target(inertial_nav.get_position().xy());
        // set delay to start of wind compensation estimate updates
        wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
    }

    // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
        pitch_mode = roll_mode;

        // handle combined roll+pitch mode
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER: {
                // reduce brake_to_loiter timer
                if (brake.to_loiter_timer > 0) {
                    brake.to_loiter_timer--;
                } else {
                    // progress to full loiter on next iteration
                    roll_mode = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // mix of brake and loiter controls.  0 = fully brake
                // controls, 1 = fully loiter controls
                const float brake_to_loiter_mix = (float)brake.to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                // calculate brake.roll and pitch angles to counter-act velocity
                update_brake_angle_from_velocity(brake.roll, vel_right);
                update_brake_angle_from_velocity(brake.pitch, -vel_fw);

                // run loiter controller
                loiter_nav->update(false);

                // calculate final roll and pitch output by mixing loiter and brake controls
                roll = mix_controls(brake_to_loiter_mix, brake.roll + wind_comp_roll, loiter_nav->get_roll());
                pitch = mix_controls(brake_to_loiter_mix, brake.pitch + wind_comp_pitch, loiter_nav->get_pitch());

                // check for pilot input
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // no need to reset brake.pitch here as wind comp has not been updated since last brake.pitch computation
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll)) {
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
                roll = loiter_nav->get_roll();
                pitch = loiter_nav->get_pitch();

                // update wind compensation estimate
                update_wind_comp_estimate();

                // check for pilot input
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // reset brake.pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        brake.pitch = 0.0f;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        // if roll not overriden switch roll-mode to brake (but be ready to go back to loiter any time)
                        if (is_zero(target_roll)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll = 0.0f;
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
    float angle_max = copter.aparm.angle_max;
    roll = constrain_float(roll, -angle_max, angle_max);
    pitch = constrain_float(pitch, -angle_max, angle_max);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void ModePosHold::update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    if ((lean_angle_filtered > 0 && lean_angle_raw < 0) || (lean_angle_filtered < 0 && lean_angle_raw > 0) || (fabsf(lean_angle_raw) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        if (lean_angle_filtered > 0) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            lean_angle_filtered -= MAX((float)lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1, g.poshold_brake_rate/LOOP_RATE_FACTOR));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered = MAX(lean_angle_filtered, lean_angle_raw);
        }else{
            lean_angle_filtered += MAX(-(float)lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1, g.poshold_brake_rate/LOOP_RATE_FACTOR));
            lean_angle_filtered = MIN(lean_angle_filtered, lean_angle_raw);
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
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
void ModePosHold::update_brake_angle_from_velocity(float &brake_angle, float velocity)
{
    float lean_angle;
    float brake_rate = g.poshold_brake_rate;

    brake_rate /= 4.0f;
    if (brake_rate <= 1.0f) {
        brake_rate = 1.0f;
    }

    // calculate velocity-only based lean angle
    if (velocity >= 0) {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (velocity + 60.0f));
    } else {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (-velocity + 60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    brake_angle = constrain_float(lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // constrain final brake_angle
    brake_angle = constrain_float(brake_angle, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}

// initialise wind compensation estimate back to zero
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ef.zero();
    wind_comp_timer = 0;
    wind_comp_roll = 0.0f;
    wind_comp_pitch = 0.0f;
}

// update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
void ModePosHold::update_wind_comp_estimate()
{
    // check wind estimate start has not been delayed
    if (wind_comp_start_timer > 0) {
        wind_comp_start_timer--;
        return;
    }

    // check horizontal velocity is low
    if (inertial_nav.get_speed_xy() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    const Vector3f& accel_target = pos_control->get_accel_target_cmss();

    // update wind compensation in earth-frame lean angles
    if (is_zero(wind_comp_ef.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.x = accel_target.x;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.x = (1.0f-TC_WIND_COMP)*wind_comp_ef.x + TC_WIND_COMP*accel_target.x;
    }
    if (is_zero(wind_comp_ef.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.y = accel_target.y;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.y = (1.0f-TC_WIND_COMP)*wind_comp_ef.y + TC_WIND_COMP*accel_target.y;
    }

    // limit acceleration
    const float accel_lim_cmss = tanf(radians(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max * 0.01f)) * 981.0f;
    const float wind_comp_ef_len = wind_comp_ef.length();
    if (!is_zero(accel_lim_cmss) && (wind_comp_ef_len > accel_lim_cmss)) {
        wind_comp_ef *= accel_lim_cmss / wind_comp_ef_len;
    }
}

// get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
void ModePosHold::get_wind_comp_lean_angles(float &roll_angle, float &pitch_angle)
{
    // reduce rate to 10hz
    wind_comp_timer++;
    if (wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle = atanf((-wind_comp_ef.x*ahrs.sin_yaw() + wind_comp_ef.y*ahrs.cos_yaw())/(GRAVITY_MSS*100))*(18000.0f/M_PI);
    pitch_angle = atanf(-(wind_comp_ef.x*ahrs.cos_yaw() + wind_comp_ef.y*ahrs.sin_yaw())/(GRAVITY_MSS*100))*(18000.0f/M_PI);
}

// roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::roll_controller_to_pilot_override()
{
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_roll to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_roll = 0.0f;
    // store final controller output for mixing with pilot input
    controller_final_roll = roll;
}

// pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::pitch_controller_to_pilot_override()
{
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_pitch = 0.0f;
    // store final loiter outputs for mixing with pilot input
    controller_final_pitch = pitch;
}

#endif
