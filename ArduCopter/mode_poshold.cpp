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

// poshold_init - initialise PosHold controller
bool Copter::ModePosHold::init(bool ignore_checks)
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise lean angles to current attitude
    pilot_roll = 0;
    pilot_pitch = 0;

    // compute brake_gain
    brake_gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) / 100.0f;

    if (ap.land_complete) {
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
    wind_comp_ef.zero();
    wind_comp_roll = 0;
    wind_comp_pitch = 0;
    wind_comp_timer = 0;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Copter::ModePosHold::run()
{
    float takeoff_climb_rate = 0.0f;
    float brake_to_loiter_mix;          // mix of brake and loiter controls.  0 = fully brake controls, 1 = fully loiter controls
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    const Vector3f& vel = inertial_nav.get_velocity();

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);
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
    if (ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Pos Hold State Machine Determination
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate);

    // state machine
    switch (poshold_state) {

    case AltHold_MotorStopped:

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        loiter_nav->init_target();
        loiter_nav->update();

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // init and update loiter although pilot is controlling lean angles
        loiter_nav->init_target();
        loiter_nav->update();

        // set position controller targets
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Landed_Ground_Idle:

        loiter_nav->init_target();
        loiter_nav->update();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:

        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Flying:

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // adjust climb rate using rangefinder
        if (copter.rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = copter.get_surface_tracking_climb_rate(target_climb_rate);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
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
                brake_roll = 0;                  // initialise braking angle to zero
                brake_angle_max_roll = 0;        // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake_timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                braking_time_updated_roll = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            roll = pilot_roll + wind_comp_roll;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_roll angle to counter-act velocity
            update_brake_angle_from_velocity(brake_roll, vel_right);

            // update braking time estimate
            if (!braking_time_updated_roll) {
                // check if brake angle is increasing
                if (abs(brake_roll) >= brake_angle_max_roll) {
                    brake_angle_max_roll = abs(brake_roll);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake_timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(abs(brake_roll))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                    braking_time_updated_roll = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (brake_timeout_roll > 50*LOOP_RATE_FACTOR)) {
                brake_timeout_roll = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            if (brake_timeout_roll > 0) {
                brake_timeout_roll--;
            } else {
                // indicate that we are ready to move to Loiter.
                // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                //  logic for engaging loiter is handled below the roll and pitch mode switch statements
                roll_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            roll = brake_roll + wind_comp_roll;

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
                brake_pitch = 0;                 // initialise braking angle to zero
                brake_angle_max_pitch = 0;       // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake_timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                braking_time_updated_pitch = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            pitch = pilot_pitch + wind_comp_pitch;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            update_brake_angle_from_velocity(brake_pitch, -vel_fw);

            // update braking time estimate
            if (!braking_time_updated_pitch) {
                // check if brake angle is increasing
                if (abs(brake_pitch) >= brake_angle_max_pitch) {
                    brake_angle_max_pitch = abs(brake_pitch);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake_timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(abs(brake_pitch))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                    braking_time_updated_pitch = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (brake_timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                brake_timeout_pitch = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            if (brake_timeout_pitch > 0) {
                brake_timeout_pitch--;
            } else {
                // indicate that we are ready to move to Loiter.
                // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            pitch = brake_pitch + wind_comp_pitch;

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
        brake_to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
        // init loiter controller
        loiter_nav->init_target(inertial_nav.get_position());
        // set delay to start of wind compensation estimate updates
        wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
    }

    // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
        pitch_mode = roll_mode;

        // handle combined roll+pitch mode
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER:
                // reduce brake_to_loiter timer
                if (brake_to_loiter_timer > 0) {
                    brake_to_loiter_timer--;
                } else {
                    // progress to full loiter on next iteration
                    roll_mode = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // calculate percentage mix of loiter and brake control
                brake_to_loiter_mix = (float)brake_to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                // calculate brake_roll and pitch angles to counter-act velocity
                update_brake_angle_from_velocity(brake_roll, vel_right);
                update_brake_angle_from_velocity(brake_pitch, -vel_fw);

                // run loiter controller
                loiter_nav->update();

                // calculate final roll and pitch output by mixing loiter and brake controls
                roll = mix_controls(brake_to_loiter_mix, brake_roll + wind_comp_roll, loiter_nav->get_roll());
                pitch = mix_controls(brake_to_loiter_mix, brake_pitch + wind_comp_pitch, loiter_nav->get_pitch());

                // check for pilot input
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // no need to reset brake_pitch here as wind comp has not been updated since last brake_pitch computation
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll)) {
                            // switch roll-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset brake_roll here as wind comp has not been updated since last brake_roll computation
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                        }
                    }
                }
                break;

            case RPMode::LOITER:
                // run loiter controller
                loiter_nav->update();

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
                        // reset brake_pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        brake_pitch = 0;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        // if roll not overriden switch roll-mode to brake (but be ready to go back to loiter any time)
                        if (is_zero(target_roll)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake_roll = 0;
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
    roll = constrain_int16(roll, -angle_max, angle_max);
    pitch = constrain_int16(pitch, -angle_max, angle_max);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void Copter::ModePosHold::update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw)
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
int16_t Copter::ModePosHold::mix_controls(float mix_ratio, int16_t first_control, int16_t second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return (int16_t)((mix_ratio * first_control) + ((1.0f-mix_ratio)*second_control));
}

// update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
void Copter::ModePosHold::update_brake_angle_from_velocity(int16_t &brake_angle, float velocity)
{
    float lean_angle;
    int16_t brake_rate = g.poshold_brake_rate;

    brake_rate /= 4;
    if (brake_rate <= 0) {
        brake_rate = 1;
    }

    // calculate velocity-only based lean angle
    if (velocity >= 0) {
        lean_angle = -brake_gain * velocity * (1.0f+500.0f/(velocity+60.0f));
    } else {
        lean_angle = -brake_gain * velocity * (1.0f+500.0f/(-velocity+60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    brake_angle = constrain_int16((int16_t)lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // constrain final brake_angle
    brake_angle = constrain_int16(brake_angle, -g.poshold_brake_angle_max, g.poshold_brake_angle_max);
}

// update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
void Copter::ModePosHold::update_wind_comp_estimate()
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
    //  To-Do: clean this up by using accessor in loiter controller (or move entire PosHold controller to a library shared with loiter)
    const Vector3f& accel_target = pos_control->get_accel_target();

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
}

// get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
void Copter::ModePosHold::get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle)
{
    // reduce rate to 10hz
    wind_comp_timer++;
    if (wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle = atanf((-wind_comp_ef.x*ahrs.sin_yaw() + wind_comp_ef.y*ahrs.cos_yaw())/981)*(18000/M_PI);
    pitch_angle = atanf(-(wind_comp_ef.x*ahrs.cos_yaw() + wind_comp_ef.y*ahrs.sin_yaw())/981)*(18000/M_PI);
}

// roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void Copter::ModePosHold::roll_controller_to_pilot_override()
{
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_roll to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_roll = 0;
    // store final controller output for mixing with pilot input
    controller_final_roll = roll;
}

// pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void Copter::ModePosHold::pitch_controller_to_pilot_override()
{
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_pitch = 0;
    // store final loiter outputs for mixing with pilot input
    controller_final_pitch = pitch;
}

#endif
