/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if POSHOLD_ENABLED == ENABLED

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
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD      (DEG_TO_RAD*18.0f)    // max angle required (in radians) after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s

// mission state enumeration
enum poshold_rp_mode {
    POSHOLD_PILOT_OVERRIDE=0,            // pilot is controlling this axis (i.e. roll or pitch)
    POSHOLD_BRAKE,                       // this axis is braking towards zero
    POSHOLD_BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
    POSHOLD_BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
    POSHOLD_LOITER,                      // both vehicle axis are holding position
    POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
};

static struct {
    poshold_rp_mode roll_mode            : 3;    // roll mode: pilot override, brake or loiter
    poshold_rp_mode pitch_mode           : 3;    // pitch mode: pilot override, brake or loiter
    uint8_t braking_time_updated_roll   : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
    uint8_t braking_time_updated_pitch  : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
    uint8_t loiter_reset_I              : 1;    // true the very first time PosHold enters loiter, thereafter we trust the i terms loiter has

    // pilot input related variables
    float pilot_roll_rad;                         // pilot requested roll angle (filtered to slow returns to zero)
    float pilot_pitch_rad;                        // pilot requested roll angle (filtered to slow returns to zero)

    // braking related variables
    float brake_gain_rad;                           // gain used during conversion of vehicle's velocity to lean angle during braking (calculated from brake_rate)
    float brake_roll_rad;                         // target roll angle during braking periods
    float brake_pitch_rad;                        // target pitch angle during braking periods
    int16_t brake_timeout_roll;                 // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    int16_t brake_timeout_pitch;                // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    float brake_angle_max_roll_rad;               // maximum lean angle achieved during braking.  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    float brake_angle_max_pitch_rad;              // maximum lean angle achieved during braking  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    int16_t brake_to_loiter_timer;              // cycles to mix brake and loiter controls in POSHOLD_BRAKE_TO_LOITER

    // loiter related variables
    int16_t controller_to_pilot_timer_roll;     // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    int16_t controller_to_pilot_timer_pitch;    // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    float controller_final_roll_rad;              // final roll angle from controller as we exit brake or loiter mode (used for mixing with pilot input)
    float controller_final_pitch_rad;             // final pitch angle from controller as we exit brake or loiter mode (used for mixing with pilot input)

    // wind compensation related variables
    Vector2f wind_comp_ef;                      // wind compensation in earth frame, filtered lean angles from position controller
    float wind_comp_roll_rad;                     // roll angle to compensate for wind
    float wind_comp_pitch_rad;                    // pitch angle to compensate for wind
    uint16_t wind_comp_start_timer;             // counter to delay start of wind compensation for a short time after loiter is engaged
    int8_t  wind_comp_timer;                    // counter to reduce wind comp roll/pitch lean angle calcs to 10hz

    // final output
    float roll_rad;   // final roll angle sent to attitude controller
    float pitch_rad;  // final pitch angle sent to attitude controller
} poshold;

// poshold_init - initialise PosHold controller
bool Copter::poshold_init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok() && !ignore_checks) {
        return false;
    }
    
    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // initialise lean angles to current attitude
    poshold.pilot_roll_rad = 0;
    poshold.pilot_pitch_rad = 0;

    // compute brake_gain
    poshold.brake_gain_rad = (15.0f * radians((g.poshold_brake_rate*0.01f) + 0.95f)) / 100.0f;

    if (ap.land_complete) {
        // if landed begin in loiter mode
        poshold.roll_mode = POSHOLD_LOITER;
        poshold.pitch_mode = POSHOLD_LOITER;
        // set target to current position
        // only init here as we can switch to PosHold in flight with a velocity <> 0 that will be used as _last_vel in PosControl and never updated again as we inhibit Reset_I
        wp_nav.init_loiter_target();
    }else{
        // if not landed start in pilot override to avoid hard twitch
        poshold.roll_mode = POSHOLD_PILOT_OVERRIDE;
        poshold.pitch_mode = POSHOLD_PILOT_OVERRIDE;
    }

    // loiter's I terms should be reset the first time only
    poshold.loiter_reset_I = true;

    // initialise wind_comp each time PosHold is switched on
    poshold.wind_comp_ef.zero();
    poshold.wind_comp_roll_rad = 0.0f;
    poshold.wind_comp_pitch_rad = 0.0f;
    poshold.wind_comp_timer = 0;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Copter::poshold_run()
{
    float target_roll_rad, target_pitch_rad;  // pilot's roll and pitch angle inputs
    float target_yaw_rate_rads = 0;          // pilot desired yaw rate in centi-degrees/sec
    float target_climb_rate = 0;      // pilot desired climb rate in centimeters/sec
    float takeoff_climb_rate = 0.0f;    // takeoff induced climb rate
    float brake_to_loiter_mix;          // mix of brake and loiter controls.  0 = fully brake controls, 1 = fully loiter controls
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float vel_fw, vel_right;            // vehicle's current velocity in body-frame forward and right directions
    const Vector3f& vel = inertial_nav.get_velocity();

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(0.0f);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot's desired yaw rate
        target_yaw_rate_rads = get_pilot_desired_yaw_rate_rad(radians(channel_yaw->get_control_in()*0.01f));

        // get pilot desired climb rate (for alt-hold mode and take-off)
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // check for take-off
        if (ap.land_complete && (takeoff_state.running || target_climb_rate > 0.0f)) {
            if (!takeoff_state.running) {
                takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            }

            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // if landed initialise loiter targets, set throttle to zero and exit
    if (ap.land_complete) {
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        wp_nav.init_loiter_target();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_rad(0, 0, 0, get_smoothing_gain());
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control.update_z_controller();
        return;
    }else{
        // convert pilot input to lean angles
        get_pilot_desired_lean_angles_rad(radians(channel_roll->get_control_in()*0.01f), radians(channel_pitch->get_control_in()*0.01f), target_roll_rad, target_pitch_rad, radians(aparm.angle_max*0.01f));

        // convert inertial nav earth-frame velocities to body-frame
        // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
        vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
        vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();
        
        // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
        if (poshold.roll_mode != POSHOLD_LOITER || poshold.pitch_mode != POSHOLD_LOITER)
        poshold_get_wind_comp_lean_angles_rad(poshold.wind_comp_roll_rad, poshold.wind_comp_pitch_rad);

        // Roll state machine
        //  Each state (aka mode) is responsible for:
        //      1. dealing with pilot input
        //      2. calculating the final roll output to the attitude controller
        //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
        switch (poshold.roll_mode) {

            case POSHOLD_PILOT_OVERRIDE:
                // update pilot desired roll angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle_rad(poshold.pilot_roll_rad, target_roll_rad);

                // switch to BRAKE mode for next iteration if no pilot input
                if (is_zero(target_roll_rad) && (fabsf(poshold.pilot_roll_rad) < 2.0f * radians(g.poshold_brake_rate*0.01f))) {
                    // initialise BRAKE mode
                    poshold.roll_mode = POSHOLD_BRAKE;        // Set brake roll mode
                    poshold.brake_roll_rad = 0.0f;                  // initialise braking angle to zero
                    poshold.brake_angle_max_roll_rad = 0.0f;        // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                    poshold.brake_timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                    poshold.braking_time_updated_roll = false;   // flag the braking time can be re-estimated
                }

                // final lean angle should be pilot input plus wind compensation
                poshold.roll_rad = poshold.pilot_roll_rad + poshold.wind_comp_roll_rad;
                break;

            case POSHOLD_BRAKE:
            case POSHOLD_BRAKE_READY_TO_LOITER:
                // calculate brake_roll angle to counter-act velocity
                poshold_update_brake_angle_from_velocity_rad(poshold.brake_roll_rad, vel_right);

                // update braking time estimate
                if (!poshold.braking_time_updated_roll) {
                    // check if brake angle is increasing
                    if (abs(poshold.brake_roll_rad) >= poshold.brake_angle_max_roll_rad) {
                        poshold.brake_angle_max_roll_rad = abs(poshold.brake_roll_rad);
                    } else {
                        // braking angle has started decreasing so re-estimate braking time
                        poshold.brake_timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(abs(poshold.brake_roll_rad)/(10.0f*radians(g.poshold_brake_rate*0.01f))));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                        poshold.braking_time_updated_roll = true;
                    }
                }

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (poshold.brake_timeout_roll > 50*LOOP_RATE_FACTOR)) {
                    poshold.brake_timeout_roll = 50*LOOP_RATE_FACTOR;
                }

                // reduce braking timer
                if (poshold.brake_timeout_roll > 0) {
                    poshold.brake_timeout_roll--;
                } else {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to POSHOLD_BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the roll and pitch mode switch statements
                    poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                }

                // final lean angle is braking angle + wind compensation angle
                poshold.roll_rad = poshold.brake_roll_rad + poshold.wind_comp_roll_rad;

                // check for pilot input
                if (!is_zero(target_roll_rad)) {
                    // init transition to pilot override
                    poshold_roll_controller_to_pilot_override();
                }
                break;

            case POSHOLD_BRAKE_TO_LOITER:
            case POSHOLD_LOITER:
                // these modes are combined roll-pitch modes and are handled below
                break;

            case POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE:
                // update pilot desired roll angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle_rad(poshold.pilot_roll_rad, target_roll_rad);

                // count-down loiter to pilot timer
                if (poshold.controller_to_pilot_timer_roll > 0) {
                    poshold.controller_to_pilot_timer_roll--;
                } else {
                    // when timer runs out switch to full pilot override for next iteration
                    poshold.roll_mode = POSHOLD_PILOT_OVERRIDE;
                }

                // calculate controller_to_pilot mix ratio
                controller_to_pilot_roll_mix = (float)poshold.controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

                // mix final loiter lean angle and pilot desired lean angles
                poshold.roll_rad = poshold_mix_controls_rad(controller_to_pilot_roll_mix, poshold.controller_final_roll_rad, poshold.pilot_roll_rad + poshold.wind_comp_roll_rad);
                break;
        }

        // Pitch state machine
        //  Each state (aka mode) is responsible for:
        //      1. dealing with pilot input
        //      2. calculating the final pitch output to the attitude contpitcher
        //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
        switch (poshold.pitch_mode) {

            case POSHOLD_PILOT_OVERRIDE:
                // update pilot desired pitch angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle_rad(poshold.pilot_pitch_rad, target_pitch_rad);

                // switch to BRAKE mode for next iteration if no pilot input
                if (is_zero(target_pitch_rad) && (fabsf(poshold.pilot_pitch_rad) < 2.0f * radians(g.poshold_brake_rate*0.01f))) {
                    // initialise BRAKE mode
                    poshold.pitch_mode = POSHOLD_BRAKE;       // set brake pitch mode
                    poshold.brake_pitch_rad = 0.0f;                 // initialise braking angle to zero
                    poshold.brake_angle_max_pitch_rad = 0.0f;       // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                    poshold.brake_timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                    poshold.braking_time_updated_pitch = false;   // flag the braking time can be re-estimated
                }

                // final lean angle should be pilot input plus wind compensation
                poshold.pitch_rad = poshold.pilot_pitch_rad + poshold.wind_comp_pitch_rad;
                break;

            case POSHOLD_BRAKE:
            case POSHOLD_BRAKE_READY_TO_LOITER:
                // calculate brake_pitch angle to counter-act velocity
                poshold_update_brake_angle_from_velocity_rad(poshold.brake_pitch_rad, -vel_fw);

                // update braking time estimate
                if (!poshold.braking_time_updated_pitch) {
                    // check if brake angle is increasing
                    if (abs(poshold.brake_pitch_rad) >= poshold.brake_angle_max_pitch_rad) {
                        poshold.brake_angle_max_pitch_rad = abs(poshold.brake_pitch_rad);
                    } else {
                        // braking angle has started decreasing so re-estimate braking time
                        poshold.brake_timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(abs(poshold.brake_pitch_rad)/(10.0f*radians(g.poshold_brake_rate*0.01f))));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                        poshold.braking_time_updated_pitch = true;
                    }
                }

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (poshold.brake_timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                    poshold.brake_timeout_pitch = 50*LOOP_RATE_FACTOR;
                }

                // reduce braking timer
                if (poshold.brake_timeout_pitch > 0) {
                    poshold.brake_timeout_pitch--;
                } else {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to POSHOLD_BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                    poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                }

                // final lean angle is braking angle + wind compensation angle
                poshold.pitch_rad = poshold.brake_pitch_rad + poshold.wind_comp_pitch_rad;

                // check for pilot input
                if (!is_zero(target_pitch_rad)) {
                    // init transition to pilot override
                    poshold_pitch_controller_to_pilot_override();
                }
                break;

            case POSHOLD_BRAKE_TO_LOITER:
            case POSHOLD_LOITER:
                // these modes are combined pitch-pitch modes and are handled below
                break;

            case POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE:
                // update pilot desired pitch angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle_rad(poshold.pilot_pitch_rad, target_pitch_rad);

                // count-down loiter to pilot timer
                if (poshold.controller_to_pilot_timer_pitch > 0) {
                    poshold.controller_to_pilot_timer_pitch--;
                } else {
                    // when timer runs out switch to full pilot override for next iteration
                    poshold.pitch_mode = POSHOLD_PILOT_OVERRIDE;
                }

                // calculate controller_to_pilot mix ratio
                controller_to_pilot_pitch_mix = (float)poshold.controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

                // mix final loiter lean angle and pilot desired lean angles
                poshold.pitch_rad = poshold_mix_controls_rad(controller_to_pilot_pitch_mix, poshold.controller_final_pitch_rad, poshold.pilot_pitch_rad + poshold.wind_comp_pitch_rad);
                break;
        }

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        //
        // Shared roll & pitch states (POSHOLD_BRAKE_TO_LOITER and POSHOLD_LOITER)
        //

        // switch into LOITER mode when both roll and pitch are ready
        if (poshold.roll_mode == POSHOLD_BRAKE_READY_TO_LOITER && poshold.pitch_mode == POSHOLD_BRAKE_READY_TO_LOITER) {
            poshold.roll_mode = POSHOLD_BRAKE_TO_LOITER;
            poshold.pitch_mode = POSHOLD_BRAKE_TO_LOITER;
            poshold.brake_to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
            // init loiter controller
            wp_nav.init_loiter_target(inertial_nav.get_position(), poshold.loiter_reset_I); // (false) to avoid I_term reset. In original code, velocity(0,0,0) was used instead of current velocity: wp_nav.init_loiter_target(inertial_nav.get_position(), Vector3f(0,0,0));
            // at this stage, we are going to run update_loiter that will reset I_term once. From now, we ensure next time that we will enter loiter and update it, I_term won't be reset anymore
            poshold.loiter_reset_I = false;
            // set delay to start of wind compensation estimate updates
            poshold.wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
        }

        // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
        if (poshold.roll_mode == POSHOLD_BRAKE_TO_LOITER || poshold.roll_mode == POSHOLD_LOITER) {

            // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
            poshold.pitch_mode = poshold.roll_mode;

            // handle combined roll+pitch mode
            switch (poshold.roll_mode) {
                case POSHOLD_BRAKE_TO_LOITER:
                    // reduce brake_to_loiter timer
                    if (poshold.brake_to_loiter_timer > 0) {
                        poshold.brake_to_loiter_timer--;
                    } else {
                        // progress to full loiter on next iteration
                        poshold.roll_mode = POSHOLD_LOITER;
                        poshold.pitch_mode = POSHOLD_LOITER;
                    }

                    // calculate percentage mix of loiter and brake control
                    brake_to_loiter_mix = (float)poshold.brake_to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                    // calculate brake_roll and pitch angles to counter-act velocity
                    poshold_update_brake_angle_from_velocity_rad(poshold.brake_roll_rad, vel_right);
                    poshold_update_brake_angle_from_velocity_rad(poshold.brake_pitch_rad, -vel_fw);

                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

                    // calculate final roll and pitch output by mixing loiter and brake controls
                    poshold.roll_rad = poshold_mix_controls_rad(brake_to_loiter_mix, poshold.brake_roll_rad + poshold.wind_comp_roll_rad, wp_nav.get_roll_rad());
                    poshold.pitch_rad = poshold_mix_controls_rad(brake_to_loiter_mix, poshold.brake_pitch_rad + poshold.wind_comp_pitch_rad, wp_nav.get_pitch_rad());

                    // check for pilot input
                    if (!is_zero(target_roll_rad) || !is_zero(target_pitch_rad)) {
                        // if roll input switch to pilot override for roll
                        if (!is_zero(target_roll_rad)) {
                            // init transition to pilot override
                            poshold_roll_controller_to_pilot_override();
                            // switch pitch-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset poshold.brake_pitch here as wind comp has not been updated since last brake_pitch computation
                            poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                        }
                        // if pitch input switch to pilot override for pitch
                        if (!is_zero(target_pitch_rad)) {
                            // init transition to pilot override
                            poshold_pitch_controller_to_pilot_override();
                            if (is_zero(target_roll_rad)) {
                                // switch roll-mode to brake (but ready to go back to loiter anytime)
                                // no need to reset poshold.brake_roll here as wind comp has not been updated since last brake_roll computation
                                poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                            }
                        }
                    }
                    break;

                case POSHOLD_LOITER:
                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

                    // set roll angle based on loiter controller outputs
                    poshold.roll_rad = wp_nav.get_roll_rad();
                    poshold.pitch_rad = wp_nav.get_pitch_rad();

                    // update wind compensation estimate
                    poshold_update_wind_comp_estimate();

                    // check for pilot input
                    if (!is_zero(target_roll_rad) || !is_zero(target_pitch_rad)) {
                        // if roll input switch to pilot override for roll
                        if (!is_zero(target_roll_rad)) {
                            // init transition to pilot override
                            poshold_roll_controller_to_pilot_override();
                            // switch pitch-mode to brake (but ready to go back to loiter anytime)
                            poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                            // reset brake_pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                            poshold.brake_pitch_rad = 0.0f;
                        }
                        // if pitch input switch to pilot override for pitch
                        if (!is_zero(target_pitch_rad)) {
                            // init transition to pilot override
                            poshold_pitch_controller_to_pilot_override();
                            // if roll not overriden switch roll-mode to brake (but be ready to go back to loiter any time)
                            if (is_zero(target_roll_rad)) {
                                poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                                poshold.brake_roll_rad = 0.0f;
                            }
                        }
                    }
                    break;

                default:
                    // do nothing for uncombined roll and pitch modes
                    break;
            }
        }
        
        // constrain target pitch/roll angles
        poshold.roll_rad = constrain_float(poshold.roll_rad, radians(-aparm.angle_max*0.01f), radians(aparm.angle_max*0.01f));
        poshold.pitch_rad = constrain_float(poshold.pitch_rad, radians(-aparm.angle_max*0.01f), radians(aparm.angle_max*0.01f));

        // update attitude controller targets
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_rad(poshold.roll_rad, poshold.pitch_rad, target_yaw_rate_rads, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }
        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void Copter::poshold_update_pilot_lean_angle_rad(float &lean_angle_filtered_rad, float &lean_angle_raw_rad)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    if ((lean_angle_filtered_rad > 0.0f && lean_angle_raw_rad < 0.0f) || (lean_angle_filtered_rad < 0.0f && lean_angle_raw_rad > 0.0f) || (fabsf(lean_angle_raw_rad) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE_RAD)) {
        lean_angle_filtered_rad = lean_angle_raw_rad;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        if (lean_angle_filtered_rad > 0.0f) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            lean_angle_filtered_rad -= MAX((float)lean_angle_filtered_rad * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1, g.poshold_brake_rate/LOOP_RATE_FACTOR));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered_rad = MAX(lean_angle_filtered_rad, lean_angle_raw_rad);
        }else{
            lean_angle_filtered_rad += MAX(-(float)lean_angle_filtered_rad * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1, g.poshold_brake_rate/LOOP_RATE_FACTOR));
            lean_angle_filtered_rad = MIN(lean_angle_filtered_rad, lean_angle_raw_rad);
        }
    }
}

// poshold_mix_controls - mixes two controls based on the mix_ratio
//  mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
float Copter::poshold_mix_controls_rad(float mix_ratio, float first_control_rad, float second_control_rad)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return ((mix_ratio * first_control_rad) + ((1.0f-mix_ratio)*second_control_rad));
}

// poshold_update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
void Copter::poshold_update_brake_angle_from_velocity_rad(float &brake_angle_rad, float velocity)
{
    float lean_angle_rad;
    float brake_rate_rads = radians(g.poshold_brake_rate*0.01f);

    brake_rate_rads /= 4.0f;
    if (brake_rate_rads <= 0.0f) {
        brake_rate_rads = radians(0.01f);
    }

    // calculate velocity-only based lean angle
    if (velocity >= 0) {
        lean_angle_rad = -poshold.brake_gain_rad * velocity * (1.0f+500.0f/(velocity+60.0f));
    } else {
        lean_angle_rad = -poshold.brake_gain_rad * velocity * (1.0f+500.0f/(-velocity+60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    brake_angle_rad = constrain_float(lean_angle_rad, brake_angle_rad - brake_rate_rads, brake_angle_rad + brake_rate_rads);

    // constrain final brake_angle
    brake_angle_rad = constrain_float(brake_angle_rad, radians(-g.poshold_brake_angle_max*0.01f), radians(g.poshold_brake_angle_max*0.01f));
}

// poshold_update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
void Copter::poshold_update_wind_comp_estimate()
{
    // check wind estimate start has not been delayed
    if (poshold.wind_comp_start_timer > 0) {
        poshold.wind_comp_start_timer--;
        return;
    }

    // check horizontal velocity is low
    if (inertial_nav.get_velocity_xy() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    //  To-Do: clean this up by using accessor in loiter controller (or move entire PosHold controller to a library shared with loiter)
    const Vector3f& accel_target = pos_control.get_accel_target();

    // update wind compensation in earth-frame lean angles
    if (is_zero(poshold.wind_comp_ef.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        poshold.wind_comp_ef.x = accel_target.x;
    } else {
        // low pass filter the position controller's lean angle output
        poshold.wind_comp_ef.x = (1.0f-TC_WIND_COMP)*poshold.wind_comp_ef.x + TC_WIND_COMP*accel_target.x;
    }
    if (is_zero(poshold.wind_comp_ef.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        poshold.wind_comp_ef.y = accel_target.y;
    } else {
        // low pass filter the position controller's lean angle output
        poshold.wind_comp_ef.y = (1.0f-TC_WIND_COMP)*poshold.wind_comp_ef.y + TC_WIND_COMP*accel_target.y;
    }
}

// poshold_get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
void Copter::poshold_get_wind_comp_lean_angles_rad(float &roll_angle_rad, float &pitch_angle_rad)
{
    // reduce rate to 10hz
    poshold.wind_comp_timer++;
    if (poshold.wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    poshold.wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle_rad = atanf((-poshold.wind_comp_ef.x*ahrs.sin_yaw() + poshold.wind_comp_ef.y*ahrs.cos_yaw())/981);
    pitch_angle_rad = atanf(-(poshold.wind_comp_ef.x*ahrs.cos_yaw() + poshold.wind_comp_ef.y*ahrs.sin_yaw())/981);
}

// poshold_roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void Copter::poshold_roll_controller_to_pilot_override()
{
    poshold.roll_mode = POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE;
    poshold.controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_roll to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    poshold.pilot_roll_rad = 0.0f;
    // store final controller output for mixing with pilot input
    poshold.controller_final_roll_rad = poshold.roll_rad;
}

// poshold_pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void Copter::poshold_pitch_controller_to_pilot_override()
{
    poshold.pitch_mode = POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE;
    poshold.controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    poshold.pilot_pitch_rad = 0.0f;
    // store final loiter outputs for mixing with pilot input
    poshold.controller_final_pitch_rad = poshold.pitch_rad;
}

#endif  // POSHOLD_ENABLED == ENABLED
