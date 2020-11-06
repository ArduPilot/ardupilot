/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  control code for tailsitters. Enabled by setting Q_FRAME_CLASS=10 
  or by setting Q_TAILSIT_MOTMX nonzero and Q_FRAME_CLASS and Q_FRAME_TYPE
  to a configuration supported by AP_MotorsMatrix
 */

#include <math.h>
#include "Plane.h"

bool tailsitter::setup()
{
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }
    float loop_delta_t = 1.0 / plane.scheduler.get_loop_rate_hz();

    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav) + sizeof(*ahrs_view) + sizeof(*loiter_nav)) {
        AP_BoardConfig::config_error("Not enough memory for quadplane");
    }

    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    enum AP_Motors::motor_frame_class motor_class = (enum AP_Motors::motor_frame_class)frame_class.get();
    switch (motor_class) {
    case AP_Motors::MOTOR_FRAME_QUAD:
        setup_default_channels(4);
        break;
    case AP_Motors::MOTOR_FRAME_HEXA:
        setup_default_channels(6);
        break;
    case AP_Motors::MOTOR_FRAME_OCTA:
    case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        setup_default_channels(8);
        break;
    case AP_Motors::MOTOR_FRAME_Y6:
        setup_default_channels(7);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        SRV_Channels::set_default_function(CH_5, SRV_Channel::k_motor1);
        SRV_Channels::set_default_function(CH_6, SRV_Channel::k_motor2);
        SRV_Channels::set_default_function(CH_8, SRV_Channel::k_motor4);
        SRV_Channels::set_default_function(CH_11, SRV_Channel::k_motor7);
        AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
        break;
    default:
        AP_BoardConfig::config_error("Unknown Q_FRAME_CLASS %u", (unsigned)frame_class.get());
    }

    if (tailsitter_vars.motor_mask == 0) {
        // this is a duo-motor tailsitter (vectored thrust if tilt.tilt_mask != 0)
        motors = new AP_MotorsTailsitter(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsTailsitter::var_info;

    } else {
        // this is a copter tailsitter with motor layout specified by frame_class and frame_type
        motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsMatrix::var_info;
    }

    if (!motors) {
        AP_BoardConfig::config_error("Unable to allocate %s", "motors");
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // create the attitude view used by the VTOL code
    ahrs_view = ahrs.create_view(ROTATION_PITCH_90, ahrs_trim_pitch);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate %s", "ahrs_view");
    }

    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, loop_delta_t);
    if (!attitude_control) {
        AP_BoardConfig::config_error("Unable to allocate %s", "attitude_control");
    }
    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    if (!pos_control) {
        AP_BoardConfig::config_error("Unable to allocate %s", "pos_control");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        AP_BoardConfig::config_error("Unable to allocate %s", "wp_nav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!loiter_nav) {
        AP_BoardConfig::config_error("Unable to allocate %s", "loiter_nav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    motors->init((AP_Motors::motor_frame_class)frame_class.get(), (AP_Motors::motor_frame_type)frame_type.get());

    if (!motors->initialised_ok()) {
        AP_BoardConfig::config_error("unknown Q_FRAME_TYPE %u", (unsigned)frame_type.get());
    }
    motors->set_throttle_range(thr_min_pwm, thr_max_pwm);
    motors->set_update_rate(rc_speed);
    motors->set_interlock(true);
    pos_control->set_dt(loop_delta_t);
    attitude_control->parameter_sanity_check();

    // setup the trim of any motors used by AP_Motors so I/O board
    // failsafe will disable motors
    for (uint8_t i=0; i<8; i++) {
        SRV_Channel::Aux_servo_function_t func = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_failsafe_pwm(func, thr_min_pwm);
    }

    transition_state = TRANSITION_DONE;

    // default QAssist state as set with Q_OPTIONS
    if ((options & OPTION_Q_ASSIST_FORCE_ENABLE) != 0) {
        q_assist_state = Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE;
    }

    setup_defaults();

    // param count will have changed
    AP_Param::invalidate_count();

    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised");
    initialised = true;
    return true;
}

/*
  ask the multicopter attitude control to match the roll and pitch rates being demanded by the
  fixed wing controller if not in a pure VTOL mode
 */
void tailsitter::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    check_attitude_relax();

    // normal control modes for VTOL and FW flight
    // tailsitter in transition to VTOL flight is not really in a VTOL mode yet
    if (in_vtol_mode() && !in_tailsitter_vtol_transition()) {

        // tailsitter-only body-frame roll control options
        // Angle mode attitude control for pitch and body-frame roll, rate control for euler yaw.
        if (tailsitter_vars.input_type & TAILSITTER_INPUT_BF_ROLL) {

            if (!(tailsitter_vars.input_type & TAILSITTER_INPUT_PLANE)) {
                // In multicopter input mode, the roll and yaw stick axes are independent of pitch
                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll(false,
                                                                                plane.nav_roll_cd,
                                                                                plane.nav_pitch_cd,
                                                                                yaw_rate_cds);
            } else {
                // In plane input mode, the roll and yaw sticks are swapped
                // and their effective axes rotate from yaw to roll and vice versa
                // as pitch goes from zero to 90.
                // So it is necessary to also rotate their scaling.

                // Get the roll angle and yaw rate limits
                int16_t roll_limit = aparm.angle_max;
                // separate limit for tailsitter roll, if set
                if (tailsitter_vars.max_roll_angle > 0) {
                    roll_limit = tailsitter_vars.max_roll_angle * 100.0f;
                }
                // Prevent a divide by zero
                float yaw_rate_limit = ((yaw_rate_max < 1.0f) ? 1 : yaw_rate_max) * 100.0f;
                float yaw2roll_scale = roll_limit / yaw_rate_limit;

                // Rotate as a function of Euler pitch and swap roll/yaw
                float euler_pitch = radians(.01f * plane.nav_pitch_cd);
                float spitch = fabsf(sinf(euler_pitch));
                float y2r_scale = linear_interpolate(1, yaw2roll_scale, spitch, 0, 1);

                float p_yaw_rate = plane.nav_roll_cd / y2r_scale;
                float p_roll_angle = -y2r_scale * yaw_rate_cds;

                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll(true,
                                                                                p_roll_angle,
                                                                                plane.nav_pitch_cd,
                                                                                p_yaw_rate);
            }
            return;
        }

        // use euler angle attitude control
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      yaw_rate_cds);
    } else {
        // use the fixed wing desired rates
        float roll_rate = plane.rollController.get_pid_info().target;
        float pitch_rate = plane.pitchController.get_pid_info().target;
        // tailsitter roll and yaw swapped due to change in reference frame
        attitude_control->input_rate_bf_roll_pitch_yaw_2(yaw_rate_cds,pitch_rate*100.0f, -roll_rate*100.0f);
    }
}

// hold in stabilize with given throttle
void tailsitter::hold_stabilize(float throttle_in)
{
    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    if ((throttle_in <= 0) && (air_mode == AirMode::OFF)) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        bool should_boost = true;
        if (assisted_flight) {
            // tailsitters in forward flight should not use angle boost
            should_boost = false;
        }
        attitude_control->set_throttle_out(throttle_in, should_boost, 0);
    }
}

void tailsitter::get_acro_target_roll_yaw(float &target_roll, float &target_yaw) {
    // Note that the 90 degree Y rotation for copter mode swaps body-frame roll and yaw
    target_roll =  plane.channel_rudder->norm_input() * acro_yaw_rate * 100.0f;
    target_yaw  = -plane.channel_roll->norm_input() * acro_roll_rate * 100.0f;
};

bool tailsitter::is_flying()
{
    if (!available()) {
        return false;
    }
    return QuadPlane::is_flying() || in_tailsitter_vtol_transition();
}

bool tailsitter::assistance_needed(float aspeed, bool have_airspeed)
{
    if (is_contol_surface_tailsitter()) {
        // assistance disabled
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    return QuadPlane::assistance_needed(aspeed,have_airspeed);
};

/*
  update for transition from quadplane to fixed wing mode
 */
void tailsitter::update_transition(void)
{
    if (plane.control_mode == &plane.mode_manual ||
        plane.control_mode == &plane.mode_acro ||
        plane.control_mode == &plane.mode_training) {
        transition_state = TRANSITION_DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
        assisted_flight = false;
        return;
    }

    const uint32_t now = millis();

    if (!hal.util->get_soft_armed()) {
        // reset the failure timer if we haven't started transitioning
        transition_start_ms = now;
    }

    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(aspeed);

    // tailsitters use angle wait, not airspeed wait
    if (transition_state == TRANSITION_AIRSPEED_WAIT) {
        transition_state = TRANSITION_ANGLE_WAIT_FW;
    }

    /*
      see if we should provide some assistance
     */
    if (assistance_safe() && (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE ||
        (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED && assistance_needed(aspeed, have_airspeed)))) {
        // the quad should provide some assistance to the plane
        assisted_flight = true;
    } else {
        assisted_flight = false;
    }

    if (transition_state == TRANSITION_ANGLE_WAIT_FW && tailsitter_transition_fw_complete()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        transition_state = TRANSITION_DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
    }

    if (transition_state < TRANSITION_DONE) {
        plane.TECS_controller.set_pitch_max_limit((transition_pitch_max+1)*2);
    }
    if (transition_state < TRANSITION_DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }
    
    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT:
    case TRANSITION_TIMER:
        break; // should never hit this on a tailsitter

    case TRANSITION_ANGLE_WAIT_FW: {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        assisted_flight = true;
        // calculate transition rate in degrees per
        // millisecond. Assume we want to get to the transition angle
        // in half the transition time
        float transition_rate = tailsitter_vars.transition_angle / float(transition_time_ms/2);
        uint32_t dt = now - transition_start_ms;
        float pitch_cd;
        pitch_cd = constrain_float((-transition_rate * dt)*100, -8500, 0);
        // if already pitched forward at start of transition, wait until curve catches up
        plane.nav_pitch_cd = (pitch_cd > transition_initial_pitch)? transition_initial_pitch : pitch_cd;
        plane.nav_roll_cd = 0;
        check_attitude_relax();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      0);
        // set throttle at either hover throttle or current throttle, whichever is higher, through the transition
        attitude_control->set_throttle_out(MAX(motors->get_throttle_hover(),attitude_control->get_throttle_in()), true, 0);
        break;
    }

    case TRANSITION_ANGLE_WAIT_VTOL: // nothing to do, this is handled in the fw attitude controller
    case TRANSITION_DONE:
        return;
    }

    motors_output();
}

/*
  update motor output for quadplane
 */
void tailsitter::update(void)
{
    if (!setup()) {
        return;
    }

    if ((ahrs_view != NULL) && !is_equal(_last_ahrs_trim_pitch, ahrs_trim_pitch.get())) {
        _last_ahrs_trim_pitch = ahrs_trim_pitch.get();
        ahrs_view->set_pitch_trim(_last_ahrs_trim_pitch);
    }

#if ADVANCED_FAILSAFE == ENABLED
    if (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
#endif
    
    if (motor_test.running) {
        motor_test_output();
        return;
    }

    if (SRV_Channels::get_emergency_stop()) {
        attitude_control->reset_rate_controller_I_terms();
    }

    if (!hal.util->get_soft_armed()) {
        /*
          make sure we don't have any residual control from previous flight stages
         */
        // tailsitters only relax I terms, to make ground testing easier
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0);
    }
    
    if (!in_vtol_mode()) {
        update_transition();
    } else {
        const uint32_t now = AP_HAL::millis();

        assisted_flight = false;

        // output to motors
        motors_output();

        if (now - last_vtol_mode_ms > 1000) {
            /*
              we are just entering a VTOL mode as a tailsitter, set
              our transition state so the fixed wing controller brings
              the nose up before we start trying to fly as a
              multicopter
             */
            transition_state = TRANSITION_ANGLE_WAIT_VTOL;
            transition_start_ms = now;
        } else if (transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
            float aspeed;
            bool have_airspeed = ahrs.airspeed_estimate(aspeed);
            // provide asistance in forward flight portion of tailsitter transision
            if (assistance_safe() && (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE ||
                (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED && assistance_needed(aspeed, have_airspeed)))) {
                assisted_flight = true;
            }
            if (tailsitter_transition_vtol_complete()) {
                /*
                  we have completed transition to VTOL as a tailsitter,
                  setup for the back transition when needed
                */
                gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done");
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
            }
        } else {
            /*
              setup the transition state appropriately for next time we go into a non-VTOL mode
            */
            transition_start_ms = 0;
            transition_low_airspeed_ms = 0;
            if (throttle_wait && !plane.is_flying()) {
                transition_state = TRANSITION_DONE;
            } else {
                /*
                  setup for the transition back to fixed wing for later
                 */
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
                transition_initial_pitch= constrain_float(ahrs_view->pitch_sensor,-8500,0);
            }
            last_throttle = motors->get_throttle();
        }

        last_vtol_mode_ms = now;
    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait &&
        (plane.get_throttle_input() > 10 ||
         plane.failsafe.rc_failsafe ||
         plane.failsafe.throttle_counter>0)) {
        throttle_wait = false;
    }
}

// return true if we should show VTOL view
bool tailsitter::show_vtol_view() const
{
    bool show_vtol = in_vtol_mode();

    if (hal.util->get_soft_armed()) {
        if (show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_VTOL)) {
            // in a vtol mode but still transitioning from forward flight
            return false;
        }

        if (!show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_FW)) {
            // not in VTOL mode but still transitioning from VTOL
            return true;
        }
    }

    return show_vtol;
}

/*
  return true when flying a control surface only tailsitter tailsitter
 */
bool tailsitter::is_contol_surface_tailsitter(void) const
{
    return frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER
           && ( is_zero(tailsitter_vars.vectored_hover_gain) || !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));
}

/*
  check if we are flying as a tailsitter
 */
bool tailsitter::tailsitter_active(void) const
{
    if (!is_tailsitter()) {
        return false;
    }
    if (in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
   // if (transition_state == TRANSITION_ANGLE_WAIT_FW) {
   //     return true;
   // }
    return false;
}

/*
  run output for tailsitters
 */
void tailsitter::output(void)
{
    if (!is_tailsitter() || motor_test.running) {
        // if motor test is running we don't want to overwrite it with output_motor_mask or motors_output
        return;
    }

    float tilt_left = 0.0f;
    float tilt_right = 0.0f;



    // handle forward flight modes and transition to VTOL modes
    if (!tailsitter_active() || in_tailsitter_vtol_transition()) {
        // get FW controller throttle demand and mask of motors enabled during forward flight
        float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
        if (hal.util->get_soft_armed() && in_tailsitter_vtol_transition() && !throttle_wait && is_flying()) {
            /*
              during transitions to vtol mode set the throttle to
              hover thrust, center the rudder and set the altitude controller
              integrator to the same throttle level
            */
            throttle = motors->get_throttle_hover() * 100;
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            pos_control->get_accel_z_pid().set_integrator(throttle*10);

            // override AP_MotorsTailsitter throttles during back transition
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle);
        }

        if (!assisted_flight) {
            // set AP_MotorsMatrix throttles for forward flight
            motors->output_motor_mask(throttle * 0.01f, tailsitter_vars.motor_mask, plane.rudder_dt);

            // in forward flight: set motor tilt servos and throttles using FW controller
            if (tailsitter_vars.vectored_forward_gain > 0) {
                // thrust vectoring in fixed wing flight
                float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
                float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
                tilt_left  = (elevator + aileron) * tailsitter_vars.vectored_forward_gain;
                tilt_right = (elevator - aileron) * tailsitter_vars.vectored_forward_gain;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
            return;
        }
    }

    // handle Copter controller
    // the MultiCopter rate controller has already been run in an earlier call
    // to motors_output() from quadplane.update(), unless we are in assisted flight
    if (assisted_flight && is_tailsitter_in_fw_flight()) {
        hold_stabilize(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01f);
        motors_output(true);

        if ((options & OPTION_TAILSIT_Q_ASSIST_MOTORS_ONLY) != 0) {
            // only use motors for Q assist, control surfaces remain under plane control
            // zero copter I terms and use plane
            attitude_control->reset_rate_controller_I_terms();

            // output tilt motors
            if (tailsitter_vars.vectored_hover_gain > 0) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft) * tailsitter_vars.vectored_hover_gain);
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * tailsitter_vars.vectored_hover_gain);
            }

            // skip remainder of the function that overwrites plane control surface outputs with copter
            return;
        }
    } else {
        motors_output(false);
    }

    // In full Q assist it is better to use cotper I and zero plane
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    // pull in copter control outputs
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (motors->get_yaw())*-SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (motors->get_pitch())*SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (motors->get_roll())*SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, (motors->get_throttle()) * 100);

    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        tailsitter_speed_scaling();
    }

    if (tailsitter_vars.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        float des_pitch_cd = attitude_control->get_att_target_euler_cd().y;
        int32_t pitch_error_cd = (des_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = 0;
        if (!is_zero(extra_pitch) && in_vtol_mode()) {
            extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter_vars.vectored_hover_power) * SERVO_MAX;
        }
        tilt_left  = extra_elevator + tilt_left * tailsitter_vars.vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * tailsitter_vars.vectored_hover_gain;
        if (fabsf(tilt_left) >= SERVO_MAX || fabsf(tilt_right) >= SERVO_MAX) {
            // prevent integrator windup
            motors->limit.roll = 1;
            motors->limit.pitch = 1;
            motors->limit.yaw = 1;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    }


    if (tailsitter_vars.input_mask_chan > 0 &&
        tailsitter_vars.input_mask > 0 &&
        RC_Channels::get_radio_in(tailsitter_vars.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter_vars.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter_vars.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter_vars.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        }
        if (tailsitter_vars.input_mask & TAILSITTER_MASK_RUDDER) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());
        }
    }
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool tailsitter::tailsitter_transition_fw_complete(void)
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    int32_t roll_cd = labs(ahrs_view->roll_sensor);
    if (roll_cd > 9000) {
        roll_cd = 18000 - roll_cd;
    }
    if (labs(ahrs_view->pitch_sensor) > tailsitter_vars.transition_angle*100 ||
        roll_cd > tailsitter_vars.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > uint32_t(transition_time_ms)) {
        return true;
    }
    // still waiting
    return false;
}


/*
  return true when we have completed enough of a transition to switch to VTOL control
 */
bool tailsitter::tailsitter_transition_vtol_complete(void) const
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    if (labs(plane.ahrs.pitch_sensor) > tailsitter_vars.transition_angle*100 ||
        labs(plane.ahrs.roll_sensor) > tailsitter_vars.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > 2000) {
        return true;
    }
    // still waiting
    attitude_control->reset_rate_controller_I_terms();
    return false;
}

// handle different tailsitter input types
void tailsitter::check_input(void)
{
    if (tailsitter_active() &&
        (tailsitter_vars.input_type & TAILSITTER_INPUT_PLANE)) {
        // the user has asked for body frame controls when tailsitter
        // is active. We switch around the control_in value for the
        // channels to do this, as that ensures the value is
        // consistent throughout the code
        int16_t roll_in = plane.channel_roll->get_control_in();
        int16_t yaw_in = plane.channel_rudder->get_control_in();
        plane.channel_roll->set_control_in(yaw_in);
        plane.channel_rudder->set_control_in(-roll_in);
    }
}

/*
  return true if we are a tailsitter transitioning to VTOL flight
 */
bool tailsitter::in_tailsitter_vtol_transition(uint32_t now) const
{
    if (!is_tailsitter() || !in_vtol_mode()) {
        return false;
    }
    //if (transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
    //    return true;
   // }
    if ((now != 0) && ((now - last_vtol_mode_ms) > 1000)) {
        // only just come out of forward flight
        return true;
    }
    return false;
}

/*
  return true if we are a tailsitter in FW flight
 */
bool tailsitter::is_tailsitter_in_fw_flight(void) const
{
    return is_tailsitter() && !in_vtol_mode() && transition_state == TRANSITION_DONE;
}

/*
  account for speed scaling of control surfaces in VTOL modes
*/
void tailsitter::tailsitter_speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float spd_scaler = 1.0f;

    if (tailsitter_vars.gain_scaling_mask & TAILSITTER_GSCL_ATT_THR) {
        // reduce gains when flying at high speed in Q modes:

        // critical parameter: violent oscillations if too high
        // sudden loss of attitude control if too low
        const float min_scale = tailsitter_vars.gain_scaling_min;
        float tthr = 1.25f * hover_throttle;

        // reduce control surface throws at large tilt
        // angles (assuming high airspeed)
        // ramp down from 1 to max_atten at tilt angles over trans_angle
        // (angles here are represented by their cosines)

        // Note that the cosf call will be necessary if trans_angle becomes a parameter
        // but the C language spec does not guarantee that trig functions can be used
        // in constant expressions, even though gcc currently allows it.
        constexpr float c_trans_angle = 0.9238795; // cosf(.125f * M_PI)

        // alpha = (1 - max_atten) / (c_trans_angle - cosf(radians(90)));
        const float alpha = (1 - min_scale) / c_trans_angle;
        const float beta = 1 - alpha * c_trans_angle;

        const float c_tilt = ahrs_view->get_rotation_body_to_ned().c.z;
        if (c_tilt < c_trans_angle) {
            spd_scaler = constrain_float(beta + alpha * c_tilt, min_scale, 1.0f);
            // reduce throttle attenuation threshold too
            tthr = 0.5f * hover_throttle;
        }
        // if throttle is above hover thrust, apply additional attenuation
        if (throttle > tthr) {
            const float throttle_atten = 1 - (throttle - tthr) / (1 - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = constrain_float(spd_scaler, min_scale, 1.0f);
        }

        // limit positive and negative slew rates of applied speed scaling
        constexpr float posTC = 2.0f;   // seconds
        constexpr float negTC = 1.0f;   // seconds
        const float posdelta = plane.G_Dt / posTC;
        const float negdelta = plane.G_Dt / negTC;
        spd_scaler = constrain_float(spd_scaler, last_spd_scaler - negdelta, last_spd_scaler + posdelta);
        last_spd_scaler = spd_scaler;
    }

    // if gain attenuation isn't active and boost is enabled
    if ((spd_scaler >= 1.0f) && (tailsitter_vars.gain_scaling_mask & TAILSITTER_GSCL_BOOST)) {
        // boost gains at low throttle
        if (is_zero(throttle)) {
            spd_scaler = tailsitter_vars.throttle_scale_max;
        } else {
            spd_scaler = constrain_float(hover_throttle / throttle, 1.0f, tailsitter_vars.throttle_scale_max);
        }
    }

    // record for QTUN log
    log_spd_scaler = spd_scaler;

    const SRV_Channel::Aux_servo_function_t functions[] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator,
        SRV_Channel::Aux_servo_function_t::k_rudder};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        v *= spd_scaler;
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}
