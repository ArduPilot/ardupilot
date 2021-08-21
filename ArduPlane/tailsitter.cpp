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
#include "tailsitter.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

const AP_Param::GroupInfo Tailsitter::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tailsitter
    // @Values: 0:Disable, 1:Enable, 2:Enable Always
    // @Description: This enables Tailsitter functionality. A value of 2 forces Qassist active and always stabilize in forward flight with airmode for stabalisation at 0 throttle, for use on vehicles with no control surfaces, vehicle will not arm in forward flight modes, see also Q_OPTIONS "Mtrs_Only_Qassist"
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tailsitter, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE
    // @DisplayName: Tailsitter fixed wing transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Units: deg
    // @Range: 5 80
    AP_GROUPINFO("ANGLE", 2, Tailsitter, transition_angle_fw, 45),

    // @Param: ANG_VT
    // @DisplayName: Tailsitter VTOL transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from fixed wing control to VTOL control, if zero Q_TAILSIT_ANGLE will be used
    // @Units: deg
    // @Range: 5 80
    AP_GROUPINFO("ANG_VT", 3, Tailsitter, transition_angle_vtol, 0),

    // @Param: INPUT
    // @DisplayName: Tailsitter input type bitmask
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When PlaneMode is not enabled (bit0 = 0) the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When PlaneMode input is enabled, the roll and yaw sticks are swapped so that the roll stick controls earth-frame yaw and rudder controls earth-frame roll. When body-frame roll is enabled (bit1 = 1), the yaw stick controls earth-frame yaw rate and the roll stick controls roll in the tailsitter's body frame when flying level.
    // @Bitmask: 0:PlaneMode,1:BodyFrameRoll
    AP_GROUPINFO("INPUT", 4, Tailsitter, input_type, 0),

    // 5 was MASK
    // 6 was MASKCH

    // @Param: VFGAIN
    // @DisplayName: Tailsitter vector thrust gain in forward flight
    // @Description: This controls the amount of vectored thrust control used in forward flight for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("VFGAIN", 7, Tailsitter, vectored_forward_gain, 0),

    // @Param: VHGAIN
    // @DisplayName: Tailsitter vector thrust gain in hover
    // @Description: This controls the amount of vectored thrust control used in hover for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("VHGAIN", 8, Tailsitter, vectored_hover_gain, 0.5),

    // @Param: VHPOW
    // @DisplayName: Tailsitter vector thrust gain power
    // @Description: This controls the amount of extra pitch given to the vectored control when at high pitch errors
    // @Range: 0 4
    // @Increment: 0.1
    AP_GROUPINFO("VHPOW", 9, Tailsitter, vectored_hover_power, 2.5),

    // @Param: GSCMAX
    // @DisplayName: Maximum tailsitter gain scaling
    // @Description: Maximum gain scaling for tailsitter Q_TAILSIT_GSCMSK options
    // @Range: 1 5
    // @User: Standard
    AP_GROUPINFO("GSCMAX", 10, Tailsitter, throttle_scale_max, 2),

    // @Param: RLL_MX
    // @DisplayName: Maximum Roll angle
    // @Description: Maximum Allowed roll angle for tailsitters. If this is zero then Q_ANGLE_MAX is used.
    // @Units: deg
    // @Range: 0 80
    // @User: Standard
    AP_GROUPINFO("RLL_MX", 11, Tailsitter, max_roll_angle, 0),

    // @Param: MOTMX
    // @DisplayName: Tailsitter motor mask
    // @Description: Bitmask of motors to remain active in forward flight for a 'Copter' tailsitter. Non-zero indicates airframe is a Copter tailsitter and uses copter style motor layouts determined by Q_FRAME_CLASS and Q_FRAME_TYPE. This should be zero for non-Copter tailsitters.
    // @User: Standard
    // @Bitmask: 0:Motor 1,1:Motor 2,2:Motor 3,3:Motor 4, 4:Motor 5,5:Motor 6,6:Motor 7,7:Motor 8
    AP_GROUPINFO("MOTMX", 12, Tailsitter, motor_mask, 0),

    // @Param: GSCMSK
    // @DisplayName: Tailsitter gain scaling mask
    // @Description: Bitmask of gain scaling methods to be applied: Throttle: scale gains with throttle, ATT_THR: reduce gain at high throttle/tilt, 2:Disk theory velocity calculation, requires Q_TAILSIT_DSKLD to be set, ATT_THR must not be set, 3:Altitude correction, scale with air density
    // @User: Standard
    // @Bitmask: 0:Throttle,1:ATT_THR,2:Disk Theory,3:Altitude correction
    AP_GROUPINFO("GSCMSK", 13, Tailsitter, gain_scaling_mask, TAILSITTER_GSCL_THROTTLE),

    // @Param: GSCMIN
    // @DisplayName: Minimum tailsitter gain scaling
    // @Description: Minimum gain scaling for tailsitter Q_TAILSIT_GSCMSK options
    // @Range: 0.1 1
    // @User: Standard
    AP_GROUPINFO("GSCMIN", 14, Tailsitter, gain_scaling_min, 0.4),

    // @Param: DSKLD
    // @DisplayName: Tailsitter disk loading
    // @Description: This is the vehicle weight in kg divided by the total disk area of all propellers in m^2. Only used with Q_TAILSIT_GSCMSK = 4
    // @Units: kg/m/m
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DSKLD", 15, Tailsitter, disk_loading, 0),

    // @Param: RAT_FW
    // @DisplayName: Tailsitter VTOL to forward flight transition rate
    // @Description: The pitch rate at which tailsitter aircraft will pitch down in the transition from VTOL to forward flight
    // @Units: deg/s
    // @Range: 10 500
    AP_GROUPINFO("RAT_FW", 16, Tailsitter, transition_rate_fw, 50),

    // @Param: RAT_VT
    // @DisplayName: Tailsitter forward flight to VTOL transition rate
    // @Description: The pitch rate at which tailsitter aircraft will pitch up in the transition from forward flight to VTOL
    // @Units: deg/s
    // @Range: 10 500
    AP_GROUPINFO("RAT_VT", 17, Tailsitter, transition_rate_vtol, 50),

    // @Param: THR_VT
    // @DisplayName: Tailsitter forward flight to VTOL transition throttle
    // @Description: Throttle used during FW->VTOL transition, -1 uses hover throttle
    // @Units: %
    // @Range: -1 100
    AP_GROUPINFO("THR_VT", 18, Tailsitter, transition_throttle_vtol, -1),

    AP_GROUPEND
};

/*
  defaults for tailsitters
 */
static const struct AP_Param::defaults_table_struct defaults_table_tailsitter[] = {
    { "KFF_RDDRMIX",       0.02 },
    { "Q_A_RAT_PIT_FF",    0.2 },
    { "Q_A_RAT_YAW_FF",    0.2 },
    { "Q_A_RAT_YAW_I",     0.18 },
    { "Q_A_ANGLE_BOOST",   0 },
    { "LIM_PITCH_MAX",    3000 },
    { "LIM_PITCH_MIN",    -3000 },
    { "MIXING_GAIN",      1.0 },
    { "RUDD_DT_GAIN",      10 },
    { "Q_TRANSITION_MS",   2000 },
    { "Q_TRANS_DECEL",    6 },
    { "Q_A_ACCEL_P_MAX",    30000},
    { "Q_A_ACCEL_R_MAX",    30000},
    { "Q_P_POSXY_P",        0.5},
    { "Q_P_VELXY_P",        1.0},
    { "Q_P_VELXY_I",        0.5},
    { "Q_P_VELXY_D",        0.25},
    
};

Tailsitter::Tailsitter(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Tailsitter::setup()
{
    // Set tailsitter enable flag based on old heuristics
    if (!enable.configured() && (((quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) || (motor_mask != 0)) && (quadplane.tiltrotor.type != Tiltrotor::TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (!enabled()) {
        return;
    }

    // Set tailsitter transition rate to match old calculation
    if (!transition_rate_fw.configured()) {
        transition_rate_fw.set_and_save(transition_angle_fw / (quadplane.transition_time_ms/2000.0f));
    }

    // TODO: update this if servo function assignments change
    // used by relax_attitude_control() to control special behavior for vectored tailsitters
    _is_vectored = (quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) &&
                   (!is_zero(vectored_hover_gain) &&
                    (SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft) ||
                     SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight)));

    // set defaults for dual/single motor tailsitter
    if (quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) {
        AP_Param::set_defaults_from_table(defaults_table_tailsitter, ARRAY_SIZE(defaults_table_tailsitter));
    }

    // Setup for control surface less operation
    if (enable == 2) {
        quadplane.q_assist_state = QuadPlane::Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE;
        quadplane.air_mode = AirMode::ASSISTED_FLIGHT_ONLY;

        // Do not allow arming in forward flight modes
        // motors will become active due to assisted flight airmode, the vehicle will try very hard to get level
        quadplane.options.set(quadplane.options.get() | QuadPlane::OPTION_ONLY_ARM_IN_QMODE_OR_AUTO);
    }

}

/*
  return true when flying a control surface only tailsitter
 */
bool Tailsitter::is_control_surface_tailsitter(void) const
{
    return quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER
           && ( is_zero(vectored_hover_gain) || !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));
}

/*
  check if we are flying as a tailsitter
 */
bool Tailsitter::active(void)
{
    if (!enabled()) {
        return false;
    }
    if (quadplane.in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
    if (quadplane.transition_state == QuadPlane::TRANSITION_ANGLE_WAIT_FW) {
        return true;
    }
    return false;
}

/*
  run output for tailsitters
 */
void Tailsitter::output(void)
{
    if (!enabled() || quadplane.motor_test.running || !quadplane.initialised) {
        // if motor test is running we don't want to overwrite it with output_motor_mask or motors_output
        return;
    }

    float tilt_left = 0.0f;
    float tilt_right = 0.0f;

    // throttle 0 to 1
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01;

    // handle forward flight modes and transition to VTOL modes
    if (!active() || in_vtol_transition()) {
        // get FW controller throttle demand and mask of motors enabled during forward flight
        if (hal.util->get_soft_armed() && in_vtol_transition() && !quadplane.throttle_wait && quadplane.is_flying()) {
            /*
              during transitions to vtol mode set the throttle to hover thrust, center the rudder
            */
            if (!is_negative(transition_throttle_vtol)) { 
                // Q_TAILSIT_THR_VT is positive use it until transition is complete
                throttle = motors->actuator_to_thrust(MIN(transition_throttle_vtol*0.01,1.0));
            } else {
                throttle = motors->get_throttle_hover();
                // work out equivelent motors throttle level for cruise
                throttle = MAX(throttle,motors->actuator_to_thrust(plane.aparm.throttle_cruise.get() * 0.01));
            }

            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            plane.rudder_dt = 0;

            // in assisted flight this is done in the normal motor output path
            if (!quadplane.assisted_flight) {
                // convert the hover throttle to the same output that would result if used via AP_Motors
                // apply expo, battery scaling and SPIN min/max.
                throttle = motors->thrust_to_actuator(throttle);

                // override AP_MotorsTailsitter throttles during back transition

                // apply PWM min and MAX to throttle left and right, just as via AP_Motors
                uint16_t throttle_pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * throttle;
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, throttle_pwm);
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, throttle_pwm);

                // throttle output is not used by AP_Motors so might have diffrent PWM range, set scaled
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100.0);
            }
        }

        if (!quadplane.assisted_flight) {
            // set AP_MotorsMatrix throttles for forward flight
            motors->output_motor_mask(throttle, motor_mask, plane.rudder_dt);

            // in forward flight: set motor tilt servos and throttles using FW controller
            if (vectored_forward_gain > 0) {
                // remove scaling from surface speed scaling and apply throttle scaling
                const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
                // thrust vectoring in fixed wing flight
                float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
                float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
                tilt_left  = (elevator + aileron) * vectored_forward_gain * scaler;
                tilt_right = (elevator - aileron) * vectored_forward_gain * scaler;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
            return;
        }
    }

    // handle Copter controller
    // the MultiCopter rate controller has already been run in an earlier call
    // to motors_output() from quadplane.update(), unless we are in assisted flight
    // tailsitter in TRANSITION_ANGLE_WAIT_FW is not really in assisted flight, its still in a VTOL mode
    if (quadplane.assisted_flight && (quadplane.transition_state != QuadPlane::TRANSITION_ANGLE_WAIT_FW)) {
        quadplane.hold_stabilize(throttle);
        quadplane.motors_output(true);

        if ((quadplane.options & QuadPlane::OPTION_TAILSIT_Q_ASSIST_MOTORS_ONLY) != 0) {
            // only use motors for Q assist, control surfaces remain under plane control
            // zero copter I terms and use plane
            quadplane.attitude_control->reset_rate_controller_I_terms();

            // output tilt motors
            tilt_left = 0.0f;
            tilt_right = 0.0f;
            if (vectored_hover_gain > 0) {
                const float hover_throttle = motors->get_throttle_hover();
                const float output_throttle = motors->get_throttle();
                float throttle_scaler = throttle_scale_max;
                if (is_positive(output_throttle)) {
                    throttle_scaler = constrain_float(hover_throttle / output_throttle, gain_scaling_min, throttle_scale_max);
                }
                tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft) * vectored_hover_gain * throttle_scaler;
                tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * vectored_hover_gain * throttle_scaler;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);


            // skip remainder of the function that overwrites plane control surface outputs with copter
            return;
        }
    } else {
        quadplane.motors_output(false);
    }

    // In full Q assist it is better to use copter I and zero plane
    plane.pitchController.reset_I();
    plane.rollController.reset_I();
    plane.yawController.reset_I();

    // pull in copter control outputs
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (motors->get_yaw()+motors->get_yaw_ff())*-SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (motors->get_pitch()+motors->get_pitch_ff())*SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (motors->get_roll()+motors->get_roll_ff())*SERVO_MAX);

    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        speed_scaling();
    }

    tilt_left = 0.0f;
    tilt_right = 0.0f;
    if (vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        float des_pitch_cd = quadplane.attitude_control->get_att_target_euler_cd().y;
        int32_t pitch_error_cd = (des_pitch_cd - quadplane.ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = 0;
        if (!is_zero(extra_pitch) && quadplane.in_vtol_mode()) {
            extra_elevator = extra_sign * powf(fabsf(extra_pitch), vectored_hover_power) * SERVO_MAX;
        }
        tilt_left  = extra_elevator + tilt_left * vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * vectored_hover_gain;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);

    // Check for saturated limits
    bool tilt_lim = _is_vectored && ((labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft)) == SERVO_MAX) || (labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_tiltMotorRight)) == SERVO_MAX));
    bool roll_lim = labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_rudder)) == SERVO_MAX;
    bool pitch_lim = labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_elevator)) == SERVO_MAX;
    bool yaw_lim = labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_aileron)) == SERVO_MAX;

    if (roll_lim) {
        motors->limit.roll = true;
    }
    if (pitch_lim || tilt_lim) {
        motors->limit.pitch = true;
    }
    if (yaw_lim || tilt_lim) {
        motors->limit.yaw = true;
    }

}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool Tailsitter::transition_fw_complete(void)
{
    if (!hal.util->get_soft_armed()) {
        // instant trainsition when disarmed, no message
        return true;
    }
    if (labs(quadplane.ahrs_view->pitch_sensor) > transition_angle_fw*100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        return true;
    }
    if (labs(quadplane.ahrs_view->roll_sensor) > MAX(4500, plane.roll_limit_cd + 500)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition FW done, roll error");
        return true;
    }
    if (AP_HAL::millis() - quadplane.transition_start_ms > ((transition_angle_fw+(quadplane.transition_initial_pitch*0.01f))/transition_rate_fw)*1500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition FW done, timeout");
        return true;
    }
    // still waiting
    return false;
}


/*
  return true when we have completed enough of a transition to switch to VTOL control
 */
bool Tailsitter::transition_vtol_complete(void) const
{
    if (!hal.util->get_soft_armed()) {
        // instant trainsition when disarmed, no message
        return true;
    }
    // for vectored tailsitters at zero pilot throttle
    if ((quadplane.get_pilot_throttle() < .05f) && _is_vectored) {
        // if we are not moving (hence on the ground?) or don't know
        // transition immediately to tilt motors up and prevent prop strikes
        if (quadplane.ahrs.groundspeed() < 1.0f) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done, zero throttle");
            return true;
        }
    }
    const float trans_angle = get_transition_angle_vtol();
    if (labs(plane.ahrs.pitch_sensor) > trans_angle*100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done");
        return true;
    }
    int32_t roll_cd = labs(plane.ahrs.roll_sensor);
    if (plane.fly_inverted()) {
        roll_cd = 18000 - roll_cd;
    }
    if (roll_cd > MAX(4500, plane.roll_limit_cd + 500)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition VTOL done, roll error");
        return true;
    }
    if (AP_HAL::millis() - quadplane.transition_start_ms >  ((trans_angle-(quadplane.transition_initial_pitch*0.01f))/transition_rate_vtol)*1500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition VTOL done, timeout");
        return true;
    }
    return false;
}

// handle different tailsitter input types
void Tailsitter::check_input(void)
{
    if (active() && (input_type & TAILSITTER_INPUT_PLANE)) {
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
bool Tailsitter::in_vtol_transition(uint32_t now) const
{
    if (!enabled() || !quadplane.in_vtol_mode()) {
        return false;
    }
    if (quadplane.transition_state == QuadPlane::TRANSITION_ANGLE_WAIT_VTOL) {
        return true;
    }
    if ((now != 0) && ((now - quadplane.last_vtol_mode_ms) > 1000)) {
        // only just come out of forward flight
        return true;
    }
    return false;
}

/*
  return true if we are a tailsitter in FW flight
 */
bool Tailsitter::is_in_fw_flight(void) const
{
    return enabled() && !quadplane.in_vtol_mode() && quadplane.transition_state == QuadPlane::TRANSITION_DONE;
}

/*
 return the tailsitter.transition_angle_vtol value if non zero, otherwise returns the tailsitter.transition_angle_fw value.
 */
int8_t Tailsitter::get_transition_angle_vtol() const
{
    if (transition_angle_vtol == 0) {
        return transition_angle_fw;
    }
    return transition_angle_vtol;
}


/*
  account for speed scaling of control surfaces in VTOL modes
*/
void Tailsitter::speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float spd_scaler = 1.0f;

    // Scaleing with throttle
    float throttle_scaler = throttle_scale_max;
    if (is_positive(throttle)) {
        throttle_scaler = constrain_float(hover_throttle / throttle, gain_scaling_min, throttle_scale_max);
    }

    if ((gain_scaling_mask & TAILSITTER_GSCL_ATT_THR) != 0) {
        // reduce gains when flying at high speed in Q modes:

        // critical parameter: violent oscillations if too high
        // sudden loss of attitude control if too low
        const float min_scale = gain_scaling_min;
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

        const float c_tilt = quadplane.ahrs_view->get_rotation_body_to_ned().c.z;
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

        // also apply throttle scaling if enabled
        if ((spd_scaler >= 1.0f) && ((gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0)) {
            spd_scaler = MAX(throttle_scaler,1.0f);
        }

    } else if (((gain_scaling_mask & TAILSITTER_GSCL_DISK_THEORY) != 0) && is_positive(disk_loading.get())) {
        // Use disk theory to estimate the velocity over the control surfaces
        // https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html

        float airspeed;
        if (!quadplane.ahrs.airspeed_estimate(airspeed)) {
            // No airspeed estimate, use throttle scaling
            spd_scaler = throttle_scaler;

        } else {


            // use the equation: T = 0.5 * rho * A (Ue^2 - U0^2) solved for Ue^2:
            // Ue^2 = (T / (0.5 * rho *A)) + U0^2
            // We don't know thrust or disk area, use T = (throttle/throttle_hover) * weight
            // ((t / t_h ) * weight) / (0.5 * rho * A) = ((t / t_h) * mass * 9.81) / (0.5 * rho * A)
            // (mass / A) is disk loading DL so:
            // Ue^2 = (((t / t_h) * DL * 9.81)/(0.5 * rho)) + U0^2

            const float rho = SSL_AIR_DENSITY * plane.barometer.get_air_density_ratio();
            float hover_rho = rho;
            if ((gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
                // if applying altitude correction use sea level density for hover case
                hover_rho = SSL_AIR_DENSITY;
            }

            // hover case: (t / t_h) = 1 and U0 = 0
            const float sq_hover_outflow = (disk_loading.get() * GRAVITY_MSS) / (0.5f * hover_rho);


            // calculate the true outflow speed
            const float sq_outflow = (((throttle/hover_throttle) *  disk_loading.get() * GRAVITY_MSS) / (0.5f * rho)) + sq(MAX(airspeed,0));

            // Scale by the ratio of squared hover outflow velocity to squared actual outflow velocity
            spd_scaler = throttle_scale_max;
            if (is_positive(sq_outflow)) {
                spd_scaler = constrain_float(sq_hover_outflow / sq_outflow, gain_scaling_min.get(), throttle_scale_max.get());
            }
        }

    } else if ((gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0) {
        spd_scaler = throttle_scaler;
    }

    if ((gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
        // air density correction
        spd_scaler /= plane.barometer.get_air_density_ratio();
    }

    // record for QTUN log
    log_spd_scaler = spd_scaler;

    const SRV_Channel::Aux_servo_function_t functions[] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator,
        SRV_Channel::Aux_servo_function_t::k_rudder,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorRight};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        if ((functions[i] == SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft) || (functions[i] == SRV_Channel::Aux_servo_function_t::k_tiltMotorRight)) {
            // always apply throttle scaling to tilts
            v *= throttle_scaler;
        } else {
            v *= spd_scaler;
        }
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}

#endif  // HAL_QUADPLANE_ENABLED
