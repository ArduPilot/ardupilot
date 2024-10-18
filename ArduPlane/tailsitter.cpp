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
    // @Description: This enables Tailsitter functionality. A value of 2 forces Qassist active and always stabilize in forward flight with airmode for stabilisation at 0 throttle, for use on vehicles with no control surfaces, vehicle will not arm in forward flight modes, see also Q_OPTIONS "Mtrs_Only_Qassist"
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
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
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

    // @Param: VT_R_P
    // @DisplayName: Tailsitter VTOL control surface roll gain
    // @Description: Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains
    // @Range: 0 2
    AP_GROUPINFO("VT_R_P", 19, Tailsitter, VTOL_roll_scale, 1),

    // @Param: VT_P_P
    // @DisplayName: Tailsitter VTOL control surface pitch gain
    // @Description: Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains
    // @Range: 0 2
    AP_GROUPINFO("VT_P_P", 20, Tailsitter, VTOL_pitch_scale, 1),

    // @Param: VT_Y_P
    // @DisplayName: Tailsitter VTOL control surface yaw gain
    // @Description: Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains
    // @Range: 0 2
    AP_GROUPINFO("VT_Y_P", 21, Tailsitter, VTOL_yaw_scale, 1),

    // @Param: MIN_VO
    // @DisplayName: Tailsitter Disk loading minimum outflow speed
    // @Description: Use in conjunction with disk theory gain scaling and Q_TAILSIT_DSKLD to specify minumum airspeed over control surfaces, this will be used to boost throttle, when descending for example, 0 disables
    // @Range: 0 15
    AP_GROUPINFO("MIN_VO", 22, Tailsitter, disk_loading_min_outflow, 0),

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
    { "PTCH_LIM_MAX_DEG",  30 },
    { "PTCH_LIM_MIN_DEG", -30 },
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

    if (enable <= 0) {
        return;
    }

    quadplane.thrust_type = QuadPlane::ThrustType::TAILSITTER;

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

    _have_elevator = SRV_Channels::function_assigned(SRV_Channel::k_elevator);
    _have_aileron = SRV_Channels::function_assigned(SRV_Channel::k_aileron);
    _have_rudder = SRV_Channels::function_assigned(SRV_Channel::k_rudder);
    _have_elevon = SRV_Channels::function_assigned(SRV_Channel::k_elevon_left) || SRV_Channels::function_assigned(SRV_Channel::k_elevon_right);
    _have_v_tail = SRV_Channels::function_assigned(SRV_Channel::k_vtail_left) || SRV_Channels::function_assigned(SRV_Channel::k_vtail_right);

    // set defaults for dual/single motor tailsitter
    if (quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) {
        AP_Param::set_defaults_from_table(defaults_table_tailsitter, ARRAY_SIZE(defaults_table_tailsitter));
    }

    // Setup for control surface less operation
    if (enable == 2) {
        quadplane.assist.set_state(VTOL_Assist::STATE::FORCE_ENABLED);
        quadplane.air_mode = AirMode::ASSISTED_FLIGHT_ONLY;

        // Do not allow arming in forward flight modes
        // motors will become active due to assisted flight airmode, the vehicle will try very hard to get level
        quadplane.options.set(quadplane.options.get() | int32_t(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO));
    }

    transition = NEW_NOTHROW Tailsitter_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tailsitter transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
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
    if (transition->transition_state == Tailsitter_Transition::TRANSITION_ANGLE_WAIT_FW) {
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
        if (plane.arming.is_armed_and_safety_off() && in_vtol_transition() && !quadplane.throttle_wait) {
            /*
              during transitions to vtol mode set the throttle to hover thrust, center the rudder
            */
            if (!is_negative(transition_throttle_vtol)) { 
                // Q_TAILSIT_THR_VT is positive use it until transition is complete
                throttle = motors->thr_lin.actuator_to_thrust(MIN(transition_throttle_vtol*0.01,1.0));
            } else {
                throttle = motors->get_throttle_hover();
                // work out equivelent motors throttle level for cruise
                throttle = MAX(throttle,motors->thr_lin.actuator_to_thrust(plane.aparm.throttle_cruise.get() * 0.01));
            }

            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0.0);
            plane.rudder_dt = 0;

            // in assisted flight this is done in the normal motor output path
            if (!quadplane.assisted_flight) {

                // keep attitude control throttle level upto date, this value should never be output to motors
                // it is used to re-set the accel Z integrator term allowing for a smooth transfer of control
                quadplane.attitude_control->set_throttle_out(throttle, false, 0);

                // convert the hover throttle to the same output that would result if used via AP_Motors
                // apply expo, battery scaling and SPIN min/max.
                throttle = motors->thr_lin.thrust_to_actuator(throttle);

                // override AP_MotorsTailsitter throttles during back transition

                // apply PWM min and MAX to throttle left and right, just as via AP_Motors
                uint16_t throttle_pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * throttle;
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, throttle_pwm);
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, throttle_pwm);

                // throttle output is not used by AP_Motors so might have different PWM range, set scaled
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
    if (quadplane.assisted_flight && (transition->transition_state != Tailsitter_Transition::TRANSITION_ANGLE_WAIT_FW)) {
        quadplane.hold_stabilize(throttle);
        quadplane.motors_output(true);

        if (quadplane.option_is_set(QuadPlane::OPTION::TAILSIT_Q_ASSIST_MOTORS_ONLY)) {
            // only use motors for Q assist, control surfaces remain under plane control. Zero copter I terms and use plane.
            // Smoothly relax to zero so there is no step change in output, must also set limit flags so integrator cannot build faster than the relax.
            // Assume there is always roll control surfaces, otherwise motors only assist should not be set.
            const float dt = quadplane.attitude_control->get_dt();

            // VTOL yaw / FW roll
            quadplane.attitude_control->get_rate_yaw_pid().relax_integrator(0.0, dt, AC_ATTITUDE_RATE_RELAX_TC);
            motors->limit.yaw = true;

            // VTOL and FW pitch
            if (_have_elevator || _have_elevon || _have_v_tail) {
                // have pitch control surfaces, use them
                quadplane.attitude_control->get_rate_pitch_pid().relax_integrator(0.0, dt, AC_ATTITUDE_RATE_RELAX_TC);
                motors->limit.pitch = true;
            } else {
                // no pitch control surfaces, zero plane I terms and use motors
                // We skip the outputting to surfaces for this axis from the copter controller but there are none setup
                plane.pitchController.reset_I();
            }

            // VTOL roll / FW yaw
            if (_have_rudder || _have_v_tail) {
                // there are yaw control  surfaces, zero motor I term
                quadplane.attitude_control->get_rate_roll_pid().relax_integrator(0.0, dt, AC_ATTITUDE_RATE_RELAX_TC);
                motors->limit.roll = true;
            } else {
                // no yaw control surfaces, zero plane I terms and use motors
                // We skip the outputting to surfaces for this axis from the copter controller but there are none setup
                plane.yawController.reset_I();
            }

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
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (motors->get_yaw()+motors->get_yaw_ff())*-SERVO_MAX*VTOL_yaw_scale);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (motors->get_pitch()+motors->get_pitch_ff())*SERVO_MAX*VTOL_pitch_scale);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (motors->get_roll()+motors->get_roll_ff())*SERVO_MAX*VTOL_roll_scale);

    if (plane.arming.is_armed_and_safety_off()) {
        // scale surfaces for throttle
        speed_scaling();
    } else if (tailsitter_motors != nullptr) {
        tailsitter_motors->set_min_throttle(0.0);
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
          reduce effect with roll error
         */
        Quaternion act;
        Vector3f z_unit{0,0,1};
        quadplane.ahrs_view->get_quat_body_to_ned(act);

        // rotate by attitude targets and then back again buy actual, resulting in vector representing roll/pitch error
        quadplane.attitude_control->get_attitude_target_quat().earth_to_body(z_unit);
        act.inverse().earth_to_body(z_unit);

        // equivelent (sort of) to `pitch_error * cos(roll_error)` without the pitfalls of euler angles
        const float pitch_error = degrees(atan2f(z_unit.x,z_unit.z)) * safe_sqrt(1.0 - z_unit.y*z_unit.y) * 50.0;

        const float extra_pitch = constrain_float(pitch_error, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;

        const float extra_sign = extra_pitch > 0?1:-1;
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
    bool tilt_lim = _is_vectored && ((fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft)) >= SERVO_MAX) || (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_tiltMotorRight)) >= SERVO_MAX));
    bool roll_lim = _have_rudder && (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_rudder)) >= SERVO_MAX);
    bool pitch_lim = _have_elevator && (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_elevator)) >= SERVO_MAX);
    bool yaw_lim = _have_aileron && (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_aileron)) >= SERVO_MAX);

    // Mix elevons and V-tail, always giving full priority to pitch
    float elevator_mix = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (100.0 - plane.g.mixing_offset) * 0.01 * plane.g.mixing_gain;
    float aileron_mix = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * (100.0 + plane.g.mixing_offset) * 0.01 * plane.g.mixing_gain;
    float rudder_mix = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * (100.0 + plane.g.mixing_offset) * 0.01 * plane.g.mixing_gain;

    const float headroom = SERVO_MAX - fabsf(elevator_mix);
    if (is_positive(headroom)) {
        if (fabsf(aileron_mix) > headroom) {
            aileron_mix *= headroom / fabsf(aileron_mix);
            yaw_lim |= _have_elevon;
        }
        if (fabsf(rudder_mix) > headroom) {
            rudder_mix *= headroom / fabsf(rudder_mix);
            roll_lim |= _have_v_tail;
        }
    } else {
        aileron_mix = 0.0;
        rudder_mix = 0.0;
        yaw_lim |= _have_elevon;
        pitch_lim |= _have_elevon || _have_v_tail;
        roll_lim |= _have_v_tail;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left, elevator_mix - aileron_mix);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right, elevator_mix + aileron_mix);
    SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right, elevator_mix - rudder_mix);
    SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left, elevator_mix + rudder_mix);

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
    if (!plane.arming.is_armed_and_safety_off()) {
        // instant transition when disarmed, no message
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
    if (AP_HAL::millis() - transition->fw_transition_start_ms > ((transition_angle_fw+(transition->fw_transition_initial_pitch*0.01f))/transition_rate_fw)*1500) {
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
    if (!plane.arming.is_armed_and_safety_off()) {
        // instant transition when disarmed, no message
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
    if (AP_HAL::millis() - transition->vtol_transition_start_ms >  ((trans_angle-(transition->vtol_transition_initial_pitch*0.01f))/transition_rate_vtol)*1500) {
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
    if (transition->transition_state == Tailsitter_Transition::TRANSITION_ANGLE_WAIT_VTOL) {
        return true;
    }
    if ((now != 0) && ((now - transition->last_vtol_mode_ms) > 1000)) {
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
    return enabled() && !quadplane.in_vtol_mode() && transition->transition_state == Tailsitter_Transition::TRANSITION_DONE;
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
    const float throttle = motors->get_throttle_out();
    float spd_scaler = 1.0f;
    float disk_loading_min_throttle = 0.0;

    // Scaling with throttle
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

            const float rho = SSL_AIR_DENSITY * quadplane.ahrs.get_air_density_ratio();
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

            if (is_positive(disk_loading_min_outflow)) {
                // calculate throttle required to give minimum outflow speed over control surfaces
                if (is_positive(airspeed)) {
                    disk_loading_min_throttle = (((sq(disk_loading_min_outflow) - sq(airspeed)) * (0.5 * rho)) / (disk_loading.get() * GRAVITY_MSS)) * hover_throttle;
                } else {
                    // estimate backwards airspeed
                    float reverse_airspeed = 0.0;
                    Vector3f vel;
                    if (quadplane.ahrs.get_velocity_NED(vel)) {
                        reverse_airspeed = quadplane.ahrs.earth_to_body(vel - quadplane.ahrs.wind_estimate()).x;
                    }
                    // make sure actually negative
                    reverse_airspeed = MIN(reverse_airspeed, 0.0);
                    disk_loading_min_throttle = (((sq(disk_loading_min_outflow) + sq(reverse_airspeed)) * (0.5 * rho)) / (disk_loading.get() * GRAVITY_MSS)) * hover_throttle;
                }
                disk_loading_min_throttle = MAX(disk_loading_min_throttle, 0.0);
            }
        }

    } else if ((gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0) {
        spd_scaler = throttle_scaler;
    }

    if ((gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
        // air density correction
        spd_scaler /= quadplane.ahrs.get_air_density_ratio();
    }

    const SRV_Channel::Aux_servo_function_t functions[] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator,
        SRV_Channel::Aux_servo_function_t::k_rudder,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorRight};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        float v = SRV_Channels::get_output_scaled(functions[i]);
        if ((functions[i] == SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft) || (functions[i] == SRV_Channel::Aux_servo_function_t::k_tiltMotorRight)) {
            // always apply throttle scaling to tilts
            v *= throttle_scaler;
        } else {
            v *= spd_scaler;
        }
        SRV_Channels::set_output_scaled(functions[i], v);
    }

    if (tailsitter_motors != nullptr) {
        tailsitter_motors->set_min_throttle(disk_loading_min_throttle);
    }

    // Record for log
    log_data.throttle_scaler = throttle_scaler;
    log_data.speed_scaler = spd_scaler;
    log_data.min_throttle = disk_loading_min_throttle;

}

#if HAL_LOGGING_ENABLED
// Write tailsitter specific log
void Tailsitter::write_log()
{
    if (!enabled()) {
        return;
    }

    struct log_tailsitter pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TSIT_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_scaler     : log_data.throttle_scaler,
        speed_scaler        : log_data.speed_scaler,
        min_throttle        : log_data.min_throttle,
    };
    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif  // HAL_LOGGING_ENABLED

// return true if pitch control should be relaxed
// on vectored belly sitters the pitch control is not relaxed in order to keep motors pointing and avoid risk of props hitting the ground
// always relax after a transition
bool Tailsitter::relax_pitch()
{
    return !enabled() || !_is_vectored || (transition->vtol_limit_start_ms != 0);
}

/*
  update for transition from quadplane to fixed wing mode
 */
void Tailsitter_Transition::update()
{
    const uint32_t now = millis();

    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    quadplane.assisted_flight = quadplane.assist.should_assist(aspeed, have_airspeed);

    if (transition_state < TRANSITION_DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }

    switch (transition_state) {

    case TRANSITION_ANGLE_WAIT_FW: {
        if (tailsitter.transition_fw_complete()) {
            transition_state = TRANSITION_DONE;
            if (plane.arming.is_armed_and_safety_off()) {
                fw_limit_start_ms = now;
                fw_limit_initial_pitch = constrain_float(quadplane.ahrs.pitch_sensor,-8500,8500);
                plane.nav_pitch_cd = fw_limit_initial_pitch;
                plane.nav_roll_cd = 0;
            }
            break;
        }
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        quadplane.assisted_flight = true;
        uint32_t dt = now - fw_transition_start_ms;
        // multiply by 0.1 to convert (degrees/second * milliseconds) to centi degrees
        plane.nav_pitch_cd = constrain_float(fw_transition_initial_pitch - (quadplane.tailsitter.transition_rate_fw * dt) * 0.1f * (plane.fly_inverted()?-1.0f:1.0f), -8500, 8500);
        plane.nav_roll_cd = 0;
        quadplane.disable_yaw_rate_time_constant();
        quadplane.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      0);
        // set throttle at either hover throttle or current throttle, whichever is higher, through the transition
        quadplane.attitude_control->set_throttle_out(MAX(motors->get_throttle_hover(),quadplane.attitude_control->get_throttle_in()), true, 0);
        quadplane.motors_output();
        break;
    }

    case TRANSITION_ANGLE_WAIT_VTOL:
        // nothing to do, this is handled in the fixed wing attitude controller
        break;

    case TRANSITION_DONE:
        break;
    }
}

void Tailsitter_Transition::VTOL_update()
{
    const uint32_t now = AP_HAL::millis();

    if ((now - last_vtol_mode_ms) > 1000) {
        /*
          we are just entering a VTOL mode as a tailsitter, set
          our transition state so the fixed wing controller brings
          the nose up before we start trying to fly as a
          multicopter
         */
        transition_state = TRANSITION_ANGLE_WAIT_VTOL;
    }
    last_vtol_mode_ms = now;

    if (transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
        float aspeed;
        bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
        // provide assistance in forward flight portion of tailsitter transition
        quadplane.assisted_flight = quadplane.assist.should_assist(aspeed, have_airspeed);
        if (!quadplane.tailsitter.transition_vtol_complete()) {
            return;
        }
        // transition to VTOL complete, if armed set vtol rate limit starting point
        if (plane.arming.is_armed_and_safety_off()) {
            vtol_limit_start_ms = now;
            vtol_limit_initial_pitch = quadplane.ahrs_view->pitch_sensor;
        }
    } else {
        // Keep assistance reset while not checking
        quadplane.assist.reset();
    }
    restart();
}

// return true if we should show VTOL view
bool Tailsitter_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_VTOL)) {
        // in a vtol mode but still transitioning from forward flight
        return false;
    }
    if (!show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_FW)) {
        // not in VTOL mode but still transitioning from VTOL
        return true;
    }
    return show_vtol;
}

void Tailsitter_Transition::set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd)
{
    uint32_t now = AP_HAL::millis();
    if (tailsitter.in_vtol_transition(now)) {
        /*
          during transition to vtol in a tailsitter try to raise the
          nose while keeping the wings level
         */
        uint32_t dt = now - vtol_transition_start_ms;
        // multiply by 0.1 to convert (degrees/second * milliseconds) to centi degrees
        nav_pitch_cd = constrain_float(vtol_transition_initial_pitch + (tailsitter.transition_rate_vtol * dt) * 0.1f, -8500, 8500);
        nav_roll_cd = 0;

    } else if (transition_state == TRANSITION_DONE) {
        // still in FW, reset transition starting point
        vtol_transition_start_ms = now;
        vtol_transition_initial_pitch = constrain_float(plane.nav_pitch_cd,-8500,8500);

        // rate limit initial pitch down
        if (fw_limit_start_ms != 0) {
            const float pitch_limit_cd = fw_limit_initial_pitch - (now - fw_limit_start_ms) * tailsitter.transition_rate_fw * 0.1;
            if ((pitch_limit_cd <= 0) || (nav_pitch_cd >= pitch_limit_cd)) {
                // never limit past 0, never limit to a smaller pitch angle
                fw_limit_start_ms = 0;
            } else {
                nav_pitch_cd = pitch_limit_cd;
                nav_roll_cd = 0;
            }
        }
    }
}

bool Tailsitter_Transition::allow_stick_mixing() const
{
    // Transitioning into VTOL flight, inital pitch up
    if (tailsitter.in_vtol_transition()) {
        return false;
    }
    // Transitioning into fixed wing flight, levelling off
    if ((transition_state == TRANSITION_DONE) && (fw_limit_start_ms != 0)) {
        return false;
    }
    return true;
}

bool Tailsitter_Transition::set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd)
{
    if (vtol_limit_start_ms == 0) {
        return false;
    }
    // prevent pitching towards 0 too quickly
    const float pitch_change_cd = (AP_HAL::millis() - vtol_limit_start_ms) * tailsitter.transition_rate_vtol * 0.1;
    if (pitch_change_cd > fabsf(vtol_limit_initial_pitch)) {
        // limit has passed 0, nothing to do
        vtol_limit_start_ms = 0;
        return false;
    }
    // continue limiting while limit angle is larger than desired angle
    if (is_negative(vtol_limit_initial_pitch)) {
        const float pitch_limit = vtol_limit_initial_pitch + pitch_change_cd;
        if (nav_pitch_cd > pitch_limit) {
            nav_pitch_cd = pitch_limit;
            nav_roll_cd = 0;
            return true;
        }
    } else {
        const float pitch_limit = vtol_limit_initial_pitch - pitch_change_cd;
        if (nav_pitch_cd < pitch_limit) {
            nav_pitch_cd = pitch_limit;
            nav_roll_cd = 0;
            return true;
        }
    }
    vtol_limit_start_ms = 0;
    return false;
}

// setup for the transition back to fixed wing
void Tailsitter_Transition::restart()
{
    transition_state = TRANSITION_ANGLE_WAIT_FW;
    fw_transition_start_ms = AP_HAL::millis();
    fw_transition_initial_pitch = constrain_float(quadplane.attitude_control->get_attitude_target_quat().get_euler_pitch() * degrees(100.0),-8500,8500);
}

// force state to FW and setup for the transition back to VTOL
void Tailsitter_Transition::force_transition_complete()
{
    transition_state = TRANSITION_DONE;
    vtol_transition_start_ms = AP_HAL::millis();
    vtol_transition_initial_pitch = constrain_float(plane.nav_pitch_cd,-8500,8500);
    fw_limit_start_ms = 0;

    quadplane.assist.reset();
}

MAV_VTOL_STATE Tailsitter_Transition::get_mav_vtol_state() const
{
    switch (transition_state) {
        case TRANSITION_ANGLE_WAIT_VTOL:
            return MAV_VTOL_STATE_TRANSITION_TO_MC;

        case TRANSITION_DONE:
            return MAV_VTOL_STATE_FW;

        case TRANSITION_ANGLE_WAIT_FW: {
            if (quadplane.in_vtol_mode()) {
                return MAV_VTOL_STATE_MC;
            }
            return MAV_VTOL_STATE_TRANSITION_TO_FW;
        }
    }

    return MAV_VTOL_STATE_UNDEFINED;
}

// only allow to weathervane once transition is complete and desired pitch has been reached
bool Tailsitter_Transition::allow_weathervane()
{
    return !tailsitter.in_vtol_transition() && (vtol_limit_start_ms == 0);
}

#endif  // HAL_QUADPLANE_ENABLED
