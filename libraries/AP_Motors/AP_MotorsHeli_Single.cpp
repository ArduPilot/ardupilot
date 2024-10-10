/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli_Single.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Single::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // Indices 1-3 were used by servo position params and should not be used

    // @Param: TAIL_TYPE
    // @DisplayName: Tail Type
    // @Description: Tail type selection. Servo Only uses tail rotor pitch to provide yaw control (including stabilization) via an output assigned to Motor4.  Servo with External Gyro uses an external gyro to control tail rotor pitch via a servo.  Yaw control without stabilization is passed to the external gyro via the output assigned to Motor4.  Direct Drive Variable Pitch(DDVP) is used for tails that have a motor whose ESC is connected to an output with function HeliTailRSC. Tail pitch is still accomplished with a servo on an output assigned to Motor4 function.  Direct Drive Fixed Pitch (DDFP) CW is used for helicopters with a rotor that spins clockwise when viewed from above with a motor whose ESC is controlled by an output whose function is Motor4. Direct Drive Fixed Pitch (DDFP) CCW is used for helicopters with a rotor that spins counter clockwise when viewed from above with a motor whose ESC is controlled by an output whose function is Motor4. In both DDFP cases, no servo is used for the tail and the tail motor esc on Motor4 output is used to control the yaw axis using motor speed.
    // @Values: 0:Servo only,1:Servo with ExtGyro,2:DirectDrive VarPitch,3:DirectDrive FixedPitch CW,4:DirectDrive FixedPitch CCW
    // @User: Standard
    AP_GROUPINFO("TAIL_TYPE", 4, AP_MotorsHeli_Single, _tail_type, float(TAIL_TYPE::SERVO)),

    // Indice 5 was used by SWASH_TYPE and should not be used

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: PWM in microseconds sent to external gyro on an servo/output whose function is Motor7 when tail type is Servo w/ ExtGyro
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN", 6, AP_MotorsHeli_Single, _ext_gyro_gain_std, AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN),

    // Index 7 was used for phase angle and should not be used

    // Indice 8 was used by COLYAW and should not be used

    // @Param: FLYBAR_MODE
    // @DisplayName: Flybar Mode Selector
    // @Description: Flybar present or not.  Affects attitude controller used during ACRO flight mode
    // @Values: 0:NoFlybar,1:Flybar
    // @User: Standard
    AP_GROUPINFO("FLYBAR_MODE", 9, AP_MotorsHeli_Single, _flybar_mode, AP_MOTORS_HELI_NOFLYBAR),

    // @Param: TAIL_SPEED
    // @DisplayName: DDVP Tail ESC speed
    // @Description: Direct drive, variable pitch tail ESC speed in percent output to the tail motor esc (HeliTailRSC Servo) when motor interlock enabled (throttle hold off) and speed fully ramped up after spoolup.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TAIL_SPEED", 10, AP_MotorsHeli_Single, _direct_drive_tailspeed, AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT),

    // @Param: GYR_GAIN_ACRO
    // @DisplayName: ACRO External Gyro Gain
    // @Description: PWM in microseconds sent to external gyro on an servo/output whose function is Motor7 when tail type is Servo w/ ExtGyro in mode ACRO instead of H_GYR_GAIN. A value of zero means to use H_GYR_GAIN
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GYR_GAIN_ACRO", 11, AP_MotorsHeli_Single,  _ext_gyro_gain_acro, 0),

    // Indices 16-19 were used by RSC_PWM_MIN, RSC_PWM_MAX, RSC_PWM_REV, and COL_CTRL_DIR and should not be used

    // @Param: SW_TYPE
    // @DisplayName: Swashplate Type
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic,1:H1 non-CPPM,2:H3_140,3:H3_120,4:H4_90,5:H4_45
    // @User: Standard

    // @Param: SW_COL_DIR
    // @DisplayName: Collective Direction
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard

    // @Param: SW_LIN_SVO
    // @DisplayName: Linearize Swash Servos
    // @Description: This linearizes the swashplate servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard

    // @Param: SW_H3_ENABLE
    // @DisplayName: H3 Generic Enable
    // @Description: Automatically set when H3 generic swash type is selected for swashplate. Do not set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced

    // @Param: SW_H3_SV1_POS
    // @DisplayName: H3 Generic Servo 1 Position
    // @Description: Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW_H3_SV2_POS
    // @DisplayName: H3 Generic Servo 2 Position
    // @Description: Azimuth position on swashplate for servo 2 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW_H3_SV3_POS
    // @DisplayName: H3 Generic Servo 3 Position
    // @Description: Azimuth position on swashplate for servo 3 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW_H3_PHANG
    // @DisplayName: H3 Generic Phase Angle Comp
    // @Description: Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -30 30
    // @Units: deg
    // @User: Advanced
    // @Increment: 1
    AP_SUBGROUPINFO(_swashplate, "SW_", 20, AP_MotorsHeli_Single, AP_MotorsHeli_Swash),

    // @Param: COL2YAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -2 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COL2YAW", 21,  AP_MotorsHeli_Single, _collective_yaw_scale, 0),

    // @Param: DDFP_THST_EXPO
    // @DisplayName: DDFP Tail Rotor Thrust Curve Expo
    // @Description: Tail rotor DDFP motor thrust curve exponent (0.0 for linear to 1.0 for second order curve)
    // @Range: -1 1
    // @User: Standard

    // @Param: DDFP_SPIN_MIN
    // @DisplayName: DDFP Tail Rotor Motor Spin minimum
    // @Description: Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.
    // @Values: 0.0:Low, 0.15:Default, 0.3:High
    // @User: Standard

    // @Param: DDFP_SPIN_MAX
    // @DisplayName: DDFP Tail Rotor Motor Spin maximum
    // @Description: Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
    // @Values: 0.9:Low, 0.95:Default, 1.0:High
    // @User: Standard

    // @Param: DDFP_BAT_IDX
    // @DisplayName: DDFP Tail Rotor Battery compensation index
    // @Description: Which battery monitor should be used for doing compensation
    // @Values: 0:First battery, 1:Second battery
    // @User: Standard

    // @Param: DDFP_BAT_V_MAX
    // @DisplayName: Battery voltage compensation maximum voltage
    // @Description: Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.2 * cell count, 0 = Disabled
    // @Range: 6 53
    // @Units: V
    // @User: Standard

    // @Param: DDFP_BAT_V_MIN
    // @DisplayName: Battery voltage compensation minimum voltage
    // @Description: Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.3 * cell count, 0 = Disabled
    // @Range: 6 42
    // @Units: V
    // @User: Standard
    AP_SUBGROUPINFO(thr_lin, "DDFP_", 22, AP_MotorsHeli_Single, Thrust_Linearization),

    // @Param: YAW_TRIM
    // @DisplayName: Tail Rotor Trim
    // @Description: Fixed offset applied to yaw output to minimize yaw I-term contribution needed to counter rotor drag. Currently only works of DDFP tails (H_TAIL_TYPE = 3 or H_TAIL_TYPE = 4). If using the H_COL2YAW compensation this trim is used to compensate for the main rotor profile drag. If H_COL2YAW is not used, this value can be set to reduce the yaw I contribution to zero when in a steady hover.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("YAW_TRIM", 23,  AP_MotorsHeli_Single, _yaw_trim, 0.0f),

    AP_GROUPEND
};

#define YAW_SERVO_MAX_ANGLE 4500

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Single::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = (1U << AP_MOTORS_MOT_4) | _swashplate.get_output_mask();

    rc_set_freq(mask, _speed_hz);
}

// init_outputs - initialise Servo/PWM ranges and endpoints
void AP_MotorsHeli_Single::init_outputs()
{
    if (!initialised_ok()) {
        // yaw servo
        add_motor_num(CH_4);

        // initialize main rotor servo
        _main_rotor.init_servo();

        switch (get_tail_type()) {
            case TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CW:
            case TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CCW:
                // DDFP tails use range as it is easier to ignore servo trim in making for simple implementation of thrust linearisation.
                SRV_Channels::set_range(SRV_Channel::k_motor4, 1.0f);
                break;

            case TAIL_TYPE::DIRECTDRIVE_VARPITCH:
            case TAIL_TYPE::DIRECTDRIVE_VARPIT_EXT_GOV:
                _tail_rotor.init_servo();
                // yaw servo is an angle from -4500 to 4500
                SRV_Channels::set_angle(SRV_Channel::k_motor4, YAW_SERVO_MAX_ANGLE);
                break;

            case TAIL_TYPE::SERVO_EXTGYRO:
                // external gyro output
                add_motor_num(AP_MOTORS_HELI_SINGLE_EXTGYRO);


                // External Gyro uses PWM output thus servo endpoints are forced
                SRV_Channels::set_output_min_max(SRV_Channels::get_motor_function(AP_MOTORS_HELI_SINGLE_EXTGYRO), 1000, 2000);
                FALLTHROUGH;

            case TAIL_TYPE::SERVO:
            default:
                // yaw servo is an angle from -4500 to 4500
                SRV_Channels::set_angle(SRV_Channel::k_motor4, YAW_SERVO_MAX_ANGLE);
                break;
        }
    }

    set_initialised_ok(_frame_class == MOTOR_FRAME_HELI);
}

// set_desired_rotor_speed
void AP_MotorsHeli_Single::set_desired_rotor_speed(float desired_speed)
{
    _main_rotor.set_desired_speed(desired_speed);

    // always send desired speed to tail rotor control, will do nothing if not DDVP not enabled
    _tail_rotor.set_desired_speed(_direct_drive_tailspeed*0.01f);
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_armed_scalars()
{
    // Set rsc mode specific parameters
    if (_main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_THROTTLECURVE || _main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_AUTOTHROTTLE) {
        _main_rotor.set_throttle_curve();
    }
    // keeps user from changing RSC mode while armed
    if (_main_rotor._rsc_mode.get() != _main_rotor.get_control_mode()) {
        _main_rotor.reset_rsc_mode_param();
        _heliflags.save_rsc_mode = true;
    }
    // saves rsc mode parameter when disarmed if it had been reset while armed
    if (_heliflags.save_rsc_mode && !armed()) {
        _main_rotor._rsc_mode.save();
        _heliflags.save_rsc_mode = false;
    }

    // allow use of external governor autorotation bailout
    if (_heliflags.in_autorotation) {
        _main_rotor.set_autorotation_flag(_heliflags.in_autorotation);
        // set bailout ramp time
        _main_rotor.use_bailout_ramp_time(_heliflags.enable_bailout);
        if (use_tail_RSC()) {
            _tail_rotor.set_autorotation_flag(_heliflags.in_autorotation);
            _tail_rotor.use_bailout_ramp_time(_heliflags.enable_bailout);
        }
    } else {
        _main_rotor.set_autorotation_flag(false);
        if (use_tail_RSC()) {
            _tail_rotor.set_autorotation_flag(false);
        }
    }
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_scalars()
{
    // range check collective min, max and zero
    if( _collective_min >= _collective_max ) {
        _collective_min.set(AP_MOTORS_HELI_COLLECTIVE_MIN);
        _collective_max.set(AP_MOTORS_HELI_COLLECTIVE_MAX);
    }

    _collective_zero_thrust_deg.set(constrain_float(_collective_zero_thrust_deg, _collective_min_deg, _collective_max_deg));

    _collective_land_min_deg.set(constrain_float(_collective_land_min_deg, _collective_min_deg, _collective_max_deg));

    if (!is_equal((float)_collective_max_deg, (float)_collective_min_deg)) {
        // calculate collective zero thrust point as a number from 0 to 1
        _collective_zero_thrust_pct = (_collective_zero_thrust_deg-_collective_min_deg)/(_collective_max_deg-_collective_min_deg);

        // calculate collective land min point as a number from 0 to 1
        _collective_land_min_pct = (_collective_land_min_deg-_collective_min_deg)/(_collective_max_deg-_collective_min_deg);
    } else {
        _collective_zero_thrust_pct = 0.0f;
        _collective_land_min_pct = 0.0f;
    }

    // configure swashplate and update scalars
    _swashplate.configure();

    // send setpoints to main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<RotorControlMode>(_main_rotor._rsc_mode.get()));
    calculate_armed_scalars();

    // send setpoints to DDVP rotor controller and trigger recalculation of scalars
    if (use_tail_RSC()) {
        _tail_rotor.set_control_mode(ROTOR_CONTROL_MODE_SETPOINT);
        _tail_rotor.set_ramp_time(_main_rotor._ramp_time.get());
        _tail_rotor.set_runup_time(_main_rotor._runup_time.get());
        _tail_rotor.set_critical_speed(_main_rotor._critical_speed.get());
        _tail_rotor.set_idle_output(_main_rotor._idle_output.get());
        _tail_rotor.set_arot_idle_output(_main_rotor._arot_idle_output.get());
        _tail_rotor.set_rsc_arot_man_enable(_main_rotor._rsc_arot_man_enable.get());
        _tail_rotor.set_rsc_arot_engage_time(_main_rotor._rsc_arot_engage_time.get());
    } else {
        _tail_rotor.set_control_mode(ROTOR_CONTROL_MODE_DISABLED);
        _tail_rotor.set_ramp_time(0);
        _tail_rotor.set_runup_time(0);
        _tail_rotor.set_critical_speed(0);
        _tail_rotor.set_idle_output(0);
        _tail_rotor.set_arot_idle_output(0);
        _tail_rotor.set_rsc_arot_man_enable(0);
        _tail_rotor.set_rsc_arot_engage_time(0);
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsHeli_Single::get_motor_mask()
{
    return _main_rotor.get_output_mask() | _tail_rotor.get_output_mask();
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Single::update_motor_control(AP_MotorsHeli_RSC::RotorControlState state)
{
    // Send state update to motors
    _tail_rotor.output(state);
    _main_rotor.output(state);

    if (state == AP_MotorsHeli_RSC::RotorControlState::STOP){
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MAX);
    }

    // Check if both rotors are run-up, tail rotor controller always returns true if not enabled
    set_rotor_runup_complete(_main_rotor.is_runup_complete() && _tail_rotor.is_runup_complete());

    // Check if both rotors are spooled down, tail rotor controller always returns true if not enabled
    _heliflags.rotor_spooldown_complete = ( _main_rotor.is_spooldown_complete() );
}

//
// move_actuators - moves swash plate and tail rotor
//                 - expected ranges:
//                       roll : -1 ~ +1
//                       pitch: -1 ~ +1
//                       collective: 0 ~ 1
//                       yaw:   -1 ~ +1
//
void AP_MotorsHeli_Single::move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)
{
    // initialize limits flag
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // rescale roll_out and pitch_out into the min and max ranges to provide linear motion
    // across the input range instead of stopping when the input hits the constrain value
    // these calculations are based on an assumption of the user specified cyclic_max
    // coming into this equation at 4500 or less
    float total_out = norm(pitch_out, roll_out);

    if (total_out > (_cyclic_max/4500.0f)) {
        float ratio = (float)(_cyclic_max/4500.0f) / total_out;
        roll_out *= ratio;
        pitch_out *= ratio;
        limit.roll = true;
        limit.pitch = true;
    }

    // constrain collective input
    float collective_out = coll_in;
    if (collective_out <= 0.0f) {
        collective_out = 0.0f;
        limit.throttle_lower = true;
    }
    if (collective_out >= 1.0f) {
        collective_out = 1.0f;
        limit.throttle_upper = true;
    }

    // ensure not below landed/landing collective
    if (_heliflags.landing_collective && collective_out < _collective_land_min_pct && !_heliflags.in_autorotation) {
        collective_out = _collective_land_min_pct;
        limit.throttle_lower = true;

    }

    // updates below land min collective flag
    if (collective_out <= _collective_land_min_pct) {
        _heliflags.below_land_min_coll = true;
    } else {
        _heliflags.below_land_min_coll = false;
    }

    // updates takeoff collective flag based on 50% hover collective
    update_takeoff_collective_flag(collective_out);

    // Get yaw offset required to cancel out steady state main rotor torque
    const float yaw_offset = get_yaw_offset(collective_out);

    // feed power estimate into main rotor controller
    // ToDo: include tail rotor power?
    // ToDo: add main rotor cyclic power?
    _main_rotor.set_collective(fabsf(collective_out));

    // scale collective pitch for swashplate servos
    float collective_scalar = ((float)(_collective_max-_collective_min))*0.001f;
    float collective_out_scaled = collective_out * collective_scalar + (_collective_min - 1000)*0.001f;

    // Caculate servo positions from swashplate library
    _swashplate.calculate(roll_out, pitch_out, collective_out_scaled);

    // update the yaw rate using the tail rotor/servo
    move_yaw(yaw_out + yaw_offset);
}

// move_yaw
void AP_MotorsHeli_Single::move_yaw(float yaw_out)
{
    // sanity check yaw_out
    if (yaw_out < -1.0f) {
        yaw_out = -1.0f;
        limit.yaw = true;
    }
    if (yaw_out > 1.0f) {
        yaw_out = 1.0f;
        limit.yaw = true;
    }

    _servo4_out = yaw_out;
}

// Get yaw offset required to cancel out steady state main rotor torque
float AP_MotorsHeli_Single::get_yaw_offset(float collective)
{
    if ((get_tail_type() == TAIL_TYPE::SERVO_EXTGYRO) || (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED)) {
        // Not in direct control of tail with external gyro or manual servo mode
        return 0.0;
    }

    if (_heliflags.in_autorotation || (_main_rotor.get_control_output() <= _main_rotor.get_idle_output())) {
        // Motor is stopped or at idle, and thus not creating torque
        return 0.0;
    }

    // sanity check collective_yaw_scale
    _collective_yaw_scale.set(constrain_float(_collective_yaw_scale, -AP_MOTORS_HELI_SINGLE_COLYAW_RANGE, AP_MOTORS_HELI_SINGLE_COLYAW_RANGE));
 
    // This feedforward compensation follows the hover performance theory that hover power required
    // is a function of gross weight to the 3/2 power
    float yaw_offset = _collective_yaw_scale * powf(fabsf(collective - _collective_zero_thrust_pct),1.5f);

    // Add yaw trim for DDFP tails
    if (have_DDFP_tail()) {
        yaw_offset += _yaw_trim.get();
    }

    return yaw_offset;
}

void AP_MotorsHeli_Single::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    // Write swashplate outputs
    _swashplate.output();

    // Output main rotor
    update_motor_control(get_rotor_control_state());

    // Output tail rotor
    switch (get_tail_type()) {
        case TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CCW:
            // Invert output for CCW tail
            _servo4_out *= -1.0;
            FALLTHROUGH;

        case TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CW: {
            // calc filtered battery voltage and lift_max
            thr_lin.update_lift_max_from_batt_voltage();

            // Only throttle up if in active spool state
            switch (_spool_state) {
                case AP_Motors::SpoolState::SHUT_DOWN:
                case AP_Motors::SpoolState::GROUND_IDLE:
                case AP_Motors::SpoolState::SPOOLING_DOWN:
                    // Set DDFP to servo min
                    output_to_ddfp_tail(0.0);
                    break;

                case AP_Motors::SpoolState::SPOOLING_UP:
                case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
                    // Operate DDFP to between DDFP_SPIN_MIN and DDFP_SPIN_MAX using thrust linearisation
                    output_to_ddfp_tail(thr_lin.thrust_to_actuator(_servo4_out));
                    break;
            }
            break;
        }

        case TAIL_TYPE::SERVO_EXTGYRO:
            // output gain to external gyro
            if (_acro_tail && _ext_gyro_gain_acro > 0) {
                rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, 1000 + _ext_gyro_gain_acro);
            } else {
                rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, 1000 + _ext_gyro_gain_std);
            }
            FALLTHROUGH;

        case TAIL_TYPE::SERVO:
        case TAIL_TYPE::DIRECTDRIVE_VARPITCH:
        case TAIL_TYPE::DIRECTDRIVE_VARPIT_EXT_GOV:
        default:
            rc_write_angle(AP_MOTORS_MOT_4, _servo4_out * YAW_SERVO_MAX_ANGLE);
            break;
    }

}

// handle output limit flags and send throttle to servos lib
void AP_MotorsHeli_Single::output_to_ddfp_tail(float throttle)
{
    // Note: yaw trim thrust has already been applied. the output should only be from 0 to 1.
    // Upper limit
    if (throttle >= 1.0){
        throttle = 1.0;
        limit.yaw = true;
    }

    // Lower limit
    if (throttle <= 0.0){
        throttle = 0.0;
        limit.yaw = true;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, throttle);
}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Single::servo_test()
{
    _servo_test_cycle_time += _dt;

    if ((_servo_test_cycle_time >= 0.0f && _servo_test_cycle_time < 0.5f)||                                   // Tilt swash back
        (_servo_test_cycle_time >= 6.0f && _servo_test_cycle_time < 6.5f)){
        _pitch_test += 2.0 * _dt;
        _oscillate_angle += 8 * M_PI * _dt;
        _yaw_test = 0.5f * sinf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 0.5f && _servo_test_cycle_time < 4.5f)||                            // Roll swash around
               (_servo_test_cycle_time >= 6.5f && _servo_test_cycle_time < 10.5f)){
        _oscillate_angle += 0.5 * M_PI * _dt;
        _roll_test = sinf(_oscillate_angle);
        _pitch_test = cosf(_oscillate_angle);
        _yaw_test = sinf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 4.5f && _servo_test_cycle_time < 5.0f)||                            // Return swash to level
               (_servo_test_cycle_time >= 10.5f && _servo_test_cycle_time < 11.0f)){
        _pitch_test -= 2.0 * _dt;
        _oscillate_angle += 8 * M_PI * _dt;
        _yaw_test = 0.5f * sinf(_oscillate_angle);
    } else if (_servo_test_cycle_time >= 5.0f && _servo_test_cycle_time < 6.0f){                              // Raise swash to top
        _collective_test += _dt;
        _oscillate_angle += 2 * M_PI * _dt;
        _yaw_test = sinf(_oscillate_angle);
    } else if (_servo_test_cycle_time >= 11.0f && _servo_test_cycle_time < 12.0f){                            // Lower swash to bottom
        _collective_test -= _dt;
        _oscillate_angle += 2 * M_PI * _dt;
        _yaw_test = sinf(_oscillate_angle);
    } else {                                                                                                  // reset cycle
        _servo_test_cycle_time = 0.0f;
        _oscillate_angle = 0.0f;
        _collective_test = 0.0f;
        _roll_test = 0.0f;
        _pitch_test = 0.0f;
        _yaw_test = 0.0f;
        // decrement servo test cycle counter at the end of the cycle
        if (_servo_test_cycle_counter > 0){
            _servo_test_cycle_counter--;
        }
    }

    // over-ride servo commands to move servos through defined ranges
    _throttle_filter.reset(constrain_float(_collective_test, 0.0f, 1.0f));
    _roll_in = constrain_float(_roll_test, -1.0f, 1.0f);
    _pitch_in = constrain_float(_pitch_test, -1.0f, 1.0f);
    _yaw_in = constrain_float(_yaw_test, -1.0f, 1.0f);
}

// Run arming checks
bool AP_MotorsHeli_Single::arming_checks(size_t buflen, char *buffer) const
{
    // run base class checks
    if (!AP_MotorsHeli::arming_checks(buflen, buffer)) {
        return false;
    }

    // returns false if direct drive tailspeed is outside of range
    if ((_direct_drive_tailspeed < 0) || (_direct_drive_tailspeed > 100)) {
        hal.util->snprintf(buffer, buflen, "H_TAIL_SPEED out of range");
        return false;
    }

    // returns false if Phase Angle is outside of range for H3 swashplate
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H3 && (_swashplate.get_phase_angle() > 30 || _swashplate.get_phase_angle() < -30)){
        hal.util->snprintf(buffer, buflen, "H_SW_H3_PHANG out of range");
        return false;
    }

    // returns false if Acro External Gyro Gain is outside of range
    if ((_ext_gyro_gain_acro < 0) || (_ext_gyro_gain_acro > 1000)) {
        hal.util->snprintf(buffer, buflen, "H_GYR_GAIN_ACRO out of range");
        return false;
    }

    // returns false if Standard External Gyro Gain is outside of range
    if ((_ext_gyro_gain_std < 0) || (_ext_gyro_gain_std > 1000)) {
        hal.util->snprintf(buffer, buflen, "H_GYR_GAIN out of range");
        return false;
    }

    // Run swashplate specific checks
    if (!_swashplate.arming_checks(buflen, buffer)) {
        return false;
    }

    return true;
}

// Helper function for param conversions which are easier to be done in the motors class
// Called from system.cpp
void AP_MotorsHeli_Single::heli_motors_param_conversions(void)
{
    // PARAMETER_CONVERSION - Added: Nov-2023
    // Convert trim for DDFP tails
    // Previous DDFP configs used servo trim for setting the yaw trim, which no longer works with thrust linearisation. Convert servo trim
    // to H_YAW_TRIM. Default thrust linearisation gives linear thrust to throttle relationship to preserve previous setup behaviours so
    // we can assume linear relationship in the conversion.
    if (have_DDFP_tail() && !_yaw_trim.configured()) {

        SRV_Channel *c = SRV_Channels::get_channel_for(SRV_Channel::k_motor4);
        if (c != nullptr) {
            uint16_t pwm_min = c->get_output_min();
            uint16_t pwm_max = c->get_output_max();
            uint16_t pwm_trim = c->get_trim();

            float trim = (float)(pwm_trim - pwm_min) / constrain_uint16(pwm_max - pwm_min, 1, 2000);
            _yaw_trim.set(trim);
        }
        // Motor 4 may not have been assigned to an output yet in which case this is unlikely to be a conversion from old setup.
        // Prevent future attempts to convert the param so we don't put a non-sense value in.
        _yaw_trim.save();
    }
}

// Helper to return true for direct drive fixed pitch tail, either CW or CCW
bool AP_MotorsHeli_Single::have_DDFP_tail() const
{
    const TAIL_TYPE type = get_tail_type();
    return (type == TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CW) ||
           (type == TAIL_TYPE::DIRECTDRIVE_FIXEDPITCH_CCW);
}

// Helper to return true if the tail RSC should be used
bool AP_MotorsHeli_Single::use_tail_RSC() const
{
    const TAIL_TYPE type = get_tail_type();
    return (type == TAIL_TYPE::DIRECTDRIVE_VARPITCH) ||
           (type == TAIL_TYPE::DIRECTDRIVE_VARPIT_EXT_GOV);
}

#if HAL_LOGGING_ENABLED
void AP_MotorsHeli_Single::Log_Write(void)
{
    // Write swash plate logging
    // For single heli we have to apply an additional cyclic scaler of sqrt(2.0) because the
    // definition of when we achieve _cyclic_max is different to dual heli. In single, _cyclic_max
    // is limited at sqrt(2.0), in dual it is limited at 1.0
    float cyclic_angle_scaler = get_cyclic_angle_scaler() * sqrtf(2.0);
    _swashplate.write_log(cyclic_angle_scaler, _collective_min_deg.get(), _collective_max_deg.get(), _collective_min.get(), _collective_max.get());

    // Write RSC logging
    _main_rotor.write_log();
    _tail_rotor.write_log();
}
#endif
