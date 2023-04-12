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
    // @Description: Tail type selection.  Simpler yaw controller used if external gyro is selected. Direct Drive Variable Pitch is used for tails that have a motor that is governed at constant speed by an ESC.  Tail pitch is still accomplished with a servo.  Direct Drive Fixed Pitch (DDFP) CW is used for helicopters with a rotor that spins clockwise when viewed from above. Direct Drive Fixed Pitch (DDFP) CCW is used for helicopters with a rotor that spins counter clockwise when viewed from above. In both DDFP cases, no servo is used for the tail and the tail motor esc is controlled by the yaw axis.
    // @Values: 0:Servo only,1:Servo with ExtGyro,2:DirectDrive VarPitch,3:DirectDrive FixedPitch CW,4:DirectDrive FixedPitch CCW,5:DDVP with external governor
    // @User: Standard
    AP_GROUPINFO("TAIL_TYPE", 4, AP_MotorsHeli_Single, _tail_type, AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO),

    // Indice 5 was used by SWASH_TYPE and should not be used

    // @Param: GYR_GAIN
    // @DisplayName: External Gyro Gain
    // @Description: PWM in microseconds sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
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
    // @Description: Direct drive, variable pitch tail ESC speed in percent output to the tail motor esc (HeliTailRSC Servo) when motor interlock enabled (throttle hold off).
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TAIL_SPEED", 10, AP_MotorsHeli_Single, _direct_drive_tailspeed, AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT),

    // @Param: GYR_GAIN_ACRO
    // @DisplayName: ACRO External Gyro Gain
    // @Description: PWM in microseconds sent to external gyro on ch7 when tail type is Servo w/ ExtGyro. A value of zero means to use H_GYR_GAIN
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

    AP_GROUPEND
};

#define YAW_SERVO_MAX_ANGLE 4500

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Single::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4;
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << (AP_MOTORS_MOT_5);
    }
    rc_set_freq(mask, _speed_hz);
}

// init_outputs - initialise Servo/PWM ranges and endpoints
bool AP_MotorsHeli_Single::init_outputs()
{
    if (!initialised_ok()) {
        // map primary swash servos
        for (uint8_t i=0; i<AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS; i++) {
            add_motor_num(CH_1+i);
        }
        if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
            add_motor_num(CH_5);
        }

        // yaw servo
        add_motor_num(CH_4);

        // initialize main rotor servo
        _main_rotor.init_servo();

        if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPIT_EXT_GOV) {
            _tail_rotor.init_servo();
        } else if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
            // external gyro output
            add_motor_num(AP_MOTORS_HELI_SINGLE_EXTGYRO);
        }
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        // External Gyro uses PWM output thus servo endpoints are forced
        SRV_Channels::set_output_min_max(SRV_Channels::get_motor_function(AP_MOTORS_HELI_SINGLE_EXTGYRO), 1000, 2000);
    }

    // reset swash servo range and endpoints
    for (uint8_t i=0; i<AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS; i++) {
        reset_swash_servo(SRV_Channels::get_motor_function(i));
    }
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        reset_swash_servo(SRV_Channels::get_motor_function(4));
    }

    // yaw servo is an angle from -4500 to 4500
    SRV_Channels::set_angle(SRV_Channel::k_motor4, YAW_SERVO_MAX_ANGLE);

    set_initialised_ok(_frame_class == MOTOR_FRAME_HELI);

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Single::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // swash servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // swash servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // swash servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // external gyro & tail servo
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
                if (_acro_tail && _ext_gyro_gain_acro > 0) {
                    rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, _ext_gyro_gain_acro);
                } else {
                    rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, _ext_gyro_gain_std);
                }
            }
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // main rotor
            rc_write(AP_MOTORS_HELI_RSC, pwm);
            break;
        default:
            // do nothing
            break;
    }
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
        gcs().send_text(MAV_SEVERITY_CRITICAL, "RSC control mode change failed");
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
        if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPIT_EXT_GOV) {
            _tail_rotor.set_autorotation_flag(_heliflags.in_autorotation);
            _tail_rotor.use_bailout_ramp_time(_heliflags.enable_bailout);
        }
    }else { 
        _main_rotor.set_autorotation_flag(false);
        if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPIT_EXT_GOV) {
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
    _swashplate.calculate_roll_pitch_collective_factors();

    // send setpoints to main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<RotorControlMode>(_main_rotor._rsc_mode.get()));
    calculate_armed_scalars();

    // send setpoints to DDVP rotor controller and trigger recalculation of scalars
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPIT_EXT_GOV) {
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
    // heli uses channels 1,2,3,4 and 8
    // setup fast channels
    uint32_t mask = 1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_RSC;

    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << 4;
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        mask |= 1U << AP_MOTORS_HELI_SINGLE_EXTGYRO;
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPIT_EXT_GOV) {
        mask |= 1U << AP_MOTORS_HELI_SINGLE_TAILRSC;
    }

    return motor_mask_to_srv_channel_mask(mask);
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Single::update_motor_control(RotorControlState state)
{
    // Send state update to motors
    _tail_rotor.output(state);
    _main_rotor.output(state);

    if (state == ROTOR_CONTROL_STOP){
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MAX);
    }

    // Check if both rotors are run-up, tail rotor controller always returns true if not enabled
    _heliflags.rotor_runup_complete = ( _main_rotor.is_runup_complete() && _tail_rotor.is_runup_complete() );

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
    float yaw_offset = 0.0f;

    // initialize limits flag
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    if (_heliflags.inverted_flight) {
        coll_in = 1 - coll_in;
    }

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

    // if servo output not in manual mode and heli is not in autorotation, process pre-compensation factors
    if (_servo_mode == SERVO_CONTROL_MODE_AUTOMATED && !_heliflags.in_autorotation) {
        // rudder feed forward based on collective
        // the feed-forward is not required when the motor is stopped or at idle, and thus not creating torque
        // also not required if we are using external gyro
        if ((_main_rotor.get_control_output() > _main_rotor.get_idle_output()) && _tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
            // sanity check collective_yaw_scale
            _collective_yaw_scale.set(constrain_float(_collective_yaw_scale, -AP_MOTORS_HELI_SINGLE_COLYAW_RANGE, AP_MOTORS_HELI_SINGLE_COLYAW_RANGE));
            // This feedforward compensation follows the hover performance theory that hover power required
            // is a function of gross weight to the 3/2 power
            yaw_offset = _collective_yaw_scale * powf(fabsf(collective_out - _collective_zero_thrust_pct),1.5f);
        }
    } else {
        yaw_offset = 0.0f;
    }

    // feed power estimate into main rotor controller
    // ToDo: include tail rotor power?
    // ToDo: add main rotor cyclic power?
    _main_rotor.set_collective(fabsf(collective_out));

    // scale collective pitch for swashplate servos
    float collective_scalar = ((float)(_collective_max-_collective_min))*0.001f;
    float collective_out_scaled = collective_out * collective_scalar + (_collective_min - 1000)*0.001f;

    // get servo positions from swashplate library
    _servo1_out = _swashplate.get_servo_out(CH_1,pitch_out,roll_out,collective_out_scaled);
    _servo2_out = _swashplate.get_servo_out(CH_2,pitch_out,roll_out,collective_out_scaled);
    _servo3_out = _swashplate.get_servo_out(CH_3,pitch_out,roll_out,collective_out_scaled);
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        _servo5_out = _swashplate.get_servo_out(CH_4,pitch_out,roll_out,collective_out_scaled);
    }

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

void AP_MotorsHeli_Single::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    // actually move the servos.  PWM is sent based on nominal 1500 center.  servo output shifts center based on trim value.
    rc_write_swash(AP_MOTORS_MOT_1, _servo1_out);
    rc_write_swash(AP_MOTORS_MOT_2, _servo2_out);
    rc_write_swash(AP_MOTORS_MOT_3, _servo3_out);
    // get servo positions from swashplate library and write to servo for 4 servo of 4 servo swashplate
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        rc_write_swash(AP_MOTORS_MOT_5, _servo5_out);
    }
    if (_tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CW && _tail_type != AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW){
        rc_write_angle(AP_MOTORS_MOT_4, _servo4_out * YAW_SERVO_MAX_ANGLE);
    }
    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO) {
        // output gain to exernal gyro
        if (_acro_tail && _ext_gyro_gain_acro > 0) {
            rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, 1000 + _ext_gyro_gain_acro);
        } else {
            rc_write(AP_MOTORS_HELI_SINGLE_EXTGYRO, 1000 + _ext_gyro_gain_std);
        }
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW) {
        _servo4_out = -_servo4_out;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            update_motor_control(ROTOR_CONTROL_STOP);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CW || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW){
                rc_write_angle(AP_MOTORS_MOT_4, -YAW_SERVO_MAX_ANGLE);
            }
            break;
        case SpoolState::GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            update_motor_control(ROTOR_CONTROL_IDLE);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CW || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW){
                rc_write_angle(AP_MOTORS_MOT_4, -YAW_SERVO_MAX_ANGLE);
            }
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            update_motor_control(ROTOR_CONTROL_ACTIVE);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CW || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW){
                // constrain output so that motor never fully stops
                 _servo4_out = constrain_float(_servo4_out, -0.9f, 1.0f);
                // output yaw servo to tail rsc
                rc_write_angle(AP_MOTORS_MOT_4, _servo4_out * YAW_SERVO_MAX_ANGLE);
            }
            break;
        case SpoolState::SPOOLING_DOWN:
            // sends idle output to motors and wait for rotor to stop
            update_motor_control(ROTOR_CONTROL_IDLE);
            if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CW || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW){
                rc_write_angle(AP_MOTORS_MOT_4, -YAW_SERVO_MAX_ANGLE);
            }
            break;

    }
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

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli_Single::parameter_check(bool display_msg) const
{
    // returns false if direct drive tailspeed is outside of range
    if ((_direct_drive_tailspeed < 0) || (_direct_drive_tailspeed > 100)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_TAIL_SPEED out of range");
        }
        return false;
    }

    // returns false if Phase Angle is outside of range for H3 swashplate
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H3 && (_swashplate.get_phase_angle() > 30 || _swashplate.get_phase_angle() < -30)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_H3_PHANG out of range");
        }
        return false;
    }

    // returns false if Acro External Gyro Gain is outside of range
    if ((_ext_gyro_gain_acro < 0) || (_ext_gyro_gain_acro > 1000)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_GYR_GAIN_ACRO out of range");
        }
        return false;
    }

    // returns false if Standard External Gyro Gain is outside of range
    if ((_ext_gyro_gain_std < 0) || (_ext_gyro_gain_std > 1000)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_GYR_GAIN out of range");
        }
        return false;
    }

    // check parent class parameters
    return AP_MotorsHeli::parameter_check(display_msg);
}
